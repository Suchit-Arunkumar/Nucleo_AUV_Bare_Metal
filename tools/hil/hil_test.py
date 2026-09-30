#!/usr/bin/env python3
"""
Hardware-in-the-loop test for Nucleo_AUV_Bare_Metal.

The laptop plays the Raspberry Pi. It talks to the NUCLEO-F446RE over the
ST-LINK USB cable (the firmware's default LINK_PORT_STLINK=1 build), sends
the same 62-byte CMD frames the Pi would, and checks what the board sends
back: the console lines and the 50 Hz telemetry frames.

Nothing needs to be connected to the board.

    pip install pyserial
    python hil_test.py                 # auto-detects the ST-LINK port
    python hil_test.py --port COM5     # or name it
    python hil_test.py --no-reset      # skip the boot test (no button press)

Exit code 0 if every test passes. A Markdown report is written next to this
script (hil_report.md) for pasting into the README.
"""
from __future__ import annotations

import argparse
import collections
import datetime as dt
import os
import platform
import random
import statistics
import struct
import subprocess
import sys
import threading
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial is required:  pip install pyserial")

# ---------------------------------------------------------------------------
# Protocol -- must match firmware/Protocol/packet.h and struct.h
# ---------------------------------------------------------------------------
STX1, STX2 = 0xAA, 0x55
PAYLOAD_LEN = 56
PACKET_SIZE = 62
TYPE_TELEMETRY, TYPE_CMD, TYPE_PID = 0x01, 0x02, 0x03

CMD_FMT = "<12f2B6x"            # current xyz rpy, target xyz rpy, armed, seq, reserved[6]
TELEM_FMT = "<2f6f8H3B5x"       # depth, raw_depth, pid_u[6], esc_pwm[8], armed, sat, link, reserved[5]
assert struct.calcsize(CMD_FMT) == PAYLOAD_LEN
assert struct.calcsize(TELEM_FMT) == PAYLOAD_LEN

NEUTRAL_US = 1500
TICK_HZ = 50                    # TIM7
CMD_TIMEOUT_MS = 500            # control_loop.c CMD_TIMEOUT_MS


def crc16(data: bytes) -> int:
    """CRC-16/IBM-3740: poly 0x1021, init 0xFFFF, no reflection, no xorout."""
    c = 0xFFFF
    for b in data:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


assert crc16(b"123456789") == 0x29B1, "CRC-16 implementation does not match IBM-3740"


def f32(x: float) -> float:
    """Round to the float32 the board will store, so values compare exactly."""
    return struct.unpack("<f", struct.pack("<f", x))[0]


def build_frame(ftype: int, payload: bytes, corrupt_crc: bool = False) -> bytes:
    body = bytes([PAYLOAD_LEN, ftype]) + payload
    c = crc16(body)
    if corrupt_crc:
        c ^= 0x0001
    return bytes([STX1, STX2]) + body + bytes([c >> 8, c & 0xFF])


def build_cmd(z: float, armed: int, seq: int, corrupt_crc: bool = False) -> bytes:
    pose = [0.0, 0.0, z, 0.0, 0.0, 0.0]
    target = [0.0] * 6
    payload = struct.pack(CMD_FMT, *pose, *target, armed & 1, seq & 0xFF)
    return build_frame(TYPE_CMD, payload, corrupt_crc)


class Telemetry:
    __slots__ = ("t", "depth", "raw_depth", "pid_u", "esc_pwm", "armed", "sat", "link")

    def __init__(self, t: float, payload: bytes):
        v = struct.unpack(TELEM_FMT, payload)
        self.t = t
        self.depth, self.raw_depth = v[0], v[1]
        self.pid_u = v[2:8]
        self.esc_pwm = v[8:16]
        self.armed, self.sat, self.link = v[16], v[17], v[18]


class StreamDecoder:
    """Split the board's byte stream into console lines and valid frames.

    Same three checks as the firmware parser (sync, LEN + known TYPE, CRC),
    dropping one byte on any failure. Console text never contains 0xAA, and
    the firmware writes lines and frames whole from its main loop, so they
    never interleave inside a frame.
    """

    def __init__(self):
        self.buf = bytearray()
        self.text = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes, t: float):
        self.buf += data
        out = []
        b = self.buf
        while b:
            if b[0] != STX1:
                self._text_byte(b[0], t, out)
                del b[0]
                continue
            if len(b) < 2:
                break
            if b[1] != STX2:
                del b[0]
                continue
            if len(b) < PACKET_SIZE:
                break
            ln, ty = b[2], b[3]
            if ln != PAYLOAD_LEN or ty not in (TYPE_TELEMETRY, TYPE_CMD, TYPE_PID):
                del b[0]
                continue
            if crc16(bytes(b[2:60])) != (b[60] << 8 | b[61]):
                self.crc_errors += 1
                del b[0]
                continue
            out.append(("frame", t, ty, bytes(b[4:60])))
            del b[:PACKET_SIZE]
        return out

    def _text_byte(self, byte: int, t: float, out: list):
        if byte == 0x0A:
            line = self.text.decode("ascii", "replace").strip("\r")
            self.text.clear()
            if line:
                out.append(("line", t, line))
        elif len(self.text) < 200:
            self.text.append(byte)


# ---------------------------------------------------------------------------
# Link: one reader thread, one 50 Hz sender thread
# ---------------------------------------------------------------------------
class Link:
    def __init__(self, port: str, baud: int):
        self.ser = serial.Serial(port, baud, timeout=0.02)
        self.dec = StreamDecoder()
        self.lock = threading.Lock()
        self.telem: list[Telemetry] = []
        self.lines: list[tuple[float, str]] = []
        self.sent: dict[float, float] = {}          # z value -> host time sent
        self.last_valid_send = 0.0
        self.mode = "idle"                          # idle | valid | badcrc | junk
        self.armed = 0
        self.seq = 0
        self._stop = threading.Event()
        self.ser.reset_input_buffer()
        self._rx = threading.Thread(target=self._reader, daemon=True)
        self._tx = threading.Thread(target=self._sender, daemon=True)
        self._rx.start()
        self._tx.start()

    def close(self):
        self.mode = "idle"
        self._stop.set()
        self._tx.join(1)
        self._rx.join(1)
        self.ser.close()

    # --- receive -----------------------------------------------------------
    def _reader(self):
        while not self._stop.is_set():
            try:
                data = self.ser.read(512)
            except serial.SerialException:
                break
            if not data:
                continue
            t = time.monotonic()
            for ev in self.dec.feed(data, t):
                with self.lock:
                    if ev[0] == "line":
                        self.lines.append((ev[1], ev[2]))
                    elif ev[2] == TYPE_TELEMETRY:
                        self.telem.append(Telemetry(ev[1], ev[3]))

    # --- transmit ----------------------------------------------------------
    def next_z(self) -> float:
        # Random, unique, exact float32 values. A regular sequence would let a
        # corrupted value (off by one step) masquerade as a real one.
        while True:
            z = f32(random.uniform(-500.0, 500.0))
            if z not in self.sent and z != -999.0:
                return z

    def _sender(self):
        period = 1.0 / TICK_HZ
        nxt = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            if now < nxt:
                time.sleep(min(nxt - now, 0.005))
                continue
            nxt += period
            if nxt < now - 0.2:                     # fell far behind (sleep granularity): resync
                nxt = now + period
            mode = self.mode
            if mode == "idle":
                continue
            self.seq += 1
            if mode == "badcrc":
                frame = build_cmd(-999.0, self.armed, self.seq, corrupt_crc=True)
                self._write(frame)
                continue
            z = self.next_z()
            frame = build_cmd(z, self.armed, self.seq)
            if mode == "junk":
                self._write_with_junk(frame)
            else:
                self._write(frame)
            t = time.monotonic()
            with self.lock:
                self.sent[z] = t
            self.last_valid_send = t

    def _write(self, data: bytes):
        try:
            self.ser.write(data)
        except serial.SerialException:
            self._stop.set()

    def _write_with_junk(self, frame: bytes):
        r = random.random()
        junk = bytes(random.randrange(256) for _ in range(random.randrange(1, 12)))
        if r < 0.3:
            junk += bytes([STX1, STX2, PAYLOAD_LEN, TYPE_CMD]) + bytes(20)   # decoy header, truncated
        self._write(junk)
        if random.random() < 0.5:                   # split the real frame across writes
            cut = random.randrange(1, PACKET_SIZE)
            self._write(frame[:cut])
            time.sleep(0.002)
            self._write(frame[cut:])
        else:
            self._write(frame)

    # --- queries -----------------------------------------------------------
    def telem_since(self, t0: float) -> list[Telemetry]:
        with self.lock:
            return [x for x in self.telem if x.t >= t0]

    def lines_since(self, t0: float) -> list[tuple[float, str]]:
        with self.lock:
            return [x for x in self.lines if x[0] >= t0]

    def wait_line(self, predicate, timeout: float, t0: float | None = None):
        t0 = time.monotonic() if t0 is None else t0
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            for t, ln in self.lines_since(t0):
                if predicate(ln):
                    return t, ln
            time.sleep(0.02)
        return None


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
class Result:
    def __init__(self, name, proves):
        self.name, self.proves = name, proves
        self.status, self.details = "SKIP", []

    def ok(self, cond: bool, msg: str):
        self.details.append(("PASS" if cond else "FAIL") + ": " + msg)
        if not cond:
            self.status = "FAIL"
        elif self.status == "SKIP":
            self.status = "PASS"
        return cond

    def note(self, msg: str):
        self.details.append("info: " + msg)


def parse_tick(line: str):
    if not line.startswith("tick="):
        return None
    try:
        fields = dict(kv.split("=") for kv in line.split())
        return int(fields["tick"]), int(fields.get("link", -1)), int(fields.get("rxdrop", -1))
    except (ValueError, KeyError):
        return None


def test_boot(link: Link, wait_reset: bool) -> Result:
    r = Result("Boot on a bare board",
               "clock, console, and every init step through the watchdog complete with no sensors attached")
    if not wait_reset:
        r.note("skipped (--no-reset)")
        return r
    print("\n  >>> Press the black RESET button (B2) on the Nucleo now. Waiting 30 s ...", flush=True)
    t0 = time.monotonic()
    hit = link.wait_line(lambda ln: ln == "BOOT OK", 30, t0)
    if not r.ok(hit is not None, "'BOOT OK' received (console readable: clock tree and USART2 baud correct)"):
        return r
    t_boot = hit[0]
    done = link.wait_line(lambda ln: ln == "BOOT DONE", 10, t_boot)
    lines = [ln for t, ln in link.lines_since(t_boot) if done is None or t <= done[0]]
    for ln in lines:
        r.note("console: " + ln)
    r.ok(done is not None, "'BOOT DONE' received: boot ran through iwdg_init() without hanging")
    if done:
        r.note(f"BOOT OK -> BOOT DONE: {1000 * (done[0] - t_boot):.0f} ms (host-measured)")
    r.ok(any(ln.startswith("SD ") for ln in lines), "SD init reported a result instead of hanging")
    r.ok(any(ln.startswith("BAR30 ") for ln in lines), "Bar30 init reported a result instead of hanging")
    r.ok(any(ln.startswith("LINK USART2") for ln in lines), "link running on USART2 (ST-LINK VCP)")
    return r


def test_console_clock(link: Link, seconds: float) -> Result:
    r = Result("Console heartbeat and clock rate",
               "main loop alive; SysTick runs at 1 kHz, i.e. the 180 MHz clock tree is right; no resets")
    t0 = time.monotonic()
    time.sleep(seconds)
    ticks = [(t, parse_tick(ln)) for t, ln in link.lines_since(t0)]
    ticks = [(t, p) for t, p in ticks if p]
    if not r.ok(len(ticks) >= 3, f"{len(ticks)} status lines in {seconds:.0f} s (expect one per 500 ms)"):
        return r
    vals = [p[0] for _, p in ticks]
    r.ok(all(b > a for a, b in zip(vals, vals[1:])), "g_tick strictly increasing (no watchdog or other reset)")
    steps = [b - a for a, b in zip(vals, vals[1:])]
    r.ok(all(500 <= s <= 520 for s in steps), f"line spacing {min(steps)}-{max(steps)} ms of board time (expect 500-501)")
    board_ms = vals[-1] - vals[0]
    host_ms = 1000 * (ticks[-1][0] - ticks[0][0])
    ratio = board_ms / host_ms if host_ms else 0
    r.ok(abs(ratio - 1) < 0.01,
         f"board clock vs laptop clock: {board_ms} ms vs {host_ms:.0f} ms ({100 * (ratio - 1):+.2f} %, limit +/-1 %)")
    r.note("ratio carries a few ms of USB timestamp jitter; a wrong PLL setting shows up as tens of percent")
    return r


def test_telemetry_idle(link: Link, seconds: float) -> Result:
    r = Result("Telemetry at 50 Hz, link idle",
               "TIM7 at 50 Hz; frames framed and CRC'd correctly by the board; idle state is safe")
    link.mode = "idle"
    time.sleep(0.8)                                     # let any earlier command time out
    crc0 = link.dec.crc_errors
    t0 = time.monotonic()
    time.sleep(seconds)
    tl = link.telem_since(t0)
    rate = len(tl) / seconds
    r.ok(48.5 <= rate <= 51.5, f"{len(tl)} frames in {seconds:.0f} s = {rate:.2f} Hz (expect 50)")
    r.ok(link.dec.crc_errors == crc0, f"{link.dec.crc_errors - crc0} frames with a bad CRC")
    r.ok(bool(tl) and all(x.link == 0 for x in tl), "link_ok = 0 with no commands")
    r.ok(bool(tl) and all(x.armed == 0 for x in tl), "armed = 0 with no commands")
    r.ok(bool(tl) and all(all(p == NEUTRAL_US for p in x.esc_pwm) for x in tl), "all 8 ESC values at 1500 us")
    return r


def echo_stats(link: Link, tl: list[Telemetry]):
    """A telemetry depth counts as an echo only if it is exactly a value we
    sent, and we sent it before that telemetry arrived. Anything else is a
    value the board made up or damaged."""
    with link.lock:
        sent = dict(link.sent)
    matched, stray, lat = 0, 0, []
    seen = set()
    for x in tl:
        t_sent = sent.get(x.depth)
        if t_sent is not None and t_sent <= x.t:
            matched += 1
            if x.depth not in seen:
                seen.add(x.depth)
                lat.append(1000 * (x.t - t_sent))
        else:
            stray += 1
    return matched, stray, lat


def test_link_echo(link: Link, seconds: float) -> Result:
    r = Result("Link up and data round trip",
               "board receives, parses and acts on CMD frames; data survives laptop -> board -> laptop exactly")
    link.armed = 0
    t0 = time.monotonic()
    link.mode = "valid"
    up = None
    end = t0 + 2
    while time.monotonic() < end and up is None:
        up = next((x for x in link.telem_since(t0) if x.link == 1), None)
        time.sleep(0.01)
    if not r.ok(up is not None, "telemetry shows link_ok = 1 after commands start"):
        return r
    r.note(f"first CMD sent -> first link_ok=1 telemetry: {1000 * (up.t - t0):.0f} ms")
    t1 = time.monotonic()
    time.sleep(seconds)
    tl = link.telem_since(t1)
    matched, stray, lat = echo_stats(link, tl)
    r.ok(len(tl) > 0 and matched >= 0.95 * len(tl),
         f"{matched}/{len(tl)} telemetry depth values are exact float32 copies of a CMD value we sent")
    r.ok(stray == 0, f"{stray} depth values that were never sent")
    r.ok(all(x.link == 1 for x in tl), "link_ok stays 1 while commands flow at 50 Hz")
    if lat:
        lat.sort()
        r.note(f"round trip (CMD sent -> its value back in telemetry): median {statistics.median(lat):.0f} ms, "
               f"p95 {lat[int(0.95 * (len(lat) - 1))]:.0f} ms, max {lat[-1]:.0f} ms")
    line = link.wait_line(lambda ln: (parse_tick(ln) or (0, 0, 0))[1] == 1, 1.5, t1)
    r.ok(line is not None, "console status line reports link=1")
    return r


def test_arm(link: Link) -> Result:
    r = Result("Arm flag round trip", "the armed bit in CMD reaches the control loop and comes back")
    link.mode = "valid"
    link.armed = 1
    time.sleep(0.3)
    t0 = time.monotonic()
    time.sleep(1.0)
    tl = link.telem_since(t0)
    r.ok(bool(tl) and all(x.armed == 1 for x in tl), f"armed = 1 in all {len(tl)} frames while commanded")
    r.ok(bool(tl) and all(all(p == NEUTRAL_US for p in x.esc_pwm) for x in tl),
         "ESC values stay at 1500 us (all PID gains are zero, so zero thrust is the correct output)")
    link.armed = 0
    time.sleep(0.3)
    t1 = time.monotonic()
    time.sleep(0.5)
    tl = link.telem_since(t1)
    r.ok(bool(tl) and all(x.armed == 0 for x in tl), "armed = 0 again once commanded off")
    return r


def test_junk(link: Link, seconds: float) -> Result:
    r = Result("Resync through junk and split frames",
               "the board's parser recovers from garbage, decoy sync bytes and frames split across writes")
    link.mode = "junk"
    t0 = time.monotonic()
    time.sleep(seconds)
    tl = link.telem_since(t0 + 0.1)
    matched, stray, _ = echo_stats(link, tl)
    r.ok(bool(tl) and all(x.link == 1 for x in tl), f"link_ok stayed 1 across {len(tl)} frames")
    r.ok(len(tl) > 0 and matched >= 0.95 * len(tl), f"{matched}/{len(tl)} depth values still track the commands")
    r.ok(stray == 0, f"{stray} depth values that were never sent")
    link.mode = "valid"
    return r


def test_bad_crc(link: Link) -> Result:
    r = Result("Corrupted frames rejected",
               "the board's CRC check rejects damaged frames: nothing from them is used, and the link times out")
    link.mode = "valid"
    time.sleep(0.5)
    link.mode = "badcrc"
    time.sleep(0.05)                                    # let an in-flight good frame finish
    t_switch = link.last_valid_send
    time.sleep(1.5)
    tl = link.telem_since(t_switch)
    r.ok(all(x.depth != -999.0 for x in tl), "no value from a bad-CRC frame ever appears in telemetry")
    down = next((x for x in tl if x.link == 0), None)
    if r.ok(down is not None, "link_ok drops to 0 while only bad frames arrive"):
        ms = 1000 * (down.t - t_switch)
        r.ok(CMD_TIMEOUT_MS - 20 <= ms <= CMD_TIMEOUT_MS + 200,
             f"dropped {ms:.0f} ms after the last good frame (timeout is {CMD_TIMEOUT_MS} ms)")
    link.mode = "valid"
    return r


def test_failsafe(link: Link, repeats: int = 3) -> Result:
    r = Result("Command-timeout failsafe",
               "if the Pi goes silent, the board disarms and drops the link within the 500 ms timeout")
    times = []
    for i in range(repeats):
        link.armed = 1
        link.mode = "valid"
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2:
            tl = link.telem_since(t0)
            if tl and tl[-1].link == 1 and tl[-1].armed == 1:
                break
            time.sleep(0.02)
        time.sleep(0.3)
        link.mode = "idle"
        time.sleep(0.03)
        t_last = link.last_valid_send
        time.sleep(1.0)
        tl = link.telem_since(t_last)
        down = next((x for x in tl if x.link == 0), None)
        if not r.ok(down is not None, f"run {i + 1}: link_ok went to 0 after commands stopped"):
            continue
        r.ok(down.armed == 0, f"run {i + 1}: armed = 0 in the same frame")
        times.append(1000 * (down.t - t_last))
    link.armed = 0
    if times:
        r.ok(all(CMD_TIMEOUT_MS - 20 <= t <= CMD_TIMEOUT_MS + 200 for t in times),
             "failsafe after " + ", ".join(f"{t:.0f}" for t in times)
             + f" ms (limit {CMD_TIMEOUT_MS} ms + one 20 ms tick + USB latency)")
    return r


def test_soak(link: Link, seconds: float) -> Result:
    r = Result(f"Soak, {seconds:.0f} s at 50 Hz both ways",
               "no dropped bytes, CRC errors, link drops or resets under sustained traffic")
    link.armed = 0
    link.mode = "valid"
    time.sleep(0.5)
    crc0 = link.dec.crc_errors
    t0 = time.monotonic()
    step = max(1.0, seconds / 10)
    while time.monotonic() - t0 < seconds:
        time.sleep(step)
        print(f"    soak {time.monotonic() - t0:4.0f}/{seconds:.0f} s", end="\r", flush=True)
    print(" " * 30, end="\r")
    tl = link.telem_since(t0)
    rate = len(tl) / seconds
    matched, stray, _ = echo_stats(link, tl)
    ticks = [parse_tick(ln) for _, ln in link.lines_since(t0)]
    ticks = [p for p in ticks if p]
    r.ok(48.5 <= rate <= 51.5, f"{len(tl)} telemetry frames = {rate:.2f} Hz")
    r.ok(link.dec.crc_errors == crc0, f"{link.dec.crc_errors - crc0} CRC errors board -> laptop")
    r.ok(bool(tl) and all(x.link == 1 for x in tl), "link never dropped")
    r.ok(stray == 0 and matched >= 0.95 * len(tl), f"{matched}/{len(tl)} depth values echo sent commands")
    if r.ok(len(ticks) >= 2, "status lines kept arriving"):
        vals = [p[0] for p in ticks]
        r.ok(all(b > a for a, b in zip(vals, vals[1:])), "no reset during the soak")
        drops = sorted({p[2] for p in ticks})
        r.ok(drops == [drops[0]] and drops[0] >= 0, f"board rx ring buffer drops: {drops[0]} (unchanged throughout)")
    link.mode = "idle"
    return r


# ---------------------------------------------------------------------------
def find_port() -> str:
    ports = list(serial.tools.list_ports.comports())
    st = [p for p in ports if p.vid == 0x0483 or "STLink" in (p.description or "") or "STM" in (p.manufacturer or "")]
    if len(st) == 1:
        return st[0].device
    listing = "\n".join(f"  {p.device}: {p.description}" for p in ports) or "  (none)"
    if not st:
        sys.exit("No ST-LINK virtual COM port found. Is the Nucleo plugged in? Ports seen:\n"
                 + listing + "\nPass --port explicitly.")
    sys.exit("More than one ST-LINK port found:\n" + listing + "\nPass --port explicitly.")


def firmware_commit() -> str:
    try:
        here = os.path.dirname(os.path.abspath(__file__))
        return subprocess.check_output(["git", "-C", here, "describe", "--always", "--dirty"],
                                       stderr=subprocess.DEVNULL, text=True).strip()
    except Exception:
        return "unknown"


def write_report(path, results, port, commit, crc_errors, started):
    lines = [
        "# Hardware-in-the-loop test report", "",
        f"- Date: {started:%Y-%m-%d %H:%M}",
        f"- Board: NUCLEO-F446RE, nothing attached, link over ST-LINK USB ({port})",
        f"- Firmware commit: `{commit}`",
        f"- Host: {platform.system()} {platform.release()}, Python {platform.python_version()}",
        f"- Board -> laptop CRC errors, whole run: {crc_errors}", "",
        "| Test | Result | What it shows |", "|---|---|---|",
    ]
    for r in results:
        lines.append(f"| {r.name} | **{r.status}** | {r.proves} |")
    lines.append("")
    for r in results:
        lines += [f"## {r.name}: {r.status}", ""] + [f"- {d}" for d in r.details] + [""]
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (default: auto-detect the ST-LINK VCP)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--no-reset", action="store_true", help="skip the boot test (no RESET button press)")
    ap.add_argument("--soak", type=float, default=60, help="soak duration in seconds (0 to skip)")
    ap.add_argument("--report", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "hil_report.md"))
    args = ap.parse_args()

    port = args.port or find_port()
    started = dt.datetime.now()
    print(f"Nucleo HIL test  port={port}  baud={args.baud}  firmware={firmware_commit()}")
    try:
        link = Link(port, args.baud)
    except serial.SerialException as e:
        sys.exit(f"Could not open {port}: {e}\n(Close CubeIDE's serial console or any terminal using it.)")

    results = []
    try:
        steps = [
            lambda: test_boot(link, not args.no_reset),
            lambda: test_console_clock(link, 10),
            lambda: test_telemetry_idle(link, 5),
            lambda: test_link_echo(link, 3),
            lambda: test_arm(link),
            lambda: test_junk(link, 3),
            lambda: test_bad_crc(link),
            lambda: test_failsafe(link),
        ]
        if args.soak > 0:
            steps.append(lambda: test_soak(link, args.soak))
        for step in steps:
            r = step()
            results.append(r)
            print(f"\n[{r.status}] {r.name}")
            for d in r.details:
                print("    " + d)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        crc_errors = link.dec.crc_errors
        link.close()

    write_report(args.report, results, port, firmware_commit(), crc_errors, started)
    ran = [r for r in results if r.status != "SKIP"]
    failed = [r for r in ran if r.status == "FAIL"]
    print(f"\n{len(ran) - len(failed)}/{len(ran)} tests passed"
          + (f", {len(results) - len(ran)} skipped" if len(results) > len(ran) else "")
          + f".  Report: {args.report}")
    sys.exit(1 if failed or not ran else 0)


if __name__ == "__main__":
    main()
