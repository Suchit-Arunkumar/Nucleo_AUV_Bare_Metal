// =============================================================================
// packet.h
// STM32F446 side of the Pi <-> MCU binary packet protocol.
//
// WIRE-COMPATIBLE WITH pico-protocol (firmware/pico_protocol.h). That repo is
// the reference: it is what runs on the vehicle (RP2350) and what the Pi-side
// Python parser expects. Any change here that is not also made there breaks
// the link. Do not "improve" the framing on this side alone.
//
// Wire format:
//   [ STX1 ][ STX2 ][ LEN ][ TYPE ][ ... PAYLOAD (56 bytes) ... ][ CRC_HI ][ CRC_LO ]
//     0xAA    0x55     1B     1B              56 bytes                1B       1B
//
//   Payload: packed structs from struct.h, little-endian (native Cortex-M4).
//   CRC    : CRC-16/IBM-3740 ("CRC-16/CCITT-FALSE"), poly 0x1021, init 0xFFFF,
//            no reflection, no final XOR. Check value of "123456789" = 0x29B1.
//            Covers [LEN, TYPE, PAYLOAD] = 58 bytes; excludes the sync bytes.
//            Transmitted big-endian: CRC_HI first.
//   Total  : 4 + 56 + 2 = 62 bytes.
//
// The STM32F4 hardware CRC unit is NOT used for this link: on the F446 it is
// fixed at CRC-32 (poly 0x04C11DB7) with 32-bit word input and cannot produce
// CRC-16. crc_hw.c is still used by the SD logger, which is a separate format.
// =============================================================================

#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>
#include <stddef.h>   // offsetof, size_t
#include "struct.h"   // TelemetryPayload, CommandPayload, PAYLOAD_LEN

// =============================================================================
// FRAMING
// =============================================================================
#define STX1         0xAA
#define STX2         0x55

// PACKET_SIZE is a literal, not derived: it is the on-wire number and every
// other size is checked against it. Deriving it would make the checks vacuous.
#define HEADER_SIZE  4      // STX1 STX2 LEN TYPE
#define CRC_SIZE     2      // CRC_HI CRC_LO
#define PACKET_SIZE  62     // total bytes on the wire

#define CRC_OFFSET   (HEADER_SIZE + PAYLOAD_LEN)   // byte 60

_Static_assert(PAYLOAD_LEN == 56, "PAYLOAD_LEN in struct.h no longer matches the link");
_Static_assert(HEADER_SIZE + PAYLOAD_LEN + CRC_SIZE == PACKET_SIZE,
               "framing constants do not add up to the on-wire packet size");

#define TYPE_TELEMETRY  0x01   // MCU -> Pi : telemetry snapshot
#define TYPE_CMD        0x02   // Pi  -> MCU: fused nav state + target
#define TYPE_PID        0x03   // Pi  -> MCU: live PID gains (not handled on
                               //             this board; framed and skipped)

// =============================================================================
// WIRE LAYOUT PINS
// Same offsets as pico_protocol.h. A struct that still totals 56 bytes but has
// a moved field produces a CRC-valid frame that decodes to wrong values; these
// turn that into a build failure.
// =============================================================================
_Static_assert(sizeof(TelemetryPayload) == PAYLOAD_LEN, "TelemetryPayload size");
_Static_assert(offsetof(TelemetryPayload, depth_m)     ==  0, "telemetry.depth_m moved");
_Static_assert(offsetof(TelemetryPayload, raw_depth_m) ==  4, "telemetry.raw_depth_m moved");
_Static_assert(offsetof(TelemetryPayload, pid_u)       ==  8, "telemetry.pid_u moved");
_Static_assert(offsetof(TelemetryPayload, esc_pwm)     == 32, "telemetry.esc_pwm moved");
_Static_assert(offsetof(TelemetryPayload, armed)       == 48, "telemetry.armed moved");
_Static_assert(offsetof(TelemetryPayload, sat_flags)   == 49, "telemetry.sat_flags moved");
_Static_assert(offsetof(TelemetryPayload, link_ok)     == 50, "telemetry.link_ok moved");
_Static_assert(offsetof(TelemetryPayload, reserved)    == 51, "telemetry.reserved moved");

_Static_assert(sizeof(CommandPayload) == PAYLOAD_LEN, "CommandPayload size");
_Static_assert(offsetof(CommandPayload, current_x)     ==  0, "cmd.current_x moved");
_Static_assert(offsetof(CommandPayload, current_y)     ==  4, "cmd.current_y moved");
_Static_assert(offsetof(CommandPayload, current_z)     ==  8, "cmd.current_z moved");
_Static_assert(offsetof(CommandPayload, current_roll)  == 12, "cmd.current_roll moved");
_Static_assert(offsetof(CommandPayload, current_pitch) == 16, "cmd.current_pitch moved");
_Static_assert(offsetof(CommandPayload, current_yaw)   == 20, "cmd.current_yaw moved");
_Static_assert(offsetof(CommandPayload, target_x)      == 24, "cmd.target_x moved");
_Static_assert(offsetof(CommandPayload, target_y)      == 28, "cmd.target_y moved");
_Static_assert(offsetof(CommandPayload, target_z)      == 32, "cmd.target_z moved");
_Static_assert(offsetof(CommandPayload, target_roll)   == 36, "cmd.target_roll moved");
_Static_assert(offsetof(CommandPayload, target_pitch)  == 40, "cmd.target_pitch moved");
_Static_assert(offsetof(CommandPayload, target_yaw)    == 44, "cmd.target_yaw moved");
_Static_assert(offsetof(CommandPayload, armed)         == 48, "cmd.armed moved");
_Static_assert(offsetof(CommandPayload, seq)           == 49, "cmd.seq moved");
_Static_assert(offsetof(CommandPayload, reserved)      == 50, "cmd.reserved moved");

// =============================================================================
// API
// =============================================================================

// CRC-16/IBM-3740 over n bytes. Exposed so a self-test can check it against
// the published check value: packet_crc16("123456789", 9) must equal 0x29B1.
uint16_t packet_crc16(const uint8_t *data, size_t n);

// Fills out_buf[PACKET_SIZE] with a complete telemetry frame.
// Caller MUST memset the TelemetryPayload to zero before filling it: the
// reserved bytes are covered by the CRC, and stack garbage there makes frames
// non-reproducible (see pico_protocol.h, "zero-initialisation contract").
void packet_build_telemetry(const TelemetryPayload *tp, uint8_t *out_buf);

// Scans the RX ring buffer for a valid CMD frame.
// Returns 1 and fills *out_cmd if one is found, 0 otherwise.
// Valid frames of other known types (TELEMETRY, PID) are consumed whole and
// skipped rather than rescanned byte by byte.
uint8_t packet_parse_cmd(CommandPayload *out_cmd);

#endif // PACKET_H
