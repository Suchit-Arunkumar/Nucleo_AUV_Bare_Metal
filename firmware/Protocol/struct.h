#ifndef STRUCT_H
#define STRUCT_H

#include <stdint.h>

// Payload layouts for the Pi <-> MCU link. Wire format and the offset pins
// that hold these layouts in place are in packet.h; the reference for both is
// pico-protocol (firmware/pico_protocol.h).

#define PAYLOAD_LEN 56

// Carried in every payload. It sits in a byte that earlier revisions sent as
// reserved zero, so a receiver that ignores it keeps working; a receiver that
// checks it can tell an old sender (0) from this layout (1).
#define PROTOCOL_VERSION 1

// ── TYPE 0x01 : TELEMETRY payload (MCU → Pi, 56 bytes) ──────────────────────
//
// Layout (__packed__, 56 bytes):
//   float    depth_m        —  4 : control depth (current_z from fused CMD state)
//   float    raw_depth_m    —  4 : raw pressure-sensor depth (telemetry/debug only)
//   float    pid_u[6]       — 24 : PID output per DOF (U[]) for surge…yaw
//   uint16_t esc_pwm[8]     — 16 : actual ESC PWM values T1–T8 (µs)
//   uint8_t  armed          —  1 : armed state (0/1)
//   uint8_t  sat_flags      —  1 : bit0=sat_vert, bit1=sat_horiz, bit2=sat_yaw
//   uint8_t  link_ok        —  1 : 1 = link healthy, 0 = lost/timeout
//   uint8_t  version        —  1 : PROTOCOL_VERSION
//   uint8_t  reserved[4]    —  4 : zero; covered by the CRC
//                              ──
//   Total                      56 bytes
//
// Python format string: '<2f6f8H4B4x'
typedef struct __attribute__((packed)) {
    float    depth_m;
    float    raw_depth_m;
    float    pid_u[6];
    uint16_t esc_pwm[8];
    uint8_t  armed;
    uint8_t  sat_flags;
    uint8_t  link_ok;
    uint8_t  version;
    uint8_t  reserved[4];
} TelemetryPayload;
_Static_assert(sizeof(TelemetryPayload) == PAYLOAD_LEN,"TelemetryPayload size mismatch");


// ── TYPE 0x02 : CMD payload (Pi → MCU, 56 bytes) ────────────────────────────
//
// Python format string: '<12f3B5x'  (armed, seq, version, reserved[5])
typedef struct __attribute__((packed)) {
    float    current_x;
    float    current_y;
    float    current_z;
    float    current_roll;
    float    current_pitch;
    float    current_yaw;

    float    target_x;
    float    target_y;
    float    target_z;
    float    target_roll;
    float    target_pitch;
    float    target_yaw;

    uint8_t  armed;
    uint8_t  seq;
    uint8_t  version;

    uint8_t  reserved[5];
} CommandPayload;
_Static_assert(sizeof(CommandPayload) == PAYLOAD_LEN,"CommandPayload size mismatch");

#endif
