#ifndef STRUCT_H
#define STRUCT_H

#include <stdint.h>
#include "stm32f446xx.h"

#define PAYLOAD_LEN 56

// ── TYPE 0x01 : TELEMETRY payload (Pico → Pi, 56 bytes) ─────────────────────
//
// Centralised debug/monitoring snapshot — everything the Pi needs in one place.
//
// Layout (__packed__, 56 bytes):
//   float    depth_m        —  4 : control depth (current_z from fused CMD state)
//   float    raw_depth_m    —  4 : raw pressure-sensor depth (telemetry/debug only)
//   float    pid_u[6]       — 24 : PID output per DOF (U[]) for surge…yaw
//   uint16_t esc_pwm[8]     — 16 : actual ESC PWM values T1–T8 (µs)
//   uint8_t  armed          —  1 : armed state (0/1)
//   uint8_t  sat_flags      —  1 : bit0=sat_vert, bit1=sat_horiz, bit2=sat_yaw
//   uint8_t  link_ok        —  1 : 1 = link healthy, 0 = lost/timeout
//   uint8_t  reserved       —  1 : zero-padding
//                              ──
//   Total                      56 bytes ✓
typedef struct __attribute__((packed)) {
    float    depth_m;
    float    raw_depth_m;
    float    pid_u[6];
    uint16_t esc_pwm[8];
    uint8_t  armed;
    uint8_t  sat_flags;
    uint8_t  link_ok;
    uint8_t  reserved[5];
} TelemetryPayload;
_Static_assert(sizeof(TelemetryPayload) == PAYLOAD_LEN,"TelemetryPayload size mismatch");




// ── TYPE 0x02 : CMD payload (Pi → Pico, 56 bytes) ───────────────────────────
// Python format string: '<12f3B5s'
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

    uint8_t  reserved[6];
} CommandPayload;
_Static_assert(sizeof(CommandPayload) == PAYLOAD_LEN,"CommandPayload size mismatch");



#endif
