// packet.h

#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>
#include "struct.h"   // TelemetryPayload, CommandPayload



#define STX1         0xAA
#define STX2         0x55
#define PAYLOAD_LEN  56
#define PACKET_SIZE  64   // 4 header + 56 payload + 4 CRC32

#define TYPE_TELEMETRY  0x01
#define TYPE_CMD        0x02

// Fills out_buf[64] with a complete telemetry frame ready to transmit
// Returns nothing — caller passes the TelemetryPayload to serialize
void packet_build_telemetry(const TelemetryPayload *tp, uint8_t *out_buf);

// Scans ring buffer for a valid CMD packet
// Returns 1 if found and CRC passes, 0 otherwise
// On success, copies parsed payload into *out_cmd
uint8_t packet_parse_cmd(CommandPayload *out_cmd);

#endif
