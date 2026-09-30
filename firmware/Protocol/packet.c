// =============================================================================
// packet.c
// Framing for the Pi <-> MCU link. Byte-identical to pico-protocol: see packet.h
// for the wire format and why the hardware CRC unit is not used here.
// =============================================================================
#include "packet.h"
#include "ring_buffer.h"
#include <string.h>

// -----------------------------------------------------------------------------
// CRC-16/IBM-3740: poly 0x1021, init 0xFFFF, no reflection, no final XOR.
// Bitwise on purpose: 58 bytes x 8 iterations is a few microseconds at
// 180 MHz, twice per 20 ms cycle. A 512-byte table buys nothing measurable.
// -----------------------------------------------------------------------------
uint16_t packet_crc16(const uint8_t *data, size_t n)
{
    uint16_t c = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        c ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++)
            c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
    }
    return c;
}

void packet_build_telemetry(const TelemetryPayload *tp, uint8_t *out_buf)
{
    out_buf[0] = STX1;
    out_buf[1] = STX2;
    out_buf[2] = PAYLOAD_LEN;
    out_buf[3] = TYPE_TELEMETRY;

    memcpy(&out_buf[HEADER_SIZE], tp, PAYLOAD_LEN);

    // [LEN, TYPE, PAYLOAD] is contiguous in out_buf starting at byte 2.
    uint16_t crc = packet_crc16(&out_buf[2], 2 + PAYLOAD_LEN);

    out_buf[CRC_OFFSET]     = (uint8_t)(crc >> 8);   // CRC_HI
    out_buf[CRC_OFFSET + 1] = (uint8_t)(crc);        // CRC_LO
}

uint8_t packet_parse_cmd(CommandPayload *out_cmd)
{
    while (rx_avail() >= PACKET_SIZE)
    {
        // --- Check 1: sync bytes --------------------------------------------
        if (rx_peek(0) != STX1 || rx_peek(1) != STX2) {
            rx_eat(1);
            continue;
        }

        // --- Check 2: length + known type ------------------------------------
        uint8_t len  = rx_peek(2);
        uint8_t type = rx_peek(3);
        uint8_t known = (type == TYPE_TELEMETRY) ||
                        (type == TYPE_CMD)       ||
                        (type == TYPE_PID);
        if (len != PAYLOAD_LEN || !known) {
            rx_eat(1);
            continue;
        }

        // --- Check 3: CRC over [LEN, TYPE, PAYLOAD] ---------------------------
        uint8_t pkt[PACKET_SIZE];
        for (uint16_t i = 0; i < PACKET_SIZE; i++)
            pkt[i] = rx_peek(i);

        uint16_t calc = packet_crc16(&pkt[2], 2 + PAYLOAD_LEN);
        uint16_t rcvd = (uint16_t)(((uint16_t)pkt[CRC_OFFSET] << 8) | pkt[CRC_OFFSET + 1]);
        if (calc != rcvd) {
            rx_eat(1);
            continue;
        }

        // Valid frame. Consume it whole either way.
        rx_eat(PACKET_SIZE);

        if (type != TYPE_CMD)
            continue;   // valid but not for us (e.g. PID) -- skip, keep scanning

        memcpy(out_cmd, &pkt[HEADER_SIZE], sizeof(CommandPayload));
        return 1;
    }
    return 0;
}
