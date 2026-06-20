#include "packet.h"
#include "ring_buffer.h"
#include "crc_hw.h"
#include <string.h>

void packet_build_telemetry(const TelemetryPayload *tp, uint8_t *out_buf){

	// include headers
	out_buf[0] = STX1;
	out_buf[1] = STX2;

	//payload len
	out_buf[2] = PAYLOAD_LEN;

	//MSG ID
	out_buf[3] = TYPE_TELEMETRY;

	memcpy(&out_buf[4], tp, sizeof(TelemetryPayload));

	uint32_t crc = crc_compute(&out_buf[2], 2 + PAYLOAD_LEN);

	out_buf[PACKET_SIZE - 4]= (crc >> 24);
	out_buf[PACKET_SIZE - 3]= (crc >> 16);
	out_buf[PACKET_SIZE - 2]= (crc >> 8);
	out_buf[PACKET_SIZE - 1]= (crc);

}

uint8_t packet_parse_cmd(CommandPayload *out_cmd)
{
    while (rx_avail() >= PACKET_SIZE)
    {
        // Check STX
        if (rx_peek(0) != STX1 ||
            rx_peek(1) != STX2)
        {
            rx_eat(1);
            continue;
        }
        // Check length
        if (rx_peek(2) != PAYLOAD_LEN)
        {
            rx_eat(1);
            continue;
        }
        // Check type
        if (rx_peek(3) != TYPE_CMD)
        {
            rx_eat(1);
            continue;
        }
        // Copy packet into temp buffer
        uint8_t pkt[PACKET_SIZE];
        for (int i = 0; i < PACKET_SIZE; i++)
        {
            pkt[i] = rx_peek(i);
        }
        // Compute CRC over LEN + TYPE + PAYLOAD
        uint32_t calc_crc =
            crc_compute(&pkt[2], 2 + PAYLOAD_LEN);
        // Reconstruct received CRC
        uint32_t recv_crc =
            ((uint32_t)pkt[60] << 24) |
            ((uint32_t)pkt[61] << 16) |
            ((uint32_t)pkt[62] << 8)  |
            ((uint32_t)pkt[63]);
        if (calc_crc != recv_crc)
        {
            rx_eat(1);
            continue;
        }
        // Copy payload into struct
        memcpy(out_cmd,
               &pkt[4],
               sizeof(CommandPayload));
        // Remove packet from buffer
        rx_eat(PACKET_SIZE);
        return 1;
    }
    return 0;
}
