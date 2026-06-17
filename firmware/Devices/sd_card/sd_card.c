#include "sd_card.h"
#include "spi.h"

static void sd_delay(void)
{
    for (volatile uint32_t i = 0; i < 8000; i++);
}

static uint8_t sd_send_cmd(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    // 1. transmit command byte: 0x40 | cmd
    spi_transmit(0x40 | cmd);

    // 2. transmit arg bytes MSB first
    spi_transmit((arg >> 24) & 0xFF);
    spi_transmit((arg >> 16) & 0xFF);
    spi_transmit((arg >>  8) & 0xFF);
    spi_transmit((arg >>  0) & 0xFF);

    // 3. transmit crc with stop bit
    spi_transmit(crc | 0x01);

    // 4. poll up to 8 times for valid R1 (bit7 == 0)
    uint8_t resp = 0xFF;
    for (uint8_t i = 0; i < 8; i++) {
        resp = spi_receive();
        if (!(resp & 0x80)) break;
    }
    return resp;
}

SD_Status sd_init(void)
{
    // 1. power stabilisation delay
    sd_delay();

    // 2. 80 dummy clocks with CS high to enter SPI mode
    spi_deselect_sd();
    for (uint8_t i = 0; i < 10; i++) spi_transmit(0xFF);

    // 3. CMD0 — software reset, enter SPI mode
    spi_select_sd();
    uint8_t r1 = sd_send_cmd(0, 0x00000000, 0x95);
    spi_deselect_sd();
    spi_transmit(0xFF);

    if (r1 != 0x01) return SD_FAIL;

    // 4. CMD8 — interface condition, check for v2 card
    spi_select_sd();
    r1 = sd_send_cmd(8, 0x000001AA, 0x87);
    uint8_t r7[4];
    for (uint8_t i = 0; i < 4; i++) r7[i] = spi_receive();
    spi_deselect_sd();
    spi_transmit(0xFF);

    // 5. ACMD41 loop — wait for card to finish init
    uint32_t timeout = 1000;
    do {
        // CMD55 — APP_CMD prefix
        spi_select_sd();
        sd_send_cmd(55, 0x00000000, 0x65);
        spi_deselect_sd();
        spi_transmit(0xFF);

        // CMD41 — send operating condition, HCS bit set for SDHC
        spi_select_sd();
        r1 = sd_send_cmd(41, 0x40000000, 0x77);
        spi_deselect_sd();
        spi_transmit(0xFF);

        sd_delay();
    } while ((r1 & 0x01) && --timeout);

    if (timeout == 0) return SD_FAIL;

    // 6. CMD58 — read OCR, verify 3.3V support
    spi_select_sd();
    r1 = sd_send_cmd(58, 0x00000000, 0xFD);
    uint8_t ocr[4];
    for (uint8_t i = 0; i < 4; i++) ocr[i] = spi_receive();
    spi_deselect_sd();
    spi_transmit(0xFF);

    if (r1 != 0x00) return SD_FAIL;
    if (!(ocr[1] & 0x30)) return SD_FAIL;

    return SD_OK;
}

SD_Status sd_write_block(uint32_t block_addr, const uint8_t *data)
{
    uint8_t r1;
    uint32_t timeout;

    // SDSC cards need byte addressing
    // block_addr *= 512;

    spi_select_sd();

    // CMD24 = WRITE_BLOCK
    r1 = sd_send_cmd(24, block_addr, 0x01);

    if (r1 != 0x00)
    {
        spi_deselect_sd();
        spi_transmit(0xFF);
        return SD_FAIL;
    }

    // One byte gap before data token
    spi_transmit(0xFF);

    // Start block token
    spi_transmit(0xFE);

    // Send 512-byte sector
    for (uint16_t i = 0; i < 512; i++)
    {
        spi_transmit(data[i]);
    }

    // Dummy CRC (ignored unless CRC enabled)
    spi_transmit(0xFF);
    spi_transmit(0xFF);

    // Data response token
    uint8_t response = spi_receive();

    if ((response & 0x1F) != 0x05)
    {
        spi_deselect_sd();
        spi_transmit(0xFF);
        return SD_FAIL;
    }

    // Wait while card is busy
    timeout = 100000;

    while (--timeout)
    {
        if (spi_receive() == 0xFF)
            break;
    }

    spi_deselect_sd();
    spi_transmit(0xFF);

    if (timeout == 0)
        return SD_FAIL;

    return SD_OK;
}

SD_Status sd_read_block(uint32_t block_addr, uint8_t *buf)
{
    uint8_t r1;
    uint32_t timeout;

    // SDSC cards require byte addressing
    // block_addr *= 512;

    // 1. CS low, send CMD17
    spi_select_sd();

    r1 = sd_send_cmd(17, block_addr, 0x01);

    if (r1 != 0x00)
    {
        spi_deselect_sd();
        spi_transmit(0xFF);
        return SD_FAIL;
    }

    // 2. Wait for start token 0xFE
    timeout = 100000;

    while (--timeout)
    {
        if (spi_receive() == 0xFE)
            break;
    }

    if (timeout == 0)
    {
        spi_deselect_sd();
        spi_transmit(0xFF);
        return SD_FAIL;
    }

    // 3. Read 512-byte sector
    for (uint16_t i = 0; i < 512; i++)
    {
        buf[i] = spi_receive();
    }

    // 4. Discard CRC16
    spi_receive();
    spi_receive();

    // 5. CS high + extra clocks
    spi_deselect_sd();
    spi_transmit(0xFF);

    // 6. Success
    return SD_OK;
}
