#include "bar30.h"
#include "i2c.h"
#include "system_init.h"   // for delay_ms

#define BAR30_ADDR     0x76
#define BAR30_RESET    0x1E
#define BAR30_PROM     0xA0
#define BAR30_CONV_D1  0x48
#define BAR30_CONV_D2  0x58
#define BAR30_ADC_READ 0x00

static uint16_t prom[8];

void bar30_init(void)
{
    uint8_t cmd;
    uint8_t buf[2];

    // 1. send reset: cmd = BAR30_RESET, i2c_write(BAR30_ADDR, &cmd, 1)
    cmd = BAR30_RESET;
    i2c_write(BAR30_ADDR , &cmd, 1);

    // 2. delay_ms(10)
    delay_ms(10);

    // 3. loop i from 0 to 7:
    //    a. cmd = BAR30_PROM + (i * 2)
    //    b. i2c_write(BAR30_ADDR, &cmd, 1)
    //    c. i2c_read(BAR30_ADDR, buf, 2)
    //    d. prom[i] = (buf[0] << 8) | buf[1]
    for(int i = 0; i < 8; i++){

    	cmd = BAR30_PROM + (i*2);
    	i2c_write(BAR30_ADDR , &cmd, 1);
    	i2c_read(BAR30_ADDR , buf, 2);
    	prom[i] = (buf[0] << 8) | buf[1];

    }

}

float bar30_read(void)
{
    uint8_t cmd;
    uint8_t buf[3];
    uint32_t D1, D2;
    int32_t dT, TEMP;
    int64_t OFF, SENS, P;

    // 1. cmd = BAR30_CONV_D1, i2c_write(BAR30_ADDR, &cmd, 1)
    cmd = BAR30_CONV_D1;
    i2c_write(BAR30_ADDR, &cmd, 1);

    // 2. delay_ms(10)
    delay_ms(10);

    // 3. cmd = BAR30_ADC_READ, i2c_write(BAR30_ADDR, &cmd, 1)
    cmd = BAR30_ADC_READ;
    i2c_write(BAR30_ADDR, &cmd, 1);

    // 4. i2c_read(BAR30_ADDR, buf, 3)
    i2c_read(BAR30_ADDR, buf, 3);

    // 5. D1 = (buf[0] << 16) | (buf[1] << 8) | buf[2]
    D1 = (buf[0] << 16) | (buf[1] << 8) | buf[2];

    // 6. cmd = BAR30_CONV_D2, i2c_write(BAR30_ADDR, &cmd, 1)
    cmd = BAR30_CONV_D2;
    i2c_write(BAR30_ADDR, &cmd, 1);

    // 7. delay_ms(10)
    delay_ms(10);

    // 8. cmd = BAR30_ADC_READ, i2c_write(BAR30_ADDR, &cmd, 1)
    cmd = BAR30_ADC_READ;
    i2c_write(BAR30_ADDR, &cmd, 1);

    // 9. i2c_read(BAR30_ADDR, buf, 3)
    i2c_read(BAR30_ADDR, buf, 3);

    // 10. D2 = (buf[0] << 16) | (buf[1] << 8) | buf[2]
    D2 = (buf[0] << 16) | (buf[1] << 8) | buf[2];

    // 11. dT   = (int32_t)D2 - ((int32_t)prom[5] << 8)
    dT   = (int32_t)D2 - ((int32_t)prom[5] << 8);

    // 12. TEMP = 2000 + ((int64_t)dT * prom[6]) / (1 << 23)
    TEMP = 2000 + ((int64_t)dT * prom[6]) / (1 << 23);

    // 13. OFF  = ((int64_t)prom[2] << 16) + ((int64_t)prom[4] * dT) / (1 << 7)
	OFF  = ((int64_t)prom[2] << 16) + ((int64_t)prom[4] * dT) / (1 << 7);

    // 14. SENS = ((int64_t)prom[1] << 15) + ((int64_t)prom[3] * dT) / (1 << 8)
    SENS = ((int64_t)prom[1] << 15) + ((int64_t)prom[3] * dT) / (1 << 8);

    // 15. P    = ((int64_t)D1 * SENS / (1 << 21) - OFF) / (1 << 13)
    P = ((int64_t)D1 * SENS / (1 << 21) - OFF) / (1 << 13);

    // 16. return (P - 101300.0f) / (1025.0f * 9.80665f)
    return (P - 101300.0f) / (1025.0f * 9.80665f);
}
