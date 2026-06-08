#ifndef OLED_H
#define OLED_H

#include <stdint.h>

#define OLED_WIDTH  128
#define OLED_HEIGHT  64

void oled_init(void);
void oled_clear(void);
void oled_update(void);
void oled_draw_string(uint8_t col, uint8_t row, const char *str);

#endif
