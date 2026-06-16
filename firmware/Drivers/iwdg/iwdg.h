#ifndef IWDG_H
#define IWDG_H

#include "stm32f446xx.h"
#include <stdint.h>

void iwdg_init(void);

void iwdg_kick(void);


#endif
