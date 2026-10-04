#ifndef IWDG_H
#define IWDG_H

#include "stm32f446xx.h"
#include <stdint.h>

int  iwdg_init(void);   // 0 = configured, -1 = PVU/RVU timeout (see iwdg.c)

void iwdg_kick(void);


#endif
