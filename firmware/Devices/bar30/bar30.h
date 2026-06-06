#ifndef BAR30_H
#define BAR30_H

#include <stm32f446xx.h>
#include <stdint.h>


// declare bar30_init()
void bar30_init(void);

// declare bar30_read()  -- returns float (depth in meters)
float bar30_read(void);

#endif
