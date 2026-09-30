# Host build of the Nucleo simulator (Linux / macOS / WSL).
# Compiles the firmware's own protocol, ring buffer and control-loop sources.
FW      := ../../../firmware
CC      ?= cc
CFLAGS  ?= -std=gnu11 -O2 -Wall -Wextra
CPPFLAGS = -Ishim -I$(FW)/Protocol -I$(FW)/Core/Inc

SRCS = nucleo_sim.c \
       $(FW)/Protocol/packet.c \
       $(FW)/Protocol/ring_buffer.c \
       $(FW)/Core/Src/control_loop.c

nucleo_sim: $(SRCS) shim/stm32f446xx.h shim/timer_pwm.h
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SRCS) -lm -o $@

clean:
	rm -f nucleo_sim

.PHONY: clean
