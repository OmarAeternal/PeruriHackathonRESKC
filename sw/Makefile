# Cross-compile on a PC:   make
# Native build on the board: make CROSS_COMPILE=
CROSS_COMPILE ?= arm-linux-gnueabihf-
CC     := $(CROSS_COMPILE)gcc
CFLAGS ?= -O2 -Wall -Wextra -std=gnu99
NF_OFFSET ?= 0x00040000

hps_driver: hps_driver.c
	$(CC) $(CFLAGS) -DNF_OFFSET=$(NF_OFFSET)u -o $@ $<

clean:
	rm -f hps_driver

.PHONY: clean
