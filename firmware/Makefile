TOOLCHAIN_PREFIX := arm-none-eabi-

CC      := $(TOOLCHAIN_PREFIX)gcc
GDB     := $(TOOLCHAIN_PREFIX)gdb
OBJCOPY := $(TOOLCHAIN_PREFIX)objcopy

CFLAGS  := -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -ffunction-sections -fdata-sections
LDFLAGS := -Wl,--gc-sections -static

#CFLAGS  += -W -Wall -Wextra -g3 -ggdb #-gsplit-dwarf
CFLAGS  += -Os
LDFLAGS += -Tgcc_arm.ld

CMSIS_HDRS := cmsis_gcc.h
CMSIS_SRCS := startup_ARMCM0plus.S

I2C_HDRS := i2c.h
I2C_SRCS := i2c.c

KEYPAD_HDRS := udrivers/keypad/generic/src/keypad.h
KEYPAD_SRCS := udrivers/keypad/generic/src/keypad.c keypad-glue.c

all: main.elf

main.elf: main.c lpc81x.h $(I2C_SRCS) $(I2C_HDRS) $(KEYPAD_SRCS) $(KEYPAD_HDRS) $(CMSIS_SRCS) $(CMSIS_HDRS)
	$(CC) $(CFLAGS) $(LDFLAGS) -Wl,-Map=main.map -o $@ $< $(I2C_SRCS) $(KEYPAD_SRCS) $(CMSIS_SRCS)

main.raw: main.elf
	$(OBJCOPY) -S -j .text -j .ARM.exidx -j .data -O binary $< $@

clean:
	$(RM) *.o *.dwo main.elf main.raw main.map

debug:
	$(GDB) -ix debug/openocd.gdb
