TARGET = firmware

include config/chip.mk

CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

OPT = -Og

CFLAGS  = $(MCU) $(OPT) -Wall -fdata-sections -ffunction-sections
CFLAGS += -DUSE_HAL_DRIVER
CFLAGS += -Iapp/inc
CFLAGS += -Iconfig
CFLAGS += -Iplatform/hal/Inc
CFLAGS += -Iplatform/cmsis/CMSIS/Include
CFLAGS += -Iplatform/cmsis/CMSIS/Device/ST/STM32F4xx/Include
CFLAGS += $(DEFS)

ASFLAGS = $(MCU) $(OPT)

LDFLAGS = -T platform/linker/$(LDSCRIPT) \
          -nostartfiles \
          --specs=nosys.specs \
          --specs=nano.specs \
          -Wl,--gc-sections

SRC = \
app/src/main.c \
config/clock.c \
platform/startup/$(STARTUP) \
platform/cmsis/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c \
platform/hal/Src/stm32f4xx_hal.c \
platform/hal/Src/stm32f4xx_hal_gpio.c \
platform/hal/Src/stm32f4xx_hal_rcc.c \
platform/hal/Src/stm32f4xx_hal_rcc_ex.c \
platform/hal/Src/stm32f4xx_hal_cortex.c \
platform/hal/Src/stm32f4xx_hal_uart.c \
platform/hal/Src/stm32f4xx_hal_dma.c \
platform/hal/Src/stm32f4xx_hal_i2c.c \
app/src/stm32f4xx_hal_msp.c \
app/src/stm32f4xx_it.c \
app/src/syscalls.c \

all: $(TARGET).elf

$(TARGET).elf:
	mkdir -p build
	$(CC) $(CFLAGS) $(SRC) $(LDFLAGS) -o build/$@
	$(OBJCOPY) -O ihex build/$@ build/$(TARGET).hex
	$(OBJCOPY) -O binary build/$@ build/$(TARGET).bin

clean:
	rm -rf build
