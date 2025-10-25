# Makefile optimized for multiple STM32Fx implementations
# To do (20251010) : Add definitions for enabling FPU on F4 versions

# Project Name:
PROJ_NAME := main

# Specify STM32 version here (valid values: f1, f4)
VER := f4

# Specify build (valid values: debug, release)
BUILD := debug


SRC_DIR := src
INC_DIR := include
BIN_DIR := bin


cflags.common := -c -mthumb -std=gnu11 -I$(INC_DIR)
cflags.f4 := -mcpu=cortex-m4
cflags.f1 := -mcpu=cortex-m3
cflags.debug := -g -Wall

ldflags.common := -nostdlib -T
#ldflags.common := -specs=nano.specs -T
ldflags.f4 := "$(SRC_DIR)/stm32f4_linker.ld"
ldflags.f1 := "$(SRC_DIR)/stm32f1_linker.ld"
ldflags.debug := -Wl,-Map=$(BIN_DIR)/$(PROJ_NAME).map


COMPILER = arm-none-eabi-gcc
CFLAGS := ${cflags.${VER}} ${cflags.common} ${cflags.${BUILD}}
LDFLAGS := ${ldflags.common} ${ldflags.${VER}} ${ldflags.${BUILD}}


this: $(BIN_DIR)/$(PROJ_NAME).elf

$(BIN_DIR)/%.o : $(SRC_DIR)/%.c
	$(COMPILER) $(CFLAGS) $^ -o $@


# To do: add auto-scan of .o files; for now, just specify each individual one (eww)
OBJ_FILES := $(BIN_DIR)/$(PROJ_NAME).o $(BIN_DIR)/stm32_i2c.o $(BIN_DIR)/sbrk.o $(BIN_DIR)/at24cxx.o

ifeq ($(VER),f4)
	OBJ_FILES += $(BIN_DIR)/stm32f401xc_startup.o
else
	OBJ_FILES += $(BIN_DIR)/stm32f103rb_startup.o
endif

$(BIN_DIR)/$(PROJ_NAME).elf : $(OBJ_FILES)
	$(COMPILER) $(LDFLAGS) $(BIN_DIR)/*.o -o $@ -lgcc


OCD_CFG := openocd -f

ifeq ($(VER),f4)
	OCD_CFG += interface/stlink.cfg -f target/stm32f4x.cfg -c "transport select dapdirect_swd"
else
	OCD_CFG += interface/stlink.cfg -f target/stm32f1x.cfg -c "transport select dapdirect_swd"
endif

load:
	$(OCD_CFG)

clear:
	del $(BIN_DIR) -f *.o *.elf *.map