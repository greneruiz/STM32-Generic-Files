#STM32 Generic Files

## Description
  This repository contains my commonly used files for STM32 -nostdlib baremetal projects. Currently supports STM32F103 and STM32F401.

## Resources
  ### stm32_i2c.h, stm32_i2c.c
    * I2C - Uses default pin mapping for the I2C channels.
        I2C1: SCL PB6, SDA PB7
        I2C2: SCL PB10, SDA PB11
        I2C3: SCL PA8, SDA PC9 (for STM32F4)

  ### ggg_stm32_helper.h
    * Debug LED for Blue/BlackPill boards: PC13
    * User Button for Blue/BlackPill boards: PA0
    * Programmable Wait-delay: TIM2
    * Half-duplex 115200 baud USART2: TX PA2
    * custom callbacks for printf dependencies

  ### stm32f103rb_startup.c, stm32f401xc_startup.c
    * device-specific startup files

  ### stm32f1_linker.ld, stm32f4_linker.ld
    * device-specific linker files

  ### Makefile
    * Makefile template to use the above dependencies
