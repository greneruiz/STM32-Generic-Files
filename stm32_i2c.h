///===================================================================
/// File Name: stm32_i2c.h
/// Type     : STM32 C-header
/// Purpose  : I2C - STM32F103RB Nucleo
/// Version  : 2.2
///===================================================================
/// Description
///		* I2C control for STM32
///===================================================================
/// Revision History
/// Version/Date : V1.0 / 2025-Aug-29 / G.RUIZ
///		* Initial release
/// Version/Date : v1.1 / 2025-Sep-03 / G.RUIZ
///		* Added STM32F1 prefix
///		* Replaced argument datatypes with stdint.h datatypes
///		* Added option to change MCU frequency
///		* Prepped functions to return error codes
///		* Renamed to stm32f1_i2c
/// Version/Date : v2.0 / 2025-Oct-12 / G.RUIZ
///		* Renamed to stm32_i2c
///		* Modified for STM32F4xx
/// Version/Date : v2.2 / 2025-Oct-24 / G.RUIZ
///		* mcu_period_ns now integer-calculated
///		* changed uint32_t mcu_period_hz to uint8_t mcu_period_mhz; 
///===================================================================



#ifndef STM32F1_I2C_H_
#define STM32F1_I2C_H_

#include <stdint.h>


///======================= Device Settings ===========================

/// @attention Device selection. Define only one:
//#define USE_STM32F1
#define USE_STM32F4


typedef enum
{
	I2C_SM_100KHZ,
	I2C_FM_400KHZ
} I2C_Mode;


typedef enum
{
	I2C_INVALID,
	I2C_CH1,
	I2C_CH2,
	I2C_CH3
} I2C_CH;



///=================== Function Declarations =========================

/// @brief Initialize selected I2C (For F1: I2C1-I2C2; For F4: I2C1-I2C3) using I2C_TypeDef* datatype
/// @param i2c_sel : STM32_I2C_CH (For F1: I2C1-I2C2; For F4: I2C1-I2C3)
extern void stm32_i2c_init(uint8_t mcu_freq_mhz, uint8_t i2csel, uint8_t i2cspeed);


/// @brief Wait for Idle, perform I2C start sequence, then wait for SB status bit
/// @param i2csel : STM32_I2C_CH (For F1: I2C1-I2C2; For F4: I2C1-I2C3)
extern void stm32_i2c_start(uint8_t i2csel);


/// @brief Send another start sequence
/// @brief Dummy-read RD register then resend start sequence
/// @param i2csel : STM32_I2C_CH (For F1: I2C1-I2C2; For F4: I2C1-I2C3)
extern void stm32_i2c_restart(uint8_t i2csel);


/// @brief Perform I2C stop sequence
/// @param i2csel : STM32_I2C_CH (For F1: I2C1-I2C2; For F4: I2C1-I2C3)
extern void stm32_i2c_stop(uint8_t i2csel);

/// @brief Used after i2c_start(), send device slave ID.
/// @brief Write slave ID, wait for ADDR status bit, then clear it by reading SR2 register
/// @param i2csel : STM32_I2C_CH (For F1: I2C1-I2C2; For F4: I2C1-I2C3)
/// @param devid : 1-byte device ID with LSB as Rd/Wr bit
extern void stm32_i2c_send_slave_id(uint8_t i2csel, uint8_t devid);


/// @brief Used after i2c_send_slave_id(), send byte/s from a byte array.
/// @brief Wait for ADDR transmit; write byte/s, then wait for completion; after last byte, wait for BTF status bit
/// @param i2csel : STM32_I2C_CH (For F1: I2C1-I2C2; For F4: I2C1-I2C3)
/// @param bytecount : Number of bytes to write (min: 1); must match wrbyte[] size
/// @param wrbyte : Pass-by-reference pointer to byte array
extern void stm32_i2c_write(uint8_t i2csel, uint32_t bytecount, uint8_t * wrbyte);


/// @brief Used after i2c_start(), write device ID and read incoming bytes. i2c_stop() is NOT required.
/// @brief Performs "Method2" I2C Read (refer to RM0008 document)
/// @param i2csel : STM32_I2C_CH (For F1: I2C1-I2C2; For F4: I2C1-I2C3)
/// @param devid : Target slave ID with LSB as read mode
/// @param bytecount : Number of bytes to read (min: 1); must match rdbyte[] size
/// @param rdbyte : Pass-by-reference pointer to byte-array, for storing read bytes
extern void stm32_i2c_read(uint8_t i2csel, uint8_t devid, uint32_t bytecount, uint8_t * rdbyte);



#endif ///STM32F1_I2C_H_