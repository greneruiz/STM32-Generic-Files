///===================================================================
/// File Name: at24cxx.h
/// Type     : Device Controller C-header
/// Purpose  : AT24CXX - Serial EEPROM
/// Version  : 3.0
///===================================================================
/// Description
///		* AT24C32 32Kb EEPROM controller for STM32F1xx.
///		* 32 bytes per page for 128 pages.
///		* Communication is thru I2C.
///		* DeviceID LSNibble is based on A2..A0 straps (for AT24C32/64)
///===================================================================
/// Revision History
/// Version/Date : V1.0 / 2025-Sep-01 / G.RUIZ
///		* Initial release
/// Version/Date : V1.1 / 2025-Sep-06 / G.RUIZ
///		* Modified for new I2C controller
/// Version/Date : V2.0 / 2025-Oct-17 / G.RUIZ
///		* Renamed to at24cxx.h
///		* Added capability for 16K & 64K devices
/// Version/Date : V3.0 / 2025-Oct-23 / G.RUIZ
///		* Implemented at24cxx.c for opacity
///		* Added typedef struct AT24CXX
///===================================================================



#ifndef __AT24CXX__
#define __AT24CXX__


#include <stdint.h>

//#define USE_DYNAMIC_ALLOCATION


#ifndef NULL
#ifdef __cplusplus
#define NULL	( 0	)
#else
#define NULL	( (void*) 0	)
#endif
#endif


/// @brief I2C ID for AT24Cxx devices; LSB depends on straps, or if used by the device as bank address
#define DEVICE_ID	( 0xA0		)

/// @brief Enumeration of supported devices
typedef enum
{
	AT24C16,
	AT24C32,
	AT24C64
} AT24C_Type;

///// @brief Enumeration of supported I2C channels (some items may be invalid, depending on MCU used)
//typedef enum
//{
//	I2C_INVALID,
//	I2C_CH1,
//	I2C_CH2,
//	I2C_CH3
//} I2C_CH;

/// @brief Read access type
typedef enum
{
	READ_CURRENT,
	READ_ADDRESSED
} ReadAccessType;

/// @brief Return types
typedef enum
{
	AT24C_OK,
	AT24C_FAIL
} AT24C_ReturnType;

/// @brief I2C Modes
typedef enum
{
	I2C_SM,
	I2C_FM
} I2CMode;


#define AT24C16_ADDR_BYTE_SIZE		( 1U		)
#define AT24C16_WRITE_CYCLE_TIME_MS	( 10U		)
#define AT24C16_BYTE_PER_PAGE		( 16U		)
#define AT24C16_PAGE_SIZE			( 128U		)

#define AT24C32_ADDR_BYTE_SIZE		( 2U		)
#define AT24C32_WRITE_CYCLE_TIME_MS	( 20U		)
#define AT24C32_BYTE_PER_PAGE		( 32U		)
#define AT24C32_PAGE_SIZE			( 128U		)

#define AT24C64_ADDR_BYTE_SIZE		( 2U		)
#define AT24C64_WRITE_CYCLE_TIME_MS	( 20U		)
#define AT24C64_BYTE_PER_PAGE		( 32U		)
#define AT24C64_PAGE_SIZE			( 256U		)


///==================== Factory Declarations =~========================

///===================================================================
/// SECTION_1.0 I2C Communication Virtual Table

/// @brief vtI2C - virtual table for I2C hardware abstraction
struct vtI2C
{
	uint8_t channel;
	uint8_t mcu_clock_mhz;
	uint8_t mode;
	/// @brief Initializes selected the I2C channel
	/// @param i2c_sel : Valid values: 0x01-0x02
	void ( * init )( uint8_t, uint8_t, uint8_t );

	/// @brief Waits for Idle, performs I2C start sequence, then waits for SB status bit
	/// @param i2c_sel : Valid values: 0x01-0x02
	void ( * start )( uint8_t );

	/// @brief Sends another start sequence
	/// @brief Dummy-read RD register then resend start sequence
	/// @param i2c_sel : Valid values: 0x01-0x02
	void ( * restart )( uint8_t );

	/// @brief Performs I2C stop sequence
	/// @param i2c_sel : Valid values: 0x01-0x02
	void ( * stop )( uint8_t );

	/// @brief Used after start(), sends device slave ID.
	/// @brief Writes slave ID, waits for ADDR status bit, then clears it by reading SR2 register
	/// @param i2c_sel : Valid values: 0x01-0x02
	/// @param devid : 1-byte device ID with LSB as Rd/Wr bit
	void ( * send_slave_id )( uint8_t, uint8_t );

	/// @brief Used after send_slave_id(), sends byte/s from a byte array.
	/// @brief Waits for ADDR transmit; write byte/s, then waits for completion; after last byte, waits for BTF status bit
	/// @param i2c_sel : Valid values: 0x01-0x02
	/// @param bytecount : Number of bytes to write (min: 1); must match wrbyte[] size
	/// @param wrbyte : Pass-by-reference pointer to byte array
	void ( * write )( uint8_t, uint32_t, uint8_t * );

	/// @brief Used after start(), writes device ID and reads incoming bytes. stop() is NOT required.
	/// @param i2c_sel : Valid values: 0x01-0x02
	/// @param devid : Target slave ID with LSB as read mode
	/// @param bytecount : Number of bytes to read (min: 1); must match rdbyte[] size
	/// @param rdbyte : Pass-by-reference pointer to byte-array, for storing read bytes
	void ( * read )( uint8_t, uint8_t, uint32_t, uint8_t * );
};


struct AT24CXX
{
	AT24C_Type type;
	AT24C_ReturnType ( * byte_write )( AT24C_Type, struct vtI2C *, uint8_t, uint16_t, uint8_t * );
	AT24C_ReturnType ( * burst_write )( AT24C_Type, struct vtI2C *, uint8_t, uint16_t, uint8_t, uint8_t * );
	AT24C_ReturnType ( * byte_read )( AT24C_Type, struct vtI2C *, ReadAccessType, uint8_t, uint16_t, uint8_t * );
	AT24C_ReturnType ( * burst_read )( AT24C_Type, struct vtI2C *, ReadAccessType, uint8_t, uint16_t, uint8_t, uint8_t * );
};



#ifdef USE_DYNAMIC_ALLOCATION
#include <stdlib.h>
	/// @brief  Factory to dynamically allocate an I2C handler object.
	/// @param i2c_channel valid values > 0x00
	/// @param mcu_clock_mhz MCU clock frequency, in Hz
	/// @param mode I2C mode (standard, fast)
	/// @return vtI2C * pointer to dynamically created I2C handler object.
	extern vtI2C * InitializeI2CHandler( uint8_t i2c_channel, uint8_t mcu_clock_mhz, uint8_t mode );

	/// @brief	Factory to dynamically allocate an AT24CXX handler object.
	/// @param type AT24C_Type device
	/// @param i2c vtI2C I2C handler pointer
	extern AT24CXX * InitializeEEPROM( AT24C_Type type, struct vtI2C * i2c );
#else
	/// @brief  Factory for vtI2C to initialize the I2C handler object.
	///	@param i2c * pass-by-reference pointer to the I2C handler object
	/// @param i2c_channel valid values > 0x00
	/// @param mcu_clock_mhz MCU clock frequency, in Hz
	/// @param mode I2C mode (standard, fast)
	/// @return vtI2C * pointer to dynamically created I2C handler object.
	extern void InitializeI2CHandler( struct vtI2C * i2c, uint8_t i2c_channel, uint8_t mcu_clock_mhz, uint8_t mode );

	/// @brief	Factory to initialize an AT24CXX handler object.
	/// @brief	This also initializes the I2C interface passed thru the vtI2C object.
	/// @param eeprom Pass-by-reference pointer to the AT24CXX handler object
	/// @param type AT24C_Type device
	/// @param i2c Pass by reference pointer to the vtI2C I2C handler pointer
	extern void InitializeEEPROM( struct AT24CXX * eeprom, AT24C_Type type, struct vtI2C * i2c );

#endif





#endif	///__AT24CXX__