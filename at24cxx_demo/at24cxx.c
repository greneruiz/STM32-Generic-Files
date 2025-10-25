///===================================================================
/// File Name: at24cxx.c
/// Type     : Device Controller C-source
/// Purpose  : AT24CXX - Serial EEPROM
/// Version  : 1.0
///===================================================================
/// Description
///		* AT24C32 32Kb EEPROM controller for STM32F1xx.
///		* 32 bytes per page for 128 pages.
///		* Communication is thru I2C.
///		* DeviceID LSNibble is based on A2..A0 straps (for AT24C32/64)
///===================================================================
/// Revision History
/// Version/Date : V1.0 / 2025-Oct-24 / G.RUIZ
///		* Initial release
///		* Implemented at24cxx.c for opacity
///		* Implemented error catches
///===================================================================




#include "at24cxx.h"
#include "stm32_i2c.h"



#if defined( USE_DYNAMIC_ALLOCATION )
	/// @brief  Factory for vtI2C to dynamically allocate an I2C handler object.
	/// @param i2c_channel valid values > 0x00
	/// @param mcu_clock_mhz MCU clock frequency, in Hz
	/// @param mode I2C mode (standard, fast)
	/// @return vtI2C * pointer to dynamically created I2C handler object.
	vtI2C * InitializeI2CHandler( uint8_t i2c_channel, uint8_t mcu_clock_mhz, uint8_t mode )
	{
		vtI2C * obj = (vtI2C *)malloc( sizeof( vtI2C ));
		if( obj == NULL )
			return NULL;

		obj->channel = i2c_channel;
		obj->mcu_clock_mhz = mcu_clock_mhz;
		obj->mode = mode;

		obj->init = &stm32_i2c_init;
		obj->start = &stm32_i2c_start;
		obj->restart = &stm32_i2c_restart;
		obj->stop = &stm32_i2c_stop;
		obj->send_slave_id = &stm32_i2c_send_slave_id;
		obj->write = &stm32_i2c_write;
		obj->read = &stm32_i2c_read;
		
		return obj;
	}
#else
	/// @brief  Factory for vtI2C to initialize the I2C handler object.
	///	@param i2c * pass-by-reference pointer to the I2C handler object
	/// @param i2c_channel valid values > 0x00
	/// @param mcu_clock_mhz MCU clock frequency, in Hz
	/// @param mode I2C mode (standard, fast)
	/// @return vtI2C * pointer to dynamically created I2C handler object.
	void InitializeI2CHandler( struct vtI2C * i2c, uint8_t i2c_channel, uint8_t mcu_clock_mhz, uint8_t mode )
	{
		i2c->channel = i2c_channel;
		i2c->mcu_clock_mhz = mcu_clock_mhz;
		i2c->mode = mode;

		i2c->init = &stm32_i2c_init;
		i2c->start = &stm32_i2c_start;
		i2c->restart = &stm32_i2c_restart;
		i2c->stop = &stm32_i2c_stop;
		i2c->send_slave_id = &stm32_i2c_send_slave_id;
		i2c->write = &stm32_i2c_write;
		i2c->read = &stm32_i2c_read;
	}
#endif


///===================================================================
/// SECTION_3.0 Function declarations

/// @brief Initialize selected I2C
/// @param i2c_sel : 1 = I2C1; 2 = I2C2
/// @param mode : 0 = SM 100kHz; 1 = FM 400kHz
AT24C_ReturnType eeprom_initialize( struct vtI2C * i2c );

/// @brief Single byte write to address
/// @param i2c_sel : 1 = I2C1; 2 = I2C2
/// @param devid_lsn : Device ID[3..1] (strap resistor values, or block select, depending on device); LSB is masked
/// @param addr : Memory address
/// @param wrbyte : Pass-by-reference pointer to address of byte to write
AT24C_ReturnType eeprom_byte_write( AT24C_Type type, struct vtI2C * i2c, uint8_t devid_lsn, uint16_t addr, uint8_t * wrbyte );

/// @brief Consecutively write an array of bytes, starting with the given address
/// @param devid_lsn : Device ID[3..1] (strap resistor values, or block select, depending on device); LSB is masked
/// @param addr : Memory address
/// @param count : Number of bytes to write
/// @param wrbyte : Pass-by-reference pointer to address of byte array to write
AT24C_ReturnType eeprom_burst_write( AT24C_Type type, struct vtI2C * i2c, uint8_t devid_lsn, uint16_t addr, uint8_t count, uint8_t * wrbyte );

/// @brief Read 1 byte from the EEPROM.
/// @param type : AT24CXX device type
/// @param devid_lsn : Device ID[3..1] (strap resistor values, or block select, depending on device); LSB is masked
/// @param addr : Memory address
/// @param rdbyte : Pass-by-reference pointer to address where the read byte will be stored
AT24C_ReturnType eeprom_byte_read( AT24C_Type type, struct vtI2C * i2c, ReadAccessType rdAccess, uint8_t devid_lsn, uint16_t addr, uint8_t * rdbyte );

/// @brief Consecutively read bytes from the EEPROM
/// @param devid_lsn : Device ID[3..1] (strap resistor values, or block select, depending on device); LSB is masked
/// @param count : Number of bytes to read
/// @param rdbyte : Pass-by-reference pointer to address of array where the read bytes will be stored
AT24C_ReturnType eeprom_burst_read( AT24C_Type type, struct vtI2C * i2c, ReadAccessType rdAccess, uint8_t devid_lsn, uint16_t addr, uint8_t count, uint8_t * rdbyte );


static uint8_t concat_devid_lsn( uint8_t devid_lsn );
static void prep_mem_addr( uint16_t addr, uint8_t * memaddr );




#if defined( USE_DYNAMIC_ALLOCATION )
	/// @brief	Factory to dynamically allocate an AT24CXX handler object.
	/// @brief	This also initializes the I2C interface passed thru the vtI2C object.
	/// @param type AT24C_Type device
	/// @param i2c vtI2C I2C handler pointer
	AT24CXX * InitializeEEPROM( AT24C_Type type, vtI2C * i2c )
	{
		AT24CXX * obj = (AT24CXX *)malloc( sizeof( AT24CXX ));
		if( obj == NULL || i2c == NULL )
			return NULL;

		obj->type = type;
		obj->byte_write = &eeprom_byte_write;
		obj->burst_write = &eeprom_burst_write;
		obj->byte_read = &eeprom_byte_read;
		obj->burst_read = &eeprom_burst_read;

		return ( eeprom_initialize( i2c ) == AT24C_OK ? obj : NULL );
	}
#else
	/// @brief	Factory to initialize an AT24CXX handler object.
	/// @brief	This also initializes the I2C interface passed thru the vtI2C object.
	/// @param eeprom Pass-by-reference pointer to the AT24CXX handler object
	/// @param type AT24C_Type device
	/// @param i2c Pass by reference pointer to the vtI2C I2C handler pointer
	void InitializeEEPROM( struct AT24CXX * eeprom, AT24C_Type type, struct vtI2C * i2c )
	{
		eeprom->type = type;
		eeprom->byte_write = &eeprom_byte_write;
		eeprom->burst_write = &eeprom_burst_write;
		eeprom->byte_read = &eeprom_byte_read;
		eeprom->burst_read = &eeprom_burst_read;

		eeprom_initialize( i2c );
	}
#endif




AT24C_ReturnType eeprom_initialize( struct vtI2C * i2c )
{
	if( i2c == NULL )
		return AT24C_FAIL;

	i2c->init( i2c->mcu_clock_mhz, i2c->channel, i2c->mode );
	return AT24C_OK;
}

AT24C_ReturnType eeprom_byte_write( AT24C_Type type, struct vtI2C * i2c, uint8_t devid_lsn, uint16_t addr, uint8_t * wrbyte )
{
	if( i2c == NULL )
		return AT24C_FAIL;

	uint8_t _devid = concat_devid_lsn( devid_lsn );
	uint8_t _memaddr[2U];
	prep_mem_addr( addr, _memaddr );
	
	i2c->start( i2c->channel );
	i2c->send_slave_id( i2c->channel,_devid );
	i2c->write( i2c->channel, (( type == AT24C16 ) ? 0x01 : 0x02 ),_memaddr );
	i2c->write( i2c->channel, 1U, wrbyte );
	i2c->stop( i2c->channel );

	return AT24C_OK;
}

AT24C_ReturnType eeprom_burst_write( AT24C_Type type, struct vtI2C * i2c, uint8_t devid_lsn, uint16_t addr, uint8_t count, uint8_t * wrbyte )
{
	if(( count < 1U ) || ( i2c == NULL ))
		return AT24C_FAIL;


	uint8_t _memaddr[2];
	uint8_t _devid = concat_devid_lsn( devid_lsn );
	prep_mem_addr( addr, _memaddr );

	i2c->start( i2c->channel );
	i2c->send_slave_id( i2c->channel,_devid );
	i2c->write( i2c->channel, (( type == AT24C16 ) ? 0x01 : 0x02 ),_memaddr );
	i2c->write( i2c->channel, count, wrbyte );
	i2c->stop( i2c->channel );

	return AT24C_OK;
}

AT24C_ReturnType eeprom_byte_read( AT24C_Type type, struct vtI2C * i2c, ReadAccessType rdAccess, uint8_t devid_lsn, uint16_t addr, uint8_t * rdbyte )
{
	if( i2c == NULL )
		return AT24C_FAIL;

	uint8_t _devid = concat_devid_lsn( devid_lsn );
	if( rdAccess == READ_CURRENT )
	{
		i2c->start( i2c->channel );
		i2c->read( i2c->channel, (_devid | 0x01 ), 1U, rdbyte );
	}
	else
	{
		uint8_t _memaddr[2U];
		prep_mem_addr( addr,_memaddr );
		
		i2c->start( i2c->channel );
		i2c->send_slave_id( i2c->channel,_devid );
		i2c->write( i2c->channel, (( type == AT24C16 ) ? 0x01 : 0x02 ),_memaddr );
		i2c->restart( i2c->channel );
		i2c->read( i2c->channel, (_devid | 0x01U ), 1U, rdbyte );
	}

	return AT24C_OK;
}

/*/// deprecated
void eeprom_currentaddr_byte_read( I2C_CH i2c_sel, uint8_t devid_lsn, uint8_t * rdbyte )
{
	uint8_t _devid = concat_devid_lsn( devid_lsn ) | 0x01;
	
	i2c.start( i2c->channel );
	i2c.read( i2c->channel,_devid, 1U, rdbyte );
}

/// deprecated
void eeprom_rand_byte_read( AT24C_Type type, I2C_CH i2c_sel, uint8_t devid_lsn, uint16_t addr, uint8_t * rdbyte )
{
	uint8_t _memaddr[2U];
	uint8_t _devid = concat_devid_lsn( devid_lsn );
	prep_mem_addr( addr, _memaddr );
	
	i2c.start( i2c_sel );
	i2c.send_slave_id( i2c_sel,_devid );
	i2c.write( i2c_sel, (( type == AT24C16 ) ? 0x01 : 0x02 ),_memaddr );
	i2c.restart( i2c_sel );
	i2c.read( i2c_sel, (_devid | 0x01U ), 1U, rdbyte );
}
*/

AT24C_ReturnType eeprom_burst_read( AT24C_Type type, struct vtI2C * i2c, ReadAccessType rdAccess, uint8_t devid_lsn, uint16_t addr, uint8_t count, uint8_t * rdbyte )
{
	if(( count < 1U ) || ( i2c == NULL ))
		return AT24C_FAIL;

	uint8_t _devid = concat_devid_lsn( devid_lsn );
	if( rdAccess == READ_CURRENT )
	{
		i2c->start( i2c->channel );
		i2c->read( i2c->channel, (_devid | 0x01U ), count, rdbyte );
	}
	else
	{
		uint8_t _memaddr[2U];
		prep_mem_addr( addr, _memaddr );

		i2c->start( i2c->channel );
		i2c->send_slave_id( i2c->channel,_devid );
		i2c->write( i2c->channel, (( type == AT24C16 ) ? 0x01 : 0x02 ),_memaddr );
		i2c->restart( i2c->channel );
		i2c->read( i2c->channel, (_devid | 0x01U ), count, rdbyte );
	}

	return AT24C_OK;
}

/*/// deprecated
void eeprom_currentaddr_seq_read( I2C_CH i2c_sel, uint8_t devid_lsn, uint8_t count, uint8_t * rdbyte )
{
	if( count > 0 )
	{
		uint8_t _devid = concat_devid_lsn( devid_lsn ) | 0x01;
	
		i2c.start( i2c_sel );
		i2c.read( i2c_sel,_devid, count, rdbyte );
	}
}

/// deprecated
void eeprom_rand_seq_read( AT24C_Type type, I2C_CH i2c_sel, uint8_t devid_lsn, uint16_t addr, uint8_t count, uint8_t * rdbyte )
{
	if( count > 0 )
	{
		uint8_t _memaddr[2U];
		uint8_t _devid = concat_devid_lsn( devid_lsn );
		prep_mem_addr( addr, _memaddr );
		
		i2c.start( i2c_sel );
		i2c.send_slave_id( i2c_sel,_devid );
		i2c.write( i2c_sel, (( type == AT24C16 ) ? 0x01 : 0x02 ),_memaddr );
		i2c.restart( i2c_sel );
		i2c.read( i2c_sel, (_devid | 0x01U ), count, rdbyte );
	}
}
*/

/// @brief Combine DEVICE_ID with the least significant nibble (programmed thru strap resistors)
/// @param devid_lsn : Device ID[3..1] (strap resistor values, or block select, depending on device); LSB is masked
/// @return concatenated device id
static uint8_t concat_devid_lsn( uint8_t devid_lsn )
{
	return ( DEVICE_ID | ( devid_lsn & 0x0E ));	
}

/// @brief Split 16b address to two bytes
/// @param addr : 16b address
/// @param memaddr : Pass-by-reference pointer to array where the 2 bytes will be stored
static void prep_mem_addr( uint16_t addr, uint8_t * memaddr )
{
	*memaddr++ = ( addr & 0xFF00 ) >> 8U;
	*memaddr = addr & 0x00FF;
}

