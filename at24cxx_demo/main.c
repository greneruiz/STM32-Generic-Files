


#include <stdint.h>
#include <stdio.h>

#include "ggg_stm32_helper.h"
#include "at24cxx.h"

#define USE_AT24C16

#if defined( USE_AT24C16 )
	#define MEM_ADDR_BYTE_SIZE		( AT24C16_ADDR_BYTE_SIZE		)
	#define MEM_WRITE_CYCLE_TIME_MS	( AT24C16_WRITE_CYCLE_TIME_MS	)
	#define MEM_BYTE_PER_PAGE		( AT24C16_BYTE_PER_PAGE			)
	#define MEM_PAGE_SIZE			( AT24C16_PAGE_SIZE				)
#elif defined( USE_AT24C32 )
	#define MEM_ADDR_BYTE_SIZE		( AT24C32_ADDR_BYTE_SIZE		)
	#define MEM_WRITE_CYCLE_TIME_MS	( AT24C32_WRITE_CYCLE_TIME_MS	)
	#define MEM_BYTE_PER_PAGE		( AT24C32_BYTE_PER_PAGE			)
	#define MEM_PAGE_SIZE			( AT24C32_PAGE_SIZE				)
#elif defined( USE_AT24C64 )
	#define MEM_ADDR_BYTE_SIZE		( AT24C64_ADDR_BYTE_SIZE		)
	#define MEM_WRITE_CYCLE_TIME_MS	( AT24C64_WRITE_CYCLE_TIME_MS	)
	#define MEM_BYTE_PER_PAGE		( AT24C64_BYTE_PER_PAGE			)
	#define MEM_PAGE_SIZE			( AT24C64_PAGE_SIZE				)
#endif


#define LOG_ERROR_AND_HOLD()		\
	{								\
		printf( " FAILED !\n" );	\
		while(1){}					\
	}

AT24C_ReturnType test_rand_byte( struct AT24CXX * eeprom, struct vtI2C * i2c, uint8_t devid_lsn )
{
	uint8_t rdByte = 0x00;
	uint8_t wrByte = 0xA5;
	char result[2] = { ' ', ' ' };
	printf( "test_rand_byte:\n" );

	if( eeprom->byte_write( eeprom->type, i2c, devid_lsn, 0x00, &wrByte ) == AT24C_FAIL )
		return AT24C_FAIL;

	stm32_set_and_wait_ms( MEM_WRITE_CYCLE_TIME_MS, 0x00 );

	if( eeprom->byte_read( eeprom->type, i2c, READ_ADDRESSED, devid_lsn, 0x00, &rdByte ) == AT24C_FAIL )
		return AT24C_FAIL;

	stm32_set_and_wait_ms( MEM_WRITE_CYCLE_TIME_MS, 0x00 );

	printf( " wr = \n" );
	byte_to_hex_chars( wrByte, result );
	puts( result );
	printf( " \n" );

	printf( "; rd = \n" );
	byte_to_hex_chars( rdByte, result );
	puts( result );
	printf( " \n\n" );

	return AT24C_OK;
}

AT24C_ReturnType test_current_byte( struct AT24CXX * eeprom, struct vtI2C * i2c, uint8_t devid_lsn )
{
	uint8_t rdByte[2];
	uint8_t wrByte[2] = { 0xFF, 0x3C };
	char result[2] = { ' ', ' ' };

	printf( "test_current_byte:\n" );
	for( uint16_t k = 0U; k < 2U; k++ )
	{
		if( eeprom->byte_write( eeprom->type, i2c, devid_lsn, k, &wrByte[k] ) == AT24C_FAIL )
			return AT24C_FAIL;
			
		stm32_set_and_wait_ms( MEM_WRITE_CYCLE_TIME_MS, 0x00 );
	}

	if( eeprom->byte_read( eeprom->type, i2c, READ_ADDRESSED, devid_lsn, 0x00, &rdByte[0] ) == AT24C_FAIL )
		return AT24C_FAIL;

	if( eeprom->byte_read( eeprom->type, i2c, READ_CURRENT, devid_lsn, 0x00, &rdByte[1] ) == AT24C_FAIL )
		return AT24C_FAIL;

	stm32_set_and_wait_ms( MEM_WRITE_CYCLE_TIME_MS, 0x00 );

	printf( " wr = \n" );
	byte_to_hex_chars( wrByte[0], result );
	puts( result );
	printf( " \n" );
	byte_to_hex_chars( wrByte[1], result );
	puts( result );

	printf( "; rd = \n" );
	byte_to_hex_chars( rdByte[0], result );
	puts( result );
	printf( " \n" );
	byte_to_hex_chars( rdByte[1], result );
	puts( result );
	printf( " \n\n" );

	return AT24C_OK;
}

AT24C_ReturnType test_page( struct AT24CXX * eeprom, struct vtI2C * i2c, uint8_t devid_lsn )
{
	uint8_t rdArray[MEM_BYTE_PER_PAGE];
	uint8_t wrArray[MEM_BYTE_PER_PAGE];
	char result[2] = { ' ', ' ' };

	for( uint8_t i = 0U; i < MEM_BYTE_PER_PAGE; i++ )
	{
		wrArray[i] = i + 0x01;
	}

	printf( "test_page:\n" );
	if( eeprom->burst_write( eeprom->type, i2c, devid_lsn, 0x00, MEM_BYTE_PER_PAGE, wrArray ) == AT24C_FAIL )
		return AT24C_FAIL;

	stm32_set_and_wait_ms( MEM_WRITE_CYCLE_TIME_MS, 0x00 );

	if( eeprom->burst_read( eeprom->type, i2c, READ_ADDRESSED, devid_lsn, 0x00, MEM_BYTE_PER_PAGE, rdArray ) == AT24C_FAIL )
		return AT24C_FAIL;

	stm32_set_and_wait_ms( MEM_WRITE_CYCLE_TIME_MS, 0x00 );

	printf( " rd = \n" );
	for( uint8_t i = 0U; i < MEM_BYTE_PER_PAGE; i++ )
	{
		byte_to_hex_chars( rdArray[i], result );
		puts( result );
		printf( " \n" );
	}
	printf( " \n\n" );

	return AT24C_OK;
}

AT24C_ReturnType test_rand_2bytes( struct AT24CXX * eeprom, struct vtI2C * i2c, uint8_t devid_lsn )
{
	uint8_t rdByte[2];
	uint8_t wrByte[2] = { 0xAB, 0xCD };
	char result[2] = { ' ', ' ' };
	printf( "\ntest_rand_2bytes: \n" );

	if( eeprom->burst_write( eeprom->type, i2c, devid_lsn, 0x10U, 0x02U, wrByte ) == AT24C_FAIL )
		return AT24C_FAIL;

	stm32_set_and_wait_ms( MEM_WRITE_CYCLE_TIME_MS, 0x00 );

	if( eeprom->burst_read( eeprom->type, i2c, READ_ADDRESSED, devid_lsn, 0x10U, 0x02U, rdByte ) == AT24C_FAIL )
		return AT24C_FAIL;

	stm32_set_and_wait_ms( MEM_WRITE_CYCLE_TIME_MS, 0x00 );

	printf( "wr = \n" );
	byte_to_hex_chars( wrByte[0], result );
	puts( result );
	printf( " \n" );
	byte_to_hex_chars( wrByte[1], result );
	puts( result );

	printf( "; rd = \n" );
	byte_to_hex_chars( rdByte[0], result );
	puts( result );
	printf( " \n" );
	byte_to_hex_chars( rdByte[1], result );
	puts( result );
	printf( " \n\n" );

	return AT24C_OK;
}


int main( void )
{
	uint8_t devid_lsn;
	uint8_t led_logic = 0x00;

	#if defined( USE_AT24C16 )
		devid_lsn = 0x00;	/// use as block select
	#else
		devid_lsn = 0x0E;	/// use straps
	#endif

	stm32_init_led_and_button();
	stm32_set_mcu_clk_to_hse( 0x01, 0x00 );
	stm32_init_sw_uart();

//	vtI2C * i2c = InitializeI2CHandler( 0x01, 25U, I2C_SM );
//	AT24CXX * eeprom = InitializeEEPROM( AT24C16, i2c );

	struct vtI2C i2c;
	struct AT24CXX eeprom;
	InitializeI2CHandler(&i2c, 0x01, 25U, I2C_SM );
	InitializeEEPROM( &eeprom, AT24C16, &i2c );
	stm32_set_and_wait_ms( 500U, 0x00 );

	if( test_rand_2bytes( &eeprom, &i2c, devid_lsn ) == AT24C_FAIL )
	LOG_ERROR_AND_HOLD()

	if( test_rand_byte( &eeprom, &i2c, devid_lsn ) == AT24C_FAIL )
	LOG_ERROR_AND_HOLD()

	if( test_current_byte( &eeprom, &i2c, devid_lsn ) == AT24C_FAIL )
	LOG_ERROR_AND_HOLD()

	if( test_page( &eeprom, &i2c, devid_lsn ) == AT24C_FAIL )
	LOG_ERROR_AND_HOLD()

	while(1)
	{
		stm32_led_set( led_logic );
		stm32_set_and_wait_ms( 500U, 0x00 );
		led_logic = ( ~led_logic ) & 0x01;
	}
}