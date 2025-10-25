///===================================================================
/// File Name: stm32f1_i2c.c
/// Type     : STM32 C-source
/// Purpose  : I2C - STM32F103RB Nucleo
/// Version  : 2.2
///===================================================================
/// Description
///		* I2C control for STM32
///		* Threadlock on status polling
///		* Requires external pull-ups
///===================================================================
/// Revision History
/// Version/Date : V1.0 / 2025-Aug-29 / G.RUIZ
///		* Initial release
/// Version/Date : v1.1 / 2025-Sep-03 / G.RUIZ
///		* Added STM32F1 prefix
///		* Replaced argument datatypes with stdint.h datatypes
///		* Added option to change MCU frequency
///		* Renamed to stm32f1_i2c
/// Version/Date : v2.0 / 2025-Oct-12 / G.RUIZ
///		* Renamed to stm32_i2c
///		* Added STM32F4xx compile-time polymorphism
/// Version/Date : v2.1 / 2025-Oct-18 / G.RUIZ
///		* Fixed bug at stm32_i2c_read() on block-reads: stop bit is 
/// 	now set thru SET_BIT (previously was thru WRITE_REG which 
///		turns off the PE bit)
/// Version/Date : v2.2 / 2025-Oct-24 / G.RUIZ
///		* mcu_period_ns now integer-calculated
///		* changed uint32_t mcu_period_hz to uint8_t mcu_period_mhz; 
///===================================================================




///======================= Device Settings ===========================

#include "stm32_i2c.h"

#if defined( USE_STM32F1 )
#define STM32F103xB
#include "stm32f1xx.h"
#elif defined( USE_STM32F4 )
#define STM32F401xC
#include "stm32f4xx.h"
#endif


///========================== Constants ==============================

#define FM_TRE_NS		( 300U			)
#define FM_PERIOD_NS	( 2500U			)
#define SM_TRE_NS		( 1000U			)
#define SM_PERIOD_NS	( 10000U		)





///==================== Function Definitions =========================

static void stm32_i2c_config_and_enable( I2C_TypeDef * i2csel, uint8_t mcu_freq_mhz, uint8_t i2cspeed )
{
	uint8_t mcu_period_ns = 1000U / mcu_freq_mhz;

	/// Toggle I2Cx Reset :
	SET_BIT( i2csel->CR1, I2C_CR1_SWRST );									
	CLEAR_BIT( i2csel->CR1, I2C_CR1_SWRST );				

//	uint32_t mode_freq = i2cspeed == I2C_SM_100KHZ ? 100000U : 400000U;
	uint32_t mode_period_ns = i2cspeed == I2C_SM_100KHZ ? SM_PERIOD_NS : FM_PERIOD_NS;

//	uint32_t mcu_freq_mhz = mcu_freq_hz / 1000000U;
	uint32_t tre_ns = i2cspeed == I2C_SM_100KHZ ? SM_TRE_NS : FM_TRE_NS; 
	uint32_t ccr_freq = (uint16_t)( ( mode_period_ns / 2U ) / (uint16_t)mcu_period_ns );
	uint32_t trise_val = (uint16_t)( ( tre_ns / (uint16_t)mcu_period_ns ) + 1U );

	/// Standard mode: i2c_tpos / sys_period = (1/100e3) / 2 ) / (1/8e6) = 40 (0x28)

	CLEAR_BIT( i2csel->CR1, I2C_CR1_PE );							/// Disable I2C
	MODIFY_REG( i2csel->CR2, I2C_CR2_FREQ, mcu_freq_mhz );			/// Set system FREQ = 8MHz
	MODIFY_REG( i2csel->CCR, I2C_CCR_CCR, ccr_freq );				/// Standard mode, set frequency
	MODIFY_REG( i2csel->TRISE, I2C_TRISE_TRISE, trise_val );		/// Set SCL rising edge time
	SET_BIT( i2csel->CR1, I2C_CR1_PE );								/// Enable I2C
}


#if defined( USE_STM32F1 )

	#define SELECT_I2C_CHAN( ch )		\
		I2C_TypeDef * _i2csel = ch == I2C_CH1 ? I2C1 : I2C2

	void stm32_i2c_init( uint8_t mcu_freq_mhz, uint8_t i2csel, I2C_Mode i2cspeed )
	{
		/// GPIOB & Alt Function clock enable :
		MODIFY_REG( RCC->APB2ENR, ( RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN ), ( RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN ));
		SELECT_I2C_CHAN( i2csel );

		if( i2csel == I2C_CH1 )								/// I2C1 is not ready and I2C1 is being initialized
		{
			//i2c_initialized_mask |= 0x01;
			/// Configure GPIO: Output 10MHz, Open-drain :
			MODIFY_REG( GPIOB->CRL, ( GPIO_CRL_CNF6_Msk | GPIO_CRL_MODE6_Msk ), ( GPIO_CRL_CNF6 | GPIO_CRL_MODE6_0 ));
			MODIFY_REG( GPIOB->CRL, ( GPIO_CRL_CNF7_Msk | GPIO_CRL_MODE7_Msk ), ( GPIO_CRL_CNF7 | GPIO_CRL_MODE7_0 ));
			SET_BIT( RCC->APB1ENR, RCC_APB1ENR_I2C1EN );			/// I2C1 clock enable
		}
		else if( i2csel == I2C_CH2 )
		{
			//i2c_initialized_mask |= 0x02;
			/// Configure GPIO: Output 10MHz, Open-drain :
			MODIFY_REG( GPIOB->CRH, ( GPIO_CRH_CNF10_Msk | GPIO_CRH_MODE10_Msk ), ( GPIO_CRH_CNF10 | GPIO_CRH_MODE10_0 ));
			MODIFY_REG( GPIOB->CRH, ( GPIO_CRH_CNF11_Msk | GPIO_CRH_MODE11_Msk ), ( GPIO_CRH_CNF11 | GPIO_CRH_MODE11_0 ));
			SET_BIT( RCC->APB1ENR, RCC_APB1ENR_I2C2EN );			/// I2C2 clock enable
		}
		else
			return;

		stm32_i2c_config_and_enable(_i2csel, mcu_freq_mhz, i2cspeed );
	}

#elif defined( USE_STM32F4 )

	#define SELECT_I2C_CHAN( ch )		\
		I2C_TypeDef * _i2csel;			\
		switch( ch )					\
		{								\
			case I2C_CH1:				\
				_i2csel = I2C1; break;	\
			case I2C_CH2:				\
				_i2csel = I2C2; break;	\
			case I2C_CH3:				\
				_i2csel = I2C3; break;	\
			default:					\
				_i2csel = I2C1;			\
		};

	void stm32_i2c_init( uint8_t mcu_freq_mhz, uint8_t i2csel, uint8_t i2cspeed )
	{
		SELECT_I2C_CHAN( i2csel );

		if( i2csel == I2C_CH1 )
		{
			/// Set the clocks:
			SET_BIT( RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN );

			/// SCL = PB6; SDA = PB7
			MODIFY_REG( GPIOB->MODER, GPIO_MODER_MODER6_Msk, GPIO_MODER_MODER6_1 );		
			MODIFY_REG( GPIOB->MODER, GPIO_MODER_MODER7_Msk, GPIO_MODER_MODER7_1 );		
			SET_BIT( GPIOB->OTYPER, GPIO_OTYPER_OT6 );
			SET_BIT( GPIOB->OTYPER, GPIO_OTYPER_OT7 );
			MODIFY_REG( GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED6_Msk, GPIO_OSPEEDR_OSPEED6_0 );
			MODIFY_REG( GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED7_Msk, GPIO_OSPEEDR_OSPEED7_0 );
			// MODIFY_REG( GPIOB->PUPDR, ( GPIO_PUPDR_PUPD7_Msk || GPIO_PUPDR_PUPD6_Msk ), ( GPIO_PUPDR_PUPD7_0 || GPIO_PUPDR_PUPD6_0 ));

			/// Alt Function: I2C
			MODIFY_REG( GPIOB->AFR[0], GPIO_AFRL_AFSEL6_Msk, GPIO_AFRL_AFSEL6_2 );
			MODIFY_REG( GPIOB->AFR[0], GPIO_AFRL_AFSEL7_Msk, GPIO_AFRL_AFSEL7_2 );
			
			/// Enable I2C clock
			SET_BIT( RCC->APB1ENR, RCC_APB1ENR_I2C1EN );
		}
		else if( i2csel == I2C_CH2 )
		{
			/// Set the clocks:
			SET_BIT( RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN );

			/// SCL = PB10; SDA = PB11
			MODIFY_REG( GPIOB->MODER, GPIO_MODER_MODER10_Msk, GPIO_MODER_MODER10_1 );		
			MODIFY_REG( GPIOB->MODER, GPIO_MODER_MODER11_Msk, GPIO_MODER_MODER11_1 );		
			SET_BIT( GPIOB->OTYPER, GPIO_OTYPER_OT10 );
			SET_BIT( GPIOB->OTYPER, GPIO_OTYPER_OT11 );
			MODIFY_REG( GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED10_Msk, GPIO_OSPEEDR_OSPEED10_0 );
			MODIFY_REG( GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED11_Msk, GPIO_OSPEEDR_OSPEED11_0 );
			// MODIFY_REG( GPIOB->PUPDR, ( GPIO_PUPDR_PUPD11_Msk || GPIO_PUPDR_PUPD10_Msk ), ( GPIO_PUPDR_PUPD11_0 || GPIO_PUPDR_PUPD10_0 ));

			/// Alt Function: I2C
			MODIFY_REG( GPIOB->AFR[1], GPIO_AFRH_AFSEL10_Msk, GPIO_AFRH_AFSEL10_2 );
			MODIFY_REG( GPIOB->AFR[1], GPIO_AFRH_AFSEL11_Msk, GPIO_AFRH_AFSEL11_2 );

			/// Enable I2C clock
			SET_BIT( RCC->APB1ENR, RCC_APB1ENR_I2C2EN );
		}
		else if( i2csel == I2C_CH3 )
		{
			/// Set the clocks:
			SET_BIT( RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN );
			SET_BIT( RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN );

			/// SCL = PA8; SDA = PC9
			MODIFY_REG( GPIOA->MODER, GPIO_MODER_MODER8_Msk, GPIO_MODER_MODER8_1 );		
			MODIFY_REG( GPIOC->MODER, GPIO_MODER_MODER9_Msk, GPIO_MODER_MODER9_1 );		
			SET_BIT( GPIOA->OTYPER, GPIO_OTYPER_OT8 );
			SET_BIT( GPIOC->OTYPER, GPIO_OTYPER_OT9 );
			MODIFY_REG( GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED8_Msk, GPIO_OSPEEDR_OSPEED8_0 );
			MODIFY_REG( GPIOC->OSPEEDR, GPIO_OSPEEDR_OSPEED9_Msk, GPIO_OSPEEDR_OSPEED9_0 );
			// MODIFY_REG( GPIOA->PUPDR, GPIO_PUPDR_PUPD8_Msk, GPIO_PUPDR_PUPD8_0 );
			// MODIFY_REG( GPIOC->PUPDR, GPIO_PUPDR_PUPD9_Msk, GPIO_PUPDR_PUPD9_0 );

			/// Alt Function: I2C
			MODIFY_REG( GPIOA->AFR[1], GPIO_AFRH_AFSEL8_Msk, GPIO_AFRH_AFSEL8_2 );
			MODIFY_REG( GPIOC->AFR[1], GPIO_AFRH_AFSEL9_Msk, GPIO_AFRH_AFSEL9_2 );

			/// Enable I2C clock
			SET_BIT( RCC->APB1ENR, RCC_APB1ENR_I2C3EN );
		}

		stm32_i2c_config_and_enable(_i2csel, mcu_freq_mhz, i2cspeed );
	}

#endif


void stm32_i2c_start( uint8_t i2csel )
{
	SELECT_I2C_CHAN( i2csel );
	while( READ_BIT(_i2csel->SR2, I2C_SR2_BUSY )){}				/// Check BUSY status *
	SET_BIT(_i2csel->CR1, I2C_CR1_START );						/// Generate start
	while( !READ_BIT(_i2csel->SR1, I2C_SR1_SB )){}				/// Check Start sequence *
}

void stm32_i2c_restart( uint8_t i2csel )
{
	SELECT_I2C_CHAN( i2csel );
	READ_REG(_i2csel->DR );										/// Dummy-read DR to clear BTF status bit
	SET_BIT(_i2csel->CR1, I2C_CR1_START );						/// Generate restart
	while( !READ_BIT(_i2csel->SR1, I2C_SR1_SB )){}				/// Check Start sequence *
}

void stm32_i2c_stop( uint8_t i2csel )
{
	SELECT_I2C_CHAN( i2csel );
	SET_BIT(_i2csel->CR1, I2C_CR1_STOP );
}

void stm32_i2c_send_slave_id( uint8_t i2csel, uint8_t devid )
{
	SELECT_I2C_CHAN( i2csel );
	WRITE_REG(_i2csel->DR, devid );								/// Write device id 
	while( !READ_BIT(_i2csel->SR1, I2C_SR1_ADDR )){}			/// Wait until AddrStatus is asserted
	READ_REG(_i2csel->SR2 );									/// Read SR2 to clear SR1[ADDR] bit
}

void stm32_i2c_write( uint8_t i2csel, uint32_t bytecount, uint8_t * wrbyte )
{
	SELECT_I2C_CHAN( i2csel );
	while( !READ_BIT(_i2csel->SR1, I2C_SR1_TXE )){}				/// Wait for ADDR tx *
	
	for( uint32_t i = 0; i < bytecount; i++ )
	{
		WRITE_REG(_i2csel->DR, *wrbyte++ );						/// Send wrbyte
		while( !READ_BIT(_i2csel->SR1, I2C_SR1_TXE )){}			/// Wait for ADDR tx *
	}

	while( !READ_BIT(_i2csel->SR1, I2C_SR1_BTF )){}				/// Wait for last byte tx *
}

void stm32_i2c_read( uint8_t i2csel, uint8_t devid, uint32_t bytecount, uint8_t * rdbyte )
{
	SELECT_I2C_CHAN( i2csel );
	WRITE_REG(_i2csel->DR, devid );								/// Write device id

	if( bytecount == 1U )
	{
		CLEAR_BIT(_i2csel->CR1, I2C_CR1_ACK );					/// Turn off Master Ack
		while( !READ_BIT(_i2csel->SR1, I2C_SR1_ADDR )){}		/// Wait until AddrStatus is asserted
		READ_REG(_i2csel->SR2 );								/// Read SR2 to clear SR1[ADDR] bit
		SET_BIT(_i2csel->CR1, I2C_CR1_STOP );					/// Generate Stop
		while( !READ_BIT(_i2csel->SR1, I2C_SR1_RXNE )){}		/// Wait until byte rx is done *
		*rdbyte = READ_REG(_i2csel->DR );						/// Read Data Register
	}
	else if( bytecount == 2U )
	{
		SET_BIT(_i2csel->CR1, I2C_CR1_POS );					/// Set POS bit
		SET_BIT(_i2csel->CR1, I2C_CR1_ACK );					/// Turn on Master Ack
		while( !READ_BIT(_i2csel->SR1, I2C_SR1_ADDR )){}		/// Wait until AddrStatus is asserted
		READ_REG(_i2csel->SR2 );								/// Read SR2 to clear SR1[ADDR] bit
		CLEAR_BIT(_i2csel->CR1, I2C_CR1_ACK );					/// Turn off Master Ack
		while( !READ_BIT(_i2csel->SR1, I2C_SR1_BTF )){}			/// Wait until Shift Register and Data Register are filled *
		SET_BIT(_i2csel->CR1, I2C_CR1_STOP );					/// Generate Stop
		*rdbyte++ = READ_REG(_i2csel->DR );						/// Read second-to-last byte
		*rdbyte = READ_REG(_i2csel->DR );						/// Read last byte
		CLEAR_BIT(_i2csel->CR1, I2C_CR1_POS );					/// Cleanup: Turn off POS bit
	}
	else
	{
		while( !READ_BIT(_i2csel->SR1, I2C_SR1_ADDR )){}		/// Wait until AddrStatus is asserted
		READ_REG(_i2csel->SR2 );								/// Read SR2 to clear SR1[ADDR] bit
		SET_BIT(_i2csel->CR1, I2C_CR1_ACK );					/// Turn on Master Ack
		
		while( bytecount > 0 )
		{
			if( bytecount > 3U )
			{
				while( !READ_BIT(_i2csel->SR1, I2C_SR1_RXNE )){}	/// Wait until byte rx is done *
				*rdbyte++ = READ_REG(_i2csel->DR );				/// Read incoming byte
				bytecount--;
			}
			else
			{
				while( !READ_BIT(_i2csel->SR1, I2C_SR1_RXNE )){}	/// Wait until data[n-2] is received *
				bytecount--;
				while( !READ_BIT(_i2csel->SR1, I2C_SR1_BTF )){}		/// Wait until data[n-1] is received *
				CLEAR_BIT(_i2csel->CR1, I2C_CR1_ACK );				/// Turn off Master Ack
				*rdbyte++ = READ_REG(_i2csel->DR );					/// Read data[n-2] to continue reading data[n] in I2C
				bytecount--;
				while( !READ_BIT(_i2csel->SR1, I2C_SR1_RXNE )){}	/// Wait until data[n] is received *
				SET_BIT(_i2csel->CR1, I2C_CR1_STOP );				/// Generate Stop
				*rdbyte++ = READ_REG(_i2csel->DR );					/// Read data[n-1] to continue reading data[n] in I2C
				bytecount--;
				while( !READ_BIT(_i2csel->SR1, I2C_SR1_RXNE )){}
				*rdbyte = READ_REG(_i2csel->DR );					/// Read data[n]
			}
		}
	}
}

