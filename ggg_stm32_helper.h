///===================================================================
/// File Name: ggg_stm32_helper.h
/// Type     : STM32 C-header
/// Purpose  : Debug Helper
/// Version  : 1.1
///===================================================================
/// Description
///		* Debug helper for STM32
///		* Wait-delay implemented on TIM2
///		* Uses PC13 LED and PA0 button
///		* Includes a half-duplex implementation of UART to COM3.
///		* Enables us to use printf() thru stdio.h.
///		* Included are function redirects for missing library calls
///		from stdio.h for printf when using arm-none-eabi-gcc -nostdlib
///
/// I've opted to do compile-time polymorphism due to contentions
/// when enabling all CMSIS device headers for the virtual-tables.
/// In addition, the code is organized by device functions, rather than
/// have the #ifdefs inside the functions themselves for clarity.
///  
/// Resources Used
///		* PC13	: LED low-speed output
///		* PA0	: Button input with pull-up enabled
///		* PA2	: USART2 TX
///===================================================================
/// Revision History
/// Version/Date : V1.0 / 2025-Oct-12 / G.RUIZ
///		* Initial release
///		* Implementations for STM32F401CC and STM32F103RB
/// Version/Date : V1.1 / 2025-Oct-17 / G.RUIZ
///		* Added conv_byte_to_chararr()
///===================================================================


#include <stdint.h>

/// @attention Device selection. Define only one:
//#define USE_STM32F1
#define USE_STM32F4


/// @attention Modify to fit your external clock frequency (in Hz):
#define MCU_EXT_FREQ_HZ	( 25000000U	)
//#define MCU_EXT_FREQ_HZ	( 8000000U	)



///======================= Device Settings ===========================

#if defined( USE_STM32F4 )
	/// @attention Enable the chosen device (refer to CMSIS device header)
	#define STM32F401xC
	#include "stm32f4xx.h"

	#define MCU_HSI_FREQ_HZ	( 16000000U )

#elif defined( USE_STM32F1 )
	/// @attention Enable the chosen device (refer to CMSIS device header)
	#define STM32F103xB
	#include "stm32f1xx.h"
	
	#define MCU_HSI_FREQ_HZ	( 8000000U	)

#else
	#error "[ERROR] No valid STM32 device enabled !!"
#endif


/// @brief Delay Counter frequency, used as prescaler for TIM2 wait-delay
#define DLYCTR_FREQ_HZ	( 1000U		)





///=================== Function Declarations =========================

///********************* LED and Pushbutton **************************

/// @brief Sets PC13 to LED and PA0 to KEY button
/// PA0 has internal pull-ups enabled.
void stm32_init_led_and_button( void );

/// @brief Set LED on or off depending on logic.
/// LED logic is inverted on Black/Blue Pills ( LED on = 0x00 ).
void stm32_led_set( uint8_t logic );

/// @brief Inverts LED's last state
void stm32_led_toggle( void );

/// @brief Read PA0 input. Reminder: pull-up enabled (active = 0V)
/// @return 0x01 = button active; 0x00 = button idle
uint8_t stm32_button_pressed( void );


///*********************** TIM2 Wait-Delay ***************************

/// @brief  Blocking function to wait for delay_ms
/// @param delay in milliseconds (ms)
/// @param waitonly if 0x00, set the given delay_ms, then wait;
///					if 0x01, use the previously set delay
void stm32_set_and_wait_ms( uint32_t delay_ms, uint8_t waitonly );

/// @brief Sets the MCU clock source to HSE (Black Pill: 25MHz crystal)
/// @param hse_ena if 0x01, use HSE; if 0x00, use HSI (16MHz)
/// @param is_clksrc 0x01 = a clock source was used; 0x00 a crystal oscillator was used
void stm32_set_mcu_clk_to_hse( uint8_t hse_ena, uint8_t is_clksrc );


///********************** UART2 Half-Duplex **************************

/// @brief Run USART2 as half-duplex (single wire) mode, baud rate 115200 bps
void stm32_init_sw_uart( void );

/// @brief Perform UART write for character
static void stm32_uart_write( uint8_t a );

/// @brief Convert a byte to a char array of size = 2
/// @param byte The byte to be converted
/// @param result The return array of size = 2
void byte_to_hex_chars( uint8_t byte, char result[2] );



///=============== STM32F4 Function Definitions ======================
#if defined( USE_STM32F4 )

///********************* LED and Pushbutton **************************

	void stm32_init_led_and_button( void )
	{
		/// Enable peripheral clocks
		MODIFY_REG( RCC->AHB1ENR, (RCC_AHB1ENR_GPIOCEN_Msk | RCC_AHB1ENR_GPIOAEN_Msk ), (RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOAEN ));

		/// PC13 Config: general purpose output
		MODIFY_REG( GPIOC->MODER, GPIO_MODER_MODER13_Msk, ( GPIO_MODER_MODER13_0 ));

		/// PA0 Config: Pull-up enabled
		MODIFY_REG( GPIOA->PUPDR, GPIO_PUPDR_PUPD0_Msk, GPIO_PUPDR_PUPD0_0 );
	}

	void stm32_led_set( uint8_t logic )
	{
		if( logic )
			SET_BIT( GPIOC->ODR, GPIO_ODR_OD13 );
		else
			CLEAR_BIT( GPIOC->ODR, GPIO_ODR_OD13 );
	}

	void stm32_led_toggle( void )
	{
		if( READ_BIT( GPIOC->ODR, GPIO_ODR_OD13 ))
			stm32_led_set( 0x00 );
		else
			stm32_led_set( 0x01 );
	}

	uint8_t stm32_button_pressed( void )
	{
		return ( ~READ_BIT( GPIOA->IDR, GPIO_IDR_ID0 ) & GPIO_IDR_ID0_Msk );
	}

///*********************** TIM2 Wait-Delay ***************************

	static void stm32_init_delay( uint16_t prescaler, uint32_t reloadvalue )
	{
		/// Counter disable
		CLEAR_BIT( TIM2->CR1, TIM_CR1_CEN );

		/// Enable TIM2 clock
		SET_BIT( RCC->APB1ENR, RCC_APB1ENR_TIM2EN );

		/// Enable TIM2 clock even when on Low-Power mode
		SET_BIT( RCC->APB1LPENR, RCC_APB1LPENR_TIM2LPEN );

		/// Set prescaler
		WRITE_REG( TIM2->PSC, prescaler );

		/// Set autoreload value
		WRITE_REG( TIM2->ARR, reloadvalue );

		/// Clear counter
		WRITE_REG( TIM2->CNT, 0x00000000UL );

		/// Counter enable
		SET_BIT( TIM2->CR1, TIM_CR1_CEN );
	}

	static void stm32_wait_delay( void )
	{
		CLEAR_BIT( TIM2->SR, TIM_SR_UIF );
		while( !READ_BIT( TIM2->SR, TIM_SR_UIF )){}
//		CLEAR_BIT( TIM2->SR, TIM_SR_UIF );
	}

	void stm32_set_and_wait_ms( uint32_t delay_ms, uint8_t waitonly )
	{
		if( waitonly == 0x00 )
		{
			uint8_t use_hse = 0x00;
			/// Check which clock source is being used as SysClk
			if( (( READ_REG( RCC->CFGR ) & RCC_CFGR_SWS_Msk ) >> RCC_CFGR_SWS_Pos ) == RCC_CFGR_SW_HSE )
				use_hse = 0x01;

			/// Initially set prescaler to 1 count = 1ms (1kHz)
			uint16_t prescaler = ( ( use_hse ? MCU_EXT_FREQ_HZ : MCU_HSI_FREQ_HZ ) / DLYCTR_FREQ_HZ ) - 1U;
			
			stm32_init_delay( prescaler, ( delay_ms - 1 ) );
		}

		stm32_wait_delay();
	}

	void stm32_set_mcu_clk_to_hse( uint8_t hse_ena, uint8_t is_clksrc )
	{
		if( hse_ena )
		{
			/// Enable HSE
			SET_BIT( RCC->CR, RCC_CR_HSEON );

			/// Wait until HSE stabilizes
			while( !READ_BIT( RCC->CR, RCC_CR_HSERDY )){}

			/// If a clock source is used (not a crystal oscillator)
			if( is_clksrc )
				SET_BIT( RCC->CR, RCC_CR_HSEBYP );

			/// Select HSE as SysClk
			MODIFY_REG( RCC->CFGR, RCC_CFGR_SW_Msk, RCC_CFGR_SW_0 );

			/// If SysClk switch failed...
			if((( READ_REG( RCC->CFGR ) & RCC_CFGR_SWS_Msk ) >> RCC_CFGR_SWS_Pos ) != RCC_CFGR_SW_HSE )
			{
				/// Select HSI as SysClk
				MODIFY_REG( RCC->CFGR, RCC_CFGR_SW_Msk, 0x00 );

				/// Disable HSE
				CLEAR_BIT( RCC->CR, RCC_CR_HSEON );
			}
		}
		else
		{
			/// Enable HSI
			SET_BIT( RCC->CR, RCC_CR_HSION );

			/// Wait until HSI stabilizes
			while( !READ_BIT( RCC->CR, RCC_CR_HSIRDY )){}

			/// Disable HSE
			CLEAR_BIT( RCC->CR, RCC_CR_HSEON );
		}
	}


///****************** STM32F1 UART2 Half-Duplex **********************

	void stm32_init_sw_uart( void )
	{
		/// Enable GPIOAx clock:
		SET_BIT( RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN );

		/// Enable USART2 clock enable:
		SET_BIT( RCC->APB1ENR, RCC_APB1ENR_USART2EN );

		/// Configure PA2 as USART2_TX:
		MODIFY_REG( GPIOA->MODER, GPIO_MODER_MODER2_Msk, GPIO_MODER_MODER2_1 );
		MODIFY_REG( GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED2_Msk, GPIO_OSPEEDR_OSPEED2_0 );
		MODIFY_REG( GPIOA->AFR[0], GPIO_AFRL_AFSEL2_Msk, ( 0x07 << GPIO_AFRL_AFSEL2_Pos ));

		/// Configure USART2 as single-wire:
		CLEAR_BIT( USART2->CR2, USART_CR2_LINEN );
		CLEAR_BIT( USART2->CR2, USART_CR2_CLKEN );
		MODIFY_REG( USART2->CR3, ( USART_CR3_SCEN_Msk | USART_CR3_HDSEL_Msk | USART_CR3_IREN_Msk ), USART_CR3_HDSEL );

		/// Set baud rate: div = fclk / (8 * (2-OVER8) * 115200) = 13.563368 (using 25MHz fclk)
		/// fraction = 16 * .563368 = 9.014 ~ 9
		/// mantissa = 13
		/// brr = 0xD9
		MODIFY_REG( USART2->BRR, 0xFFFF, 0xD9UL );

		/// Transfer direction:
		SET_BIT( USART2->CR1, USART_CR1_TE );

		/// Enable USART2:
		SET_BIT( USART2->CR1, USART_CR1_UE );
	}

	static void stm32_uart_write( uint8_t a )
	{
		while( !READ_BIT( USART2->SR, USART_SR_TXE )){}
		WRITE_REG( USART2->DR, a & 0xFF );
	}

///=============== STM32F1 Function Definitions ======================
#elif defined( USE_STM32F1 )

///********************* LED and Pushbutton **************************

	void stm32_init_led_and_button( void )
	{
		/// Enable GPIOC(LED) and GPIOA(button) clock
		MODIFY_REG( RCC->APB2ENR, ( RCC_APB2ENR_IOPAEN_Msk | RCC_APB2ENR_IOPCEN_Msk ), ( RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPAEN ));

		/// PC13 settings: output 2MHz push-pull
		MODIFY_REG( GPIOC->CRH, ( GPIO_CRH_CNF13_Msk | GPIO_CRH_MODE13_Msk ), GPIO_CRH_MODE13_1 );

		/// PA0 settings: input with pull-up
		MODIFY_REG( GPIOA->CRL, ( GPIO_CRL_CNF0_Msk | GPIO_CRL_MODE0_Msk ), GPIO_CRL_CNF0_1 );
		MODIFY_REG( GPIOA->ODR, GPIO_ODR_ODR0_Msk, GPIO_ODR_ODR0 );
	}

	void stm32_led_set( uint8_t logic )
	{
		if( logic )
			SET_BIT( GPIOC->BSRR, GPIO_BSRR_BS13 );
		else
		{
		///	CLEAR_BIT( GPIOC->BSRR, GPIO_BSRR_BS13 );
			SET_BIT( GPIOC->BSRR, GPIO_BSRR_BR13 );
		}
	}

	void stm32_led_toggle( void )
	{
		if( READ_BIT( GPIOC->ODR, GPIO_ODR_ODR13 ))
			stm32_led_set( 0x01 );
		else
			stm32_led_set( 0x00 );
	}

	uint8_t stm32_button_pressed( void )
	{
		return ( ~READ_BIT( GPIOA->IDR, GPIO_IDR_IDR0 ) & GPIO_IDR_IDR0_Msk );
	}

///*********************** TIM2 Wait-Delay ***************************

	static void stm32_init_delay( uint16_t prescaler, uint32_t reloadvalue )
	{
		/// Counter disable
		CLEAR_BIT( TIM2->CR1, TIM_CR1_CEN );

		/// Enable TIM2 clock
		SET_BIT( RCC->APB1ENR, RCC_APB1ENR_TIM2EN );

		/// Set prescaler
		WRITE_REG( TIM2->PSC, prescaler );

		/// Set autoreload value
		WRITE_REG( TIM2->ARR, reloadvalue );

		/// Clear counter
		WRITE_REG( TIM2->CNT, 0x00000000UL );

		/// Counter enable
		SET_BIT( TIM2->CR1, TIM_CR1_CEN );
	}

	static void stm32_wait_delay( void )
	{
		CLEAR_BIT( TIM2->SR, TIM_SR_UIF );
		while( !READ_BIT( TIM2->SR, TIM_SR_UIF )){}
	}

	void stm32_set_and_wait_ms( uint32_t delay_ms, uint8_t waitonly )
	{
		if( waitonly == 0x00 )
		{
			uint8_t use_hse = 0x00;
			/// Check which clock source is being used as SysClk
			if( (( READ_REG( RCC->CFGR ) & RCC_CFGR_SWS_Msk ) >> RCC_CFGR_SWS_Pos ) == RCC_CFGR_SW_HSE )
				use_hse = 0x01;

			/// Initially set prescaler to 1 count = 1ms (1kHz)
			uint16_t prescaler = ( ( use_hse ? MCU_EXT_FREQ_HZ : MCU_HSI_FREQ_HZ ) / DLYCTR_FREQ_HZ ) - 1U;
			
			stm32_init_delay( prescaler, ( delay_ms - 1 ) );
		}

		stm32_wait_delay();
	}

	void stm32_set_mcu_clk_to_hse( uint8_t hse_ena, uint8_t is_clksrc )
	{
		if( hse_ena )
		{
			/// Enable HSE
			SET_BIT( RCC->CR, RCC_CR_HSEON );

			/// Wait until HSE stabilizes
			while( !READ_BIT( RCC->CR, RCC_CR_HSERDY )){}

			/// If a clock source is used (not a crystal oscillator)
			if( is_clksrc )
				SET_BIT( RCC->CR, RCC_CR_HSEBYP );

			/// Select HSE as SysClk
			MODIFY_REG( RCC->CFGR, RCC_CFGR_SW_Msk, RCC_CFGR_SW_0 );

			/// If SysClk switch failed...
			if((( READ_REG( RCC->CFGR ) & RCC_CFGR_SWS_Msk ) >> RCC_CFGR_SWS_Pos ) != RCC_CFGR_SW_HSE )
			{
				/// Select HSI as SysClk
				MODIFY_REG( RCC->CFGR, RCC_CFGR_SW_Msk, 0x00 );

				/// Disable HSE
				CLEAR_BIT( RCC->CR, RCC_CR_HSEON );
			}
		}
		else
		{
			/// Enable HSI
			SET_BIT( RCC->CR, RCC_CR_HSION );

			/// Wait until HSI stabilizes
			while( !READ_BIT( RCC->CR, RCC_CR_HSIRDY )){}

			/// Disable HSE
			CLEAR_BIT( RCC->CR, RCC_CR_HSEON );
		}
	}


///****************** STM32F1 UART2 Half-Duplex **********************

	void stm32_init_sw_uart( void )
	{
		/// Enable GPIOAx clock and ALTIO clock:
		MODIFY_REG( RCC->APB2ENR, ( RCC_APB2ENR_IOPAEN_Msk | RCC_APB2ENR_AFIOEN_Msk ), ( RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN ));
		
		/// Enable USART2 clock enable:
		SET_BIT( RCC->APB1ENR, RCC_APB1ENR_USART2EN );
		
		/// Configure PA2 as USART2_TX:
		MODIFY_REG( GPIOA->CRL, ( GPIO_CRL_MODE2_Msk | GPIO_CRL_CNF2_Msk ), ( GPIO_CRL_MODE2_0 | GPIO_CRL_CNF2_1 ));	///A2 OUT
		
		/// Configure USART2 as single-wire:
		CLEAR_BIT( USART2->CR2, USART_CR2_LINEN );
		MODIFY_REG( USART2->CR3, ( USART_CR3_HDSEL_Msk | USART_CR3_SCEN_Msk | USART_CR3_IREN_Msk ), USART_CR3_HDSEL );
		
		/// Set baud rate: div = fclk / (16 * 115200) = 4.340278
		/// fraction = 16 * .340278 = 5.4448 ~ 5
		/// mantissa = 4
		/// brr = 0x45
		MODIFY_REG( USART2->BRR, 0xFFFF, 0x45UL );
		
		/// Transfer direction:
		SET_BIT( USART2->CR1, USART_CR1_TE );
		
		/// Enable USART2:
		SET_BIT( USART2->CR1, USART_CR1_UE );
	}

	static void stm32_uart_write( uint8_t a )
	{
		while( !READ_BIT( USART2->SR, USART_SR_TXE )){}
		WRITE_REG( USART2->DR, a & 0xFF );
	}

#endif


///=============== Callbacks for -nostdlib printf() ==================

/// @brief  Redirects printf() to uart_write
int __io_putchar( int ch )
{
	stm32_uart_write( ch );
	return ch;
}

/// @brief	Redirects puts() to this when using arm-none-eabi-gcc with -nostdlib
int puts( const char * str )
{
	uint32_t ptr = 0U;
	while( str[ptr] != '\0' )
	{
		__io_putchar( str[ptr] );
		ptr++;
	}

	return ptr;
}


/// @brief Manually convert a nibble to a char
/// @param nibble The nibble to be converted
/// @return The nibble in char form
static char nibble_to_char( uint8_t nibble )
{
    if (nibble < 10)
        return '0' + nibble; 		/// For values 0-9
	else
        return 'A' + (nibble - 10);	/// For values 10-15 (A-F)
}

/// @brief Convert a byte to a char array of size = 2
/// @param byte The byte to be converted
/// @param result The return array of size = 2
void byte_to_hex_chars( uint8_t byte, char result[2] )
{
    /// Convert nibbles to characters and store them in the result array.
    result[0] = nibble_to_char(( byte >> 4 ) & 0x0F );
    result[1] = nibble_to_char( byte & 0x0F );
}
