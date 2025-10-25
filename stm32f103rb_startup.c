/* stm32f103rb_startup.c */
#include <stdint.h>

/* Declare symbols from stm32_ls.ld linker script */
extern uint32_t _estack;
extern uint32_t _etext;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

/* Declare functions required by stm32_ls.ld linker script and main.c */
void Reset_Handler(void);
int main(void);


/* Macro for interrupt handler function definition */
#define DEFAULT_HANDLER_FX( int_name )	\
	void int_name(void)__attribute__(( weak, alias( "Default_Handler" )));


/* Declare interrupt handler functions (see Table 63 from RM0008 STM32F10_ReferenceManual ) */
DEFAULT_HANDLER_FX( NMI_Handler )
DEFAULT_HANDLER_FX( HardFault_Handler )
DEFAULT_HANDLER_FX( MemManage_Handler )
DEFAULT_HANDLER_FX( BusFault_Handler )
DEFAULT_HANDLER_FX( UsageFault_Handler )
DEFAULT_HANDLER_FX( SVCall_Handler )
DEFAULT_HANDLER_FX( DebugMonitor_Handler )
DEFAULT_HANDLER_FX( PendSV_Handler )
DEFAULT_HANDLER_FX( SysTick_Handler )
DEFAULT_HANDLER_FX( WWDG_Handler )
DEFAULT_HANDLER_FX( PVD_Handler )
DEFAULT_HANDLER_FX( Tamper_Handler )
DEFAULT_HANDLER_FX( RTC_Handler )
DEFAULT_HANDLER_FX( Flash_Handler )
DEFAULT_HANDLER_FX( RCC_Handler )
DEFAULT_HANDLER_FX( EXTI0_Handler )
DEFAULT_HANDLER_FX( EXTI1_Handler )
DEFAULT_HANDLER_FX( EXTI2_Handler )
DEFAULT_HANDLER_FX( EXTI3_Handler )
DEFAULT_HANDLER_FX( EXTI4_Handler )
DEFAULT_HANDLER_FX( DMA1_Chan1_Handler )
DEFAULT_HANDLER_FX( DMA1_Chan2_Handler )
DEFAULT_HANDLER_FX( DMA1_Chan3_Handler )
DEFAULT_HANDLER_FX( DMA1_Chan4_Handler )
DEFAULT_HANDLER_FX( DMA1_Chan5_Handler )
DEFAULT_HANDLER_FX( DMA1_Chan6_Handler )
DEFAULT_HANDLER_FX( DMA1_Chan7_Handler )
DEFAULT_HANDLER_FX( ADC1_2_Handler )
DEFAULT_HANDLER_FX( USB_HP_CAN_TX_Handler )
DEFAULT_HANDLER_FX( USB_LP_CAN_RX_Handler )
DEFAULT_HANDLER_FX( CAN_RX1_Handler )
DEFAULT_HANDLER_FX( CAN_SCE_Handler )
DEFAULT_HANDLER_FX( EXTI9_5_Handler )
DEFAULT_HANDLER_FX( TIM1_BRK_Handler )
DEFAULT_HANDLER_FX( TIM1_UP_Handler )
DEFAULT_HANDLER_FX( TIM1_TRG_COM_Handler )
DEFAULT_HANDLER_FX( TIM1_CC_Handler )
DEFAULT_HANDLER_FX( TIM2_Handler )
DEFAULT_HANDLER_FX( TIM3_Handler )
DEFAULT_HANDLER_FX( TIM4_Handler )
DEFAULT_HANDLER_FX( I2C1_EV_Handler )
DEFAULT_HANDLER_FX( I2C1_ER_Handler )
DEFAULT_HANDLER_FX( I2C2_EV_Handler )
DEFAULT_HANDLER_FX( I2C2_ER_Handler )
DEFAULT_HANDLER_FX( SPI1_Handler )
DEFAULT_HANDLER_FX( SPI2_Handler )
DEFAULT_HANDLER_FX( USART1_Handler )
DEFAULT_HANDLER_FX( USART2_Handler )
DEFAULT_HANDLER_FX( USART3_Handler )
DEFAULT_HANDLER_FX( EXTI15_10_Handler )
DEFAULT_HANDLER_FX( RTCAlarm_Handler )
DEFAULT_HANDLER_FX( USBWakeup_Handler )
DEFAULT_HANDLER_FX( TIM8_BRK_Handler )
DEFAULT_HANDLER_FX( TIM8_UP_Handler )
DEFAULT_HANDLER_FX( TIM8_TRG_COM_Handler )
DEFAULT_HANDLER_FX( TIM8_CC_Handler )
DEFAULT_HANDLER_FX( ADC3_Handler )
DEFAULT_HANDLER_FX( FSMC_Handler )
DEFAULT_HANDLER_FX( SDIO_Handler )
DEFAULT_HANDLER_FX( TIM5_Handler )
DEFAULT_HANDLER_FX( SPI3_Handler )
DEFAULT_HANDLER_FX( UART4_Handler )
DEFAULT_HANDLER_FX( UART5_Handler )
DEFAULT_HANDLER_FX( TIM6_Handler )
DEFAULT_HANDLER_FX( TIM7_Handler )
DEFAULT_HANDLER_FX( DMA2_Chan1_Handler  )
DEFAULT_HANDLER_FX( DMA2_Chan2_Handler  )
DEFAULT_HANDLER_FX( DMA2_Chan3_Handler  )
DEFAULT_HANDLER_FX( DMA2_Chan4_5_Handler  )


/* Vector Table */
uint32_t vector_tbl[] __attribute__(( section(".isr_vector_tbl"))) =
{
	(uint32_t)&_estack,
	(uint32_t)&Reset_Handler,
	(uint32_t)&NMI_Handler,
	(uint32_t)&HardFault_Handler,
	(uint32_t)&MemManage_Handler,
	(uint32_t)&BusFault_Handler,
	(uint32_t)&UsageFault_Handler,
	0,
	0,
	0,
	0,
	(uint32_t)&SVCall_Handler,
	(uint32_t)&DebugMonitor_Handler,
	0,
	(uint32_t)&PendSV_Handler,
	(uint32_t)&SysTick_Handler,
	(uint32_t)&WWDG_Handler,
	(uint32_t)&PVD_Handler,
	(uint32_t)&Tamper_Handler,
	(uint32_t)&RTC_Handler,
	(uint32_t)&Flash_Handler,
	(uint32_t)&RCC_Handler,
	(uint32_t)&EXTI0_Handler,
	(uint32_t)&EXTI1_Handler,
	(uint32_t)&EXTI2_Handler,
	(uint32_t)&EXTI3_Handler,
	(uint32_t)&EXTI4_Handler,
	(uint32_t)&DMA1_Chan1_Handler,
	(uint32_t)&DMA1_Chan2_Handler,
	(uint32_t)&DMA1_Chan3_Handler,
	(uint32_t)&DMA1_Chan4_Handler,
	(uint32_t)&DMA1_Chan5_Handler,
	(uint32_t)&DMA1_Chan6_Handler,
	(uint32_t)&DMA1_Chan7_Handler,
	(uint32_t)&ADC1_2_Handler,
	(uint32_t)&USB_HP_CAN_TX_Handler,
	(uint32_t)&USB_LP_CAN_RX_Handler,
	(uint32_t)&CAN_RX1_Handler,
	(uint32_t)&CAN_SCE_Handler,
	(uint32_t)&EXTI9_5_Handler,
	(uint32_t)&TIM1_BRK_Handler,
	(uint32_t)&TIM1_UP_Handler,
	(uint32_t)&TIM1_TRG_COM_Handler,
	(uint32_t)&TIM1_CC_Handler,
	(uint32_t)&TIM2_Handler,
	(uint32_t)&TIM3_Handler,
	(uint32_t)&TIM4_Handler,
	(uint32_t)&I2C1_EV_Handler,
	(uint32_t)&I2C1_ER_Handler,
	(uint32_t)&I2C2_EV_Handler,
	(uint32_t)&I2C2_ER_Handler,
	(uint32_t)&SPI1_Handler,
	(uint32_t)&SPI2_Handler,
	(uint32_t)&USART1_Handler,
	(uint32_t)&USART2_Handler,
	(uint32_t)&USART3_Handler,
	(uint32_t)&EXTI15_10_Handler,
	(uint32_t)&RTCAlarm_Handler,
	(uint32_t)&USBWakeup_Handler,
	(uint32_t)&TIM8_BRK_Handler,
	(uint32_t)&TIM8_UP_Handler,
	(uint32_t)&TIM8_TRG_COM_Handler,
	(uint32_t)&TIM8_CC_Handler,
	(uint32_t)&ADC3_Handler,
	(uint32_t)&FSMC_Handler,
	(uint32_t)&SDIO_Handler,
	(uint32_t)&TIM5_Handler,
	(uint32_t)&SPI3_Handler,
	(uint32_t)&UART4_Handler,
	(uint32_t)&UART5_Handler,
	(uint32_t)&TIM6_Handler,
	(uint32_t)&TIM7_Handler,
	(uint32_t)&DMA2_Chan1_Handler,
	(uint32_t)&DMA2_Chan2_Handler,
	(uint32_t)&DMA2_Chan3_Handler,
	(uint32_t)&DMA2_Chan4_5_Handler
};

void Default_Handler(void)
{
	while(1){}
}


void Reset_Handler(void)
{
	/// Calculate .data and .bss section sizes
	uint32_t data_mem_size = (uint32_t)&_edata - (uint32_t)&_sdata;
	uint32_t bss_mem_size = (uint32_t)&_ebss - (uint32_t)&_sbss;

	/// Copy .data from FLASH to SRAM
	uint32_t *p_src_mem = (uint32_t *)&_etext;
	uint32_t *p_dest_mem = (uint32_t *)&_sdata;
	
	for( uint32_t i = 0; i < data_mem_size; i++ )
	{
		*p_dest_mem++ = *p_src_mem++;
	}

	/// Reset .bss to 0
	p_dest_mem = (uint32_t *)&_sbss;
	
	for( uint32_t i = 0; i < bss_mem_size; i++ )
	{
		*p_dest_mem++ = 0;
	}

	/// Call the main function
	main();
}