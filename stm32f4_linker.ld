ENTRY( Reset_Handler )

MEMORY
{
	FLASH(rx): ORIGIN = 0x08000000, LENGTH = 256K
	SRAM(wrx): ORIGIN = 0x20000000, LENGTH = 64K
}

_estack = ORIGIN(SRAM) + LENGTH(SRAM);
__max_heap_size = 0x100;
__max_stack_size = 0x49FF;

SECTIONS
{
	.text :
	{
		. = ALIGN(4);
		*(.isr_vector_tbl)
		*(.text)
		*(.rodata)
		. = ALIGN(4);
		_etext = .;
	}>FLASH

	.data :
	{
		. = ALIGN(4);
		_sdata = .;
		*(.data)
		. = ALIGN(4);
		_edata = .;
	}>SRAM AT>FLASH

	.bss :
	{
		. = ALIGN(4);
		_sbss = .;
		__bss_start__ = _sbss;
		*(.bss)
		. = ALIGN(4);
		_ebss = .;
		__bss_end__ = _ebss;
	}>SRAM
}