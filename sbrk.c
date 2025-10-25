

register char * stack_ptr asm( "sp" );

void * _sbrk( int incr )
{
	extern char __bss_end__;	/// From linker file
	static char * heap_end;
	char * prev_heap_end;

	if( heap_end == 0 )
		heap_end = &__bss_end__;

	prev_heap_end = heap_end;

	/// If encountering heap and stack collision:
	if( heap_end + incr > stack_ptr )
	{
		while(1){}
	}

	heap_end += incr;
	return (void *)prev_heap_end;
}