#if !defined( __TRACE )
#define __TRACE


#include <stdio.h>
#include <stdarg.h>

inline void trace( char* pchMsg, ... )
{
#if defined( TRACE_DEBUG )
	va_list	args;
	va_start( args, pchMsg );
	vprintf( pchMsg, args );
	va_end( args );
#endif
}

#endif	// __TRACE
