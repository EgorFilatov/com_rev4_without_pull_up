#if !defined( ___TYPES_1 )
#define  ___TYPES_1

#pragma pack(push, 1)

#define SET_INTERRUPT       asm { sti }
#define CLEAR_INTERRUPT     asm { cli }
#define END_OF_INTERRUPT    asm { mov     al,20h ;  out     20h,al }

//#define DEBUG
//#define TRACE_DEBUG
//#define TIMER_DEBUG
//#define COMMAND_DEBUG
//#define USED_HIMEMORY

#if defined( DEBUG )
	#undef  NDEBUG
#else
	#define NDEBUG
	#undef  DEBUG
#endif

//#include "_null.h"
#include "assert.h"
#include "trace.h"

#if !defined( ASSERT )
	#define ASSERT	assert
#endif

#if !defined( TRACE )
	#define	TRACE	trace
#endif

#if !defined( NET_TRACE )
	#define NET_TRACE   net_trace_inline
#endif

#if !defined( byte )
	typedef	unsigned char		byte;
#endif
#if !defined( uint )
	typedef unsigned int		uint;
#endif
#if !defined( ushort )
//	typedef unsigned short		ushort;
	typedef uint16_t			ushort;
#endif

//typedef unsigned int			STEP;
typedef uint16_t				STEP;

#if !defined( ulong )
	typedef unsigned long		ulong;
#endif
//#if !defined( bool )
//	typedef enum { FALSE, TRUE }	bool;
//#endif

#if !defined( _bool )
	typedef uint16_t			bool2;
#endif

#if !defined( FALSE )
	#define FALSE		    	0
#endif
#if !defined( TRUE )
	#define TRUE		    	1
#endif

#if !defined( bin )
	typedef unsigned long		bin;
	#define BINMASKDEFAULT  	(bin) -1
	#define BINMAXBITPOS    	31
#endif
#if !defined( ms )
	typedef unsigned long		ms;
#endif
#if !defined( stat )
//	typedef unsigned int		state;
	typedef uint16_t			state;
#endif

struct  __POSITION {};
typedef __POSITION* POSITION;

//typedef uint    				ADDRESS;
typedef uint16_t	    		ADDRESS;
#define ERRNUM					0xFFFF
#define DOS_ERROR				-1

typedef struct __EVENT_TIME
{
		byte            Year;
		unsigned char   Month  :4;
		unsigned char   Day    :5;
		unsigned char   Hour   :6;
		unsigned char   Fl_StandartBias :1;
		byte            Minutes;
		unsigned short  mSecond;
} EVENT_TIME;

typedef enum
{
	QUALITY_GOOD_NONSPEC		= 0x00,
	QUALITY_GOOD_TI_MIN_ALARM_TS_ALARM,
	QUALITY_GOOD_TI_MAX_ALARM_TS_FAULT,
	QUALITY_GOOD_TI_MIN_FAULT,
	QUALITY_GOOD_TI_MAX_FAULT,
	QUALITY_BAD_CONFIG,
	QUALITY_BAD_NOTCONNECTED,
	QUALITY_BAD_DEVICEFAILED,
	QUALITY_BAD_SENSORFAILED,
	QUALITY_BAD_COMMFAILED,
	QUALITY_BAD_OUTOFSERVICE,
	QUALITY_UNCERTAIN_TI_TS,
	QUALITY_UNCERTAIN_CALC
}   QUALITY;

void MakeRealTime();
void GetRealTime( EVENT_TIME* pEventTime );
void SetSumWinChange( void );
uint GetFl_StandartBiasSysTime();

#pragma pack(pop)

#endif
