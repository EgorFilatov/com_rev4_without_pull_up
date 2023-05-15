#if !defined(__TIMER_H)

#define __TIMER_H

#include "types.h"

#define TIMER_ON        TRUE
#define TIMER_OFF       FALSE
#define TIMEREVENT_ON   TRUE
#define TIMEREVENT_OFF  FALSE

class CTimer
{
public:

	static unsigned long m_wCountTimer;

	unsigned long m_wCountTick;
	unsigned long m_wDefaultCountTick;
	unsigned long m_wCountTick_start;

	//unsigned * m_pCountTick_RealT;

	//static byte m_ucCallOldInt;
	//static unsigned m_wCountTick_Real;

	static unsigned long m_wCountTick_RealmSec;

protected:

public:
	struct Fl_Timer {
        unsigned On     :1;
        unsigned Fl     :1;
        unsigned mSec   :1;
        unsigned        :5;
	} Tm;


	CTimer (unsigned long lDefaultTime = 0, bool bONEvent = TIMEREVENT_OFF, bool bON = TIMER_OFF);
	~CTimer ();

	void Run (void);

	unsigned long GetDefault (void);
	void SetDefault (unsigned long lDefaultTime, bool bONEvent = TIMEREVENT_OFF);
	void Set (unsigned long lTime);
	void Reset (void);

	void Off (void);
	void SetEvent (void);

	bool GetEvent (void);
	bool IsEvent (void);
	bool IsOn (void);
	bool IsOff (void);

	unsigned long GetCurrentTime (void);       // for up to one minute
};

#endif

