#ifndef __TIMERSYS_H__
#define __TIMERSYS_H__

void TimerSys_init(void);
void Delay(uint16_t delay);
void Delayus(uint32_t delay);

struct sSysTimerFlags
{
	uint16_t f1ms : 1;
	uint16_t f1s : 1;
};

struct sSysTimer
{
	uint16_t delay;
	struct sSysTimerFlags Flags;
	
	uint16_t ms;
	uint16_t s;
};

extern volatile struct sSysTimer st;

#endif // __TIMERSYS_H__