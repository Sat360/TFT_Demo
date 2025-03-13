#include "STM32F303_Clock.h"
#include "GPIO.h"
#include "LED.h"
#include "TimerSys.h"
#include "Buttons.h"

#define SysTimerPrio 14

#define Error_Handler() {while(1);}
extern __IO uint32_t tmpio;