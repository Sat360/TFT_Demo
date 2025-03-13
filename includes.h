#include "STM32F303_Clock.h"
#include "GPIO.h"
#include "LED.h"

#define Error_Handler() {while(1);}
extern __IO uint32_t tmpio;