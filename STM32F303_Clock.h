#ifndef __SYSTEMCLOCK_CONFIG_H__
#define __SYSTEMCLOCK_CONFIG_H__

#include "stm32f3xx.h"

#define HSE_VALUE 10000000U
#define HSI_VALUE 8000000U

void SystemClock_Config(uint8_t Src, uint8_t Freq);

#endif // __SYSTEMCLOCK_CONFIG_H__