#include "includes.h"

struct sSysTimer st;

void TimerSys_init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	TIM6->CNT = 0;
	TIM6->PSC = 1;	// Prescaler set to 2
	TIM6->ARR = Apb1Clk / 2 / 1000 - 1; // 1 kHz
	TIM6->DIER = TIM_DIER_UIE;	// Enable update int
	TIM6->CR1 = TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CEN; // Auto-reload preload enable, int only on overflow, counter enable
	NVIC_SetPriority(TIM6_DAC_IRQn, SysTimerPrio);	//set IRQ priority	
	NVIC_EnableIRQ(TIM6_DAC_IRQn);		//enable timer6 int.
}

void TIM6_DAC_IRQHandler(void)
{
	TIM6->SR = 0; // Clear int req
	
	st.ms++;
	st.Flags.f1ms = 1;
	if(st.delay) st.delay--;
	
	if(st.ms >= 1000)
	{
		st.ms -= 1000;
		st.s++;
		st.Flags.f1s = 1;
	}
}