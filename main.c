#include "includes.h"
#include "main.h"

int main(void)
{
	// Enable periph
  	__IO uint32_t tmpio;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	tmpio = RCC->APB2ENR & RCC_APB2ENR_SYSCFGEN;
	(void)tmpio;
	
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	tmpio = RCC->APB1ENR & RCC_APB1ENR_PWREN;
	(void)tmpio;
	
	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

	SystemClock_Config(0, 48);
	SystemCoreClockUpdate();

	while (1)
	{
	}
}

