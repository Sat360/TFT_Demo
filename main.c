#include "includes.h"
#include "main.h"

__IO uint32_t tmpio;

int main(void)
{
	SystemClock_Config(0, 48);
	SystemCoreClockUpdate();
	
	LED_init();
	LED_set(1);

	while (1)
	{
	}
}

