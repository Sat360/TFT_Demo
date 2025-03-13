#include "includes.h"
#include "main.h"

__IO uint32_t tmpio;

int main(void)
{
	uint8_t led = 0;
	SystemClock_Config(0, 48);
	SystemCoreClockUpdate();
	
	LED_init();
	TimerSys_init();
	Buttons_init();

	LED_set(led);

	while (1)
	{
		if(st.Flags.f1ms)
		{
			st.Flags.f1ms = 0;
			Buttons_tick();
			if(Button.State)
				LED_set(0xFF);
			else
				LED_set(led);
		}
		
		if(st.Flags.f1s)
		{
			st.Flags.f1s = 0;
			led++;
		}
	}
}

