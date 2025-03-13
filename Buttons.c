#include "includes.h"

struct ButtonState Button;

void Buttons_init(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	tmpio = RCC->AHBENR & RCC_AHBENR_GPIOAEN;
	(void)tmpio;

	InitPin(Button_port, Button_pin, GPIO_MODE_INPUT, GPIO_TYPE_OD, GPIO_SPEED_LOW, GPIO_PUPD_NONE);
	
	Button.AntiBounce = 0;
	Button.State = 0;
}

void Buttons_tick(void)
{
	if(Button_port->IDR & (1 << Button_pin))
	{
		Button.AntiBounce++;
		if(Button.AntiBounce > 100)
			Button.AntiBounce = 100;
		if(Button.AntiBounce > 70)
			Button.State = 1;
	}
	else
	{
		if(Button.AntiBounce) 
			Button.AntiBounce--;
		if(Button.AntiBounce < 30)
			Button.State = 0;
	}
}