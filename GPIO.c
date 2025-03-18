#include "GPIO.h"

void InitPin(GPIO_TypeDef *GPIO, uint8_t pin, uint8_t mode, uint8_t otype, uint8_t speed, uint8_t pupd)
{
	GPIO->MODER &= ~(3 << (pin * 2));
	GPIO->MODER |= mode << (pin * 2);
	
	GPIO->OTYPER &= ~(1 << pin);
	GPIO->OTYPER |= otype << pin;
	
	GPIO->OSPEEDR &= ~(3 << (pin * 2));
	GPIO->OSPEEDR |= speed << (pin * 2);
	
	GPIO->PUPDR &= ~(3 << (pin * 2));
	GPIO->PUPDR |= pupd << (pin * 2);
}

void SetAltPin(GPIO_TypeDef *GPIO, uint8_t pin, uint8_t func)
{
	if(pin <= 7)
	{
		GPIO->AFR[0] &= ~(0x0F << (pin * 4));
		GPIO->AFR[0] |= func << (pin * 4);
	}
	else
	{
		GPIO->AFR[1] &= ~(0x0F << ((pin - 8) * 4));
		GPIO->AFR[1] |= func << ((pin - 8) * 4);
	}
}