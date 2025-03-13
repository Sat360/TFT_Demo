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
			