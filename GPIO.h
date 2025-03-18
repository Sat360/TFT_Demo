#ifndef __GPIO_H__
#define __GPIO_H__

#include "stm32f303xc.h"
#include "stdint.h"

// MODER
#define GPIO_MODE_INPUT 	0
#define GPIO_MODE_OUTPUT 	1
#define GPIO_MODE_ALT		2
#define GPIO_MODE_ANALOG	3

// OTYPER (push-pull or open-drain)
#define GPIO_TYPE_PP 0
#define GPIO_TYPE_OD 1

// OSPEEDR
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MED		1
#define GPIO_SPEED_HIGH		3

// PUPDR
#define GPIO_PUPD_NONE		0
#define GPIO_PUPD_PU		1
#define GPIO_PUPD_PD		2

void InitPin(GPIO_TypeDef *GPIO, uint8_t pin, uint8_t mode, uint8_t otype, uint8_t speed, uint8_t pupd);
void SetAltPin(GPIO_TypeDef *GPIO, uint8_t pin, uint8_t func);

#define PortSet(Port,Pin) 	Port->BSRR = (uint32_t)(1 << (Pin + 0))
#define PortReset(Port,Pin) Port->BSRR = (uint32_t)(1 << (Pin + 16))
#define PortCheck(Port,Pin) (Port->IDR & (uint32_t)(1 << Pin))

#endif // __GPIO_H__