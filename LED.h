#ifndef __LED_H__
#define __LED_H__

#define LED_port GPIOE
#define LED0_pin 8
#define LED1_pin 9
#define LED2_pin 10
#define LED3_pin 11
#define LED4_pin 12
#define LED5_pin 13
#define LED6_pin 14
#define LED7_pin 15

void LED_init(void);
void LED_set(uint8_t leds);

#endif // __LED_H__