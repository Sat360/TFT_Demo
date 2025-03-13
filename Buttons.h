#ifndef __BUTTONS_H__
#define __BUTTONS_H__

#define Button_pin 0
#define Button_port GPIOA

struct ButtonState
{
	uint8_t AntiBounce;
	uint8_t State;
};

extern struct ButtonState Button; 

void Buttons_init(void);
void Buttons_tick(void);

#endif // __BUTTONS_H__