#include "stm32f3xx.h"

extern __IO uint32_t tmpio;

#if defined(USER_VECT_TAB_ADDRESS)
/* #define VECT_TAB_SRAM */
#if defined(VECT_TAB_SRAM)
#define VECT_TAB_BASE_ADDRESS   SRAM_BASE       /*!< Vector Table base address field.
                                                     This value must be a multiple of 0x200. */
#define VECT_TAB_OFFSET         0x00000000U     /*!< Vector Table base offset field.
                                                     This value must be a multiple of 0x200. */
#else
#define VECT_TAB_BASE_ADDRESS   FLASH_BASE      /*!< Vector Table base address field.
                                                     This value must be a multiple of 0x200. */
#define VECT_TAB_OFFSET         0x00000000U     /*!< Vector Table base offset field.
                                                     This value must be a multiple of 0x200. */
#endif /* VECT_TAB_SRAM */
#endif /* USER_VECT_TAB_ADDRESS */

void SystemInit(void)
{
/* FPU settings --------------------------------------------------------------*/
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
#endif

  /* Configure the Vector Table location -------------------------------------*/
#if defined(USER_VECT_TAB_ADDRESS)
  SCB->VTOR = VECT_TAB_BASE_ADDRESS | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#endif /* USER_VECT_TAB_ADDRESS */
  
  	// Enable periph  	
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	tmpio = RCC->APB2ENR & RCC_APB2ENR_SYSCFGEN;
	(void)tmpio;
	
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	tmpio = RCC->APB1ENR & RCC_APB1ENR_PWREN;
	(void)tmpio;
	
	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

}