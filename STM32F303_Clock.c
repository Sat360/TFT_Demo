#include "includes.h"

uint32_t SystemCoreClock = 8000000;

uint32_t AhbClk = 48000000;
uint32_t Apb1Clk = 24000000;
uint32_t Apb2Clk = 48000000;
uint32_t Tim2Clk = 24000000;
uint32_t Tim15Clk = 48000000;
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};

/*
Ограничители:
Flash latency:
	0: HCLK 0 - 24 MHz (включительно)
	1: 24 - 48
	2: 48 - 78
HSI RC osc = 8 MHz (const)
LSI RC osc = 40 kHz (const)
AHB = 72 MHz max
APB2 = 72 MHz max
APB1 = 36 MHz max
USB (PLL VCO) = 48 MHz (strict)
HSE = 4 - 32 MHz (external crystall)
PLL input = 1 - 24 MHz
PLL output = 16 - 72 MHz
*/
/**
* Start system clock. Src: 0 - internal oscillator, 1 - external oscillator. Freq - needed frequency.
*/
void SystemClock_Config(const uint8_t Src, uint8_t Freq)
{
	if(Src > 1)
		Error_Handler();

	// Сброс генератора в начальное состояние
	// Select internal osc
	RCC->CFGR &= ~RCC_CFGR_PLLSRC;
	RCC->CFGR |= RCC_CFGR_PLLSRC_HSI_DIV2;
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);
	// Disable PLL
	RCC->CR &= ~RCC_CR_PLLON;
	// Reset to default values
	RCC->CFGR = 0;
	RCC->CFGR2 = 0;
	RCC->CFGR3 = 0;
	// Set flash latency to minimum
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= FLASH_ACR_LATENCY_1;
	while((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_1);
	
	// Change source oscillator
	if(Src == 1)
	{
		Error_Handler();
	}
	else
	{
		// Enable internal oscillator
		RCC->CR |= RCC_CR_HSION;
		while(!(RCC->CR & RCC_CR_HSIRDY_Msk));
	}
	
	// Set default calibration (16)
	RCC->CR &= ~RCC_CR_HSITRIM;
	RCC->CR |= RCC_CR_HSITRIM_4;
	
	// Calculate div
	uint32_t mul;
	if(Src == 1)
	{
		Error_Handler();
	}
	else
	{
		// Round freq to x4
		Freq &= ~0x3;
		// Limit to 8..64
		if(Freq < 16)
			Freq = 16;
		if(Freq > 64)
			Freq = 64;
		// freq = 12 is impossible
		if(Freq == 12)
			Freq = 16;
		mul = Freq >> 2;
		
		// Config domain
		RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL);
		RCC->CFGR |= (mul - 2) << RCC_CFGR_PLLMUL_Pos;
	}
	// Enable PLL
	if(Freq >= 16)
	{
		RCC->CR |= RCC_CR_PLLON;
		while(!(RCC->CR & RCC_CR_PLLRDY_Msk));
	}
	
	// Set AHB = PLL
	RCC->CFGR &= ~RCC_CFGR_HPRE;
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	
	// Set APB1 = if(PLL <= 36) PLL; else PLL/2
	RCC->CFGR &= ~RCC_CFGR_PPRE1;
	if(Freq > 36)
	{
		RCC->CFGR |= RCC_CFGR_PPRE1_2;
	}
	
	// Set APB2 = PLL
	RCC->CFGR &= ~RCC_CFGR_PPRE2;
	RCC->CFGR |= RCC_CFGR_PPRE2_2;
	
	// Set src
	RCC->CFGR &= ~RCC_CFGR_SW;
	if(Freq < 16)
	{
		// Direct to int osc
		RCC->CFGR |= RCC_CFGR_SW_HSI;
		while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);
	}
	{
		// PLL
		RCC->CFGR |= RCC_CFGR_SW_PLL;
		while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
	}	
	
	SystemCoreClock = Freq * 1000000;
}

void SystemCoreClockUpdate (void)
{
  uint32_t tmp = 0, pllmull = 0, pllsource = 0, predivfactor = 0;

  /* Get SYSCLK source -------------------------------------------------------*/
  tmp = RCC->CFGR & RCC_CFGR_SWS;

  switch (tmp)
  {
    case RCC_CFGR_SWS_HSI:  /* HSI used as system clock */
      SystemCoreClock = HSI_VALUE;
      break;
    case RCC_CFGR_SWS_HSE:  /* HSE used as system clock */
      SystemCoreClock = HSE_VALUE;
      break;
    case RCC_CFGR_SWS_PLL:  /* PLL used as system clock */
      /* Get PLL clock source and multiplication factor ----------------------*/
      pllmull = RCC->CFGR & RCC_CFGR_PLLMUL;
      pllsource = RCC->CFGR & RCC_CFGR_PLLSRC;
      pllmull = ( pllmull >> 18) + 2;

#if defined (STM32F302xE) || defined (STM32F303xE) || defined (STM32F398xx)
        predivfactor = (RCC->CFGR2 & RCC_CFGR2_PREDIV) + 1;
      if (pllsource == RCC_CFGR_PLLSRC_HSE_PREDIV)
      {
        /* HSE oscillator clock selected as PREDIV1 clock entry */
        SystemCoreClock = (HSE_VALUE / predivfactor) * pllmull;
      }
      else
      {
        /* HSI oscillator clock selected as PREDIV1 clock entry */
        SystemCoreClock = (HSI_VALUE / predivfactor) * pllmull;
      }
#else      
      if (pllsource == RCC_CFGR_PLLSRC_HSI_DIV2)
      {
        /* HSI oscillator clock divided by 2 selected as PLL clock entry */
        SystemCoreClock = (HSI_VALUE >> 1) * pllmull;
      }
      else
      {
        predivfactor = (RCC->CFGR2 & RCC_CFGR2_PREDIV) + 1;
        /* HSE oscillator clock selected as PREDIV1 clock entry */
        SystemCoreClock = (HSE_VALUE / predivfactor) * pllmull;
      }
#endif /* STM32F302xE || STM32F303xE || STM32F398xx */
      break;
    default: /* HSI used as system clock */
      SystemCoreClock = HSI_VALUE;
      break;
  }
  /* Compute HCLK clock frequency ----------------*/
  /* Get HCLK prescaler */
  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
  /* HCLK clock frequency */
  SystemCoreClock >>= tmp;
  
  AhbClk = SystemCoreClock;
  
  tmp = (RCC->CFGR & RCC_CFGR_PPRE1_Msk) >> RCC_CFGR_PPRE1_Pos;
  Apb1Clk = SystemCoreClock >> APBPrescTable[tmp];
  Tim2Clk = (tmp == 0 ? Apb1Clk : (Apb1Clk * 2));
  
  tmp = (RCC->CFGR & RCC_CFGR_PPRE2_Msk) >> RCC_CFGR_PPRE2_Pos;
  Apb2Clk = SystemCoreClock >> APBPrescTable[tmp];
  Tim15Clk = (tmp == 0 ? Apb2Clk : (Apb2Clk * 2));
}
