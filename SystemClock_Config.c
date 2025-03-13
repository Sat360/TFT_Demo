#include "STM32F303_Clock.h"

uint32_t SystemCoreClock = 8000000;

const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};

#if !defined  (HSE_VALUE) 
  #define HSE_VALUE    ((uint32_t)8000000) /*!< Default value of the External oscillator in Hz.
                                                This value can be provided and adapted by the user application. */
#endif /* HSE_VALUE */

#if !defined  (HSI_VALUE)
  #define HSI_VALUE    ((uint32_t)8000000) /*!< Default value of the Internal oscillator in Hz.
                                                This value can be provided and adapted by the user application. */
#endif /* HSI_VALUE */

#define LL_FLASH_LATENCY_1                 FLASH_ACR_LATENCY_0     /*!< FLASH One Latency cycle */
__STATIC_INLINE void LL_FLASH_SetLatency(uint32_t Latency)
{
  MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, Latency);
}
__STATIC_INLINE uint32_t LL_FLASH_GetLatency(void)
{
  return (uint32_t)(READ_BIT(FLASH->ACR, FLASH_ACR_LATENCY));
}

__STATIC_INLINE void LL_RCC_HSE_EnableBypass(void)
{
  SET_BIT(RCC->CR, RCC_CR_HSEBYP);
}
__STATIC_INLINE void LL_RCC_HSE_Enable(void)
{
  SET_BIT(RCC->CR, RCC_CR_HSEON);
}
__STATIC_INLINE uint32_t LL_RCC_HSE_IsReady(void)
{
  return (READ_BIT(RCC->CR, RCC_CR_HSERDY) == (RCC_CR_HSERDY));
}
__STATIC_INLINE void LL_RCC_HSI_Enable(void)
{
  SET_BIT(RCC->CR, RCC_CR_HSION);
}
__STATIC_INLINE uint32_t LL_RCC_HSI_IsReady(void)
{
  return (READ_BIT(RCC->CR, RCC_CR_HSIRDY) == (RCC_CR_HSIRDY));
}
__STATIC_INLINE void LL_RCC_HSI_SetCalibTrimming(uint32_t Value)
{
  MODIFY_REG(RCC->CR, RCC_CR_HSITRIM, Value << RCC_CR_HSITRIM_Pos);
}
__STATIC_INLINE void LL_RCC_PLL_ConfigDomain_SYS(uint32_t Source, uint32_t PLLMul)
{
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL, (Source & RCC_CFGR_PLLSRC) | PLLMul);
  MODIFY_REG(RCC->CFGR2, RCC_CFGR2_PREDIV, (Source & RCC_CFGR2_PREDIV));
}
__STATIC_INLINE void LL_RCC_PLL_Enable(void)
{
  SET_BIT(RCC->CR, RCC_CR_PLLON);
}
__STATIC_INLINE uint32_t LL_RCC_PLL_IsReady(void)
{
  return (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == (RCC_CR_PLLRDY));
}
__STATIC_INLINE void LL_RCC_SetAHBPrescaler(uint32_t Prescaler)
{
  MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, Prescaler);
}
__STATIC_INLINE void LL_RCC_SetAPB1Prescaler(uint32_t Prescaler)
{
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, Prescaler);
}
__STATIC_INLINE void LL_RCC_SetAPB2Prescaler(uint32_t Prescaler)
{
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, Prescaler);
}
__STATIC_INLINE void LL_RCC_SetSysClkSource(uint32_t Source)
{
  MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, Source);
}
__STATIC_INLINE uint32_t LL_RCC_GetSysClkSource(void)
{
  return (uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_SWS));
}



#define LL_RCC_PLLSOURCE_HSE_DIV_1         (RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR2_PREDIV_DIV1)    /*!< HSE clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_2         (RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR2_PREDIV_DIV2)    /*!< HSE/2 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_3         (RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR2_PREDIV_DIV3)    /*!< HSE/3 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_4         (RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR2_PREDIV_DIV4)    /*!< HSE/4 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_5         (RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR2_PREDIV_DIV5)    /*!< HSE/5 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_6         (RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR2_PREDIV_DIV6)    /*!< HSE/6 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_7         (RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR2_PREDIV_DIV7)    /*!< HSE/7 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_8         (RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR2_PREDIV_DIV8)    /*!< HSE/8 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_9         (RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR2_PREDIV_DIV9)    /*!< HSE/9 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_10        (RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR2_PREDIV_DIV10)   /*!< HSE/10 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_11        (RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR2_PREDIV_DIV11)   /*!< HSE/11 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_12        (RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR2_PREDIV_DIV12)   /*!< HSE/12 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_13        (RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR2_PREDIV_DIV13)   /*!< HSE/13 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_14        (RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR2_PREDIV_DIV14)   /*!< HSE/14 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_15        (RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR2_PREDIV_DIV15)   /*!< HSE/15 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_16        (RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR2_PREDIV_DIV16)   /*!< HSE/16 clock selected as PLL entry clock source */

#define LL_RCC_PLL_MUL_2                   RCC_CFGR_PLLMUL2  /*!< PLL input clock*2 */
#define LL_RCC_PLL_MUL_3                   RCC_CFGR_PLLMUL3  /*!< PLL input clock*3 */
#define LL_RCC_PLL_MUL_4                   RCC_CFGR_PLLMUL4  /*!< PLL input clock*4 */
#define LL_RCC_PLL_MUL_5                   RCC_CFGR_PLLMUL5  /*!< PLL input clock*5 */
#define LL_RCC_PLL_MUL_6                   RCC_CFGR_PLLMUL6  /*!< PLL input clock*6 */
#define LL_RCC_PLL_MUL_7                   RCC_CFGR_PLLMUL7  /*!< PLL input clock*7 */
#define LL_RCC_PLL_MUL_8                   RCC_CFGR_PLLMUL8  /*!< PLL input clock*8 */
#define LL_RCC_PLL_MUL_9                   RCC_CFGR_PLLMUL9  /*!< PLL input clock*9 */
#define LL_RCC_PLL_MUL_10                  RCC_CFGR_PLLMUL10  /*!< PLL input clock*10 */
#define LL_RCC_PLL_MUL_11                  RCC_CFGR_PLLMUL11  /*!< PLL input clock*11 */
#define LL_RCC_PLL_MUL_12                  RCC_CFGR_PLLMUL12  /*!< PLL input clock*12 */
#define LL_RCC_PLL_MUL_13                  RCC_CFGR_PLLMUL13  /*!< PLL input clock*13 */
#define LL_RCC_PLL_MUL_14                  RCC_CFGR_PLLMUL14  /*!< PLL input clock*14 */
#define LL_RCC_PLL_MUL_15                  RCC_CFGR_PLLMUL15  /*!< PLL input clock*15 */
#define LL_RCC_PLL_MUL_16                  RCC_CFGR_PLLMUL16  /*!< PLL input clock*16 */

#define LL_RCC_SYSCLK_DIV_1                RCC_CFGR_HPRE_DIV1   /*!< SYSCLK not divided */
#define LL_RCC_SYSCLK_DIV_2                RCC_CFGR_HPRE_DIV2   /*!< SYSCLK divided by 2 */
#define LL_RCC_SYSCLK_DIV_4                RCC_CFGR_HPRE_DIV4   /*!< SYSCLK divided by 4 */
#define LL_RCC_SYSCLK_DIV_8                RCC_CFGR_HPRE_DIV8   /*!< SYSCLK divided by 8 */
#define LL_RCC_SYSCLK_DIV_16               RCC_CFGR_HPRE_DIV16  /*!< SYSCLK divided by 16 */
#define LL_RCC_SYSCLK_DIV_64               RCC_CFGR_HPRE_DIV64  /*!< SYSCLK divided by 64 */
#define LL_RCC_SYSCLK_DIV_128              RCC_CFGR_HPRE_DIV128 /*!< SYSCLK divided by 128 */
#define LL_RCC_SYSCLK_DIV_256              RCC_CFGR_HPRE_DIV256 /*!< SYSCLK divided by 256 */
#define LL_RCC_SYSCLK_DIV_512              RCC_CFGR_HPRE_DIV512 /*!< SYSCLK divided by 512 */

#define LL_RCC_APB1_DIV_1                  RCC_CFGR_PPRE1_DIV1  /*!< HCLK not divided */
#define LL_RCC_APB1_DIV_2                  RCC_CFGR_PPRE1_DIV2  /*!< HCLK divided by 2 */
#define LL_RCC_APB1_DIV_4                  RCC_CFGR_PPRE1_DIV4  /*!< HCLK divided by 4 */
#define LL_RCC_APB1_DIV_8                  RCC_CFGR_PPRE1_DIV8  /*!< HCLK divided by 8 */
#define LL_RCC_APB1_DIV_16                 RCC_CFGR_PPRE1_DIV16 /*!< HCLK divided by 16 */

#define LL_RCC_SYS_CLKSOURCE_HSI           RCC_CFGR_SW_HSI    /*!< HSI selection as system clock */
#define LL_RCC_SYS_CLKSOURCE_HSE           RCC_CFGR_SW_HSE    /*!< HSE selection as system clock */
#define LL_RCC_SYS_CLKSOURCE_PLL           RCC_CFGR_SW_PLL    /*!< PLL selection as system clock */

#define LL_RCC_SYS_CLKSOURCE_STATUS_HSI    RCC_CFGR_SWS_HSI   /*!< HSI used as system clock */
#define LL_RCC_SYS_CLKSOURCE_STATUS_HSE    RCC_CFGR_SWS_HSE   /*!< HSE used as system clock */
#define LL_RCC_SYS_CLKSOURCE_STATUS_PLL    RCC_CFGR_SWS_PLL   /*!< PLL used as system clock */

#define LL_RCC_I2C1_CLKSOURCE_HSI        (uint32_t)((RCC_CFGR3_I2C1SW << 24U) | RCC_CFGR3_I2C1SW_HSI)    /*!< HSI oscillator clock used as I2C1 clock source */


void LL_SetSystemCoreClock(uint32_t HCLKFrequency)
{
  /* HCLK clock frequency */
  SystemCoreClock = HCLKFrequency;
}

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
APB2 = 36 MHz max
USB (PLL VCO) = 48 MHz (strict)
HSE = 4 - 32 MHz (external crystall)
PLL input = 1 - 24 MHz
PLL output = 16 - 72 MHz
*/
/**
* Start system clock. Src: 0 - internal oscillator, 1 - external oscillator. Freq - needed frequency.
*/
void SystemClock_Config(uint8_t Src, uint8_t Freq)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_1)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(48000000);
  LL_SetSystemCoreClock(48000000);
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
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
}
