#ifndef __MAIN_H
#define __MAIN_H

// Распиновка дисплея:
// D12 - BackLight
// D11 - TP_IRQ
// D10 - TP_CS
// D9 - Reset
// D8 - DC
// B15 - SPI2_MOSI
// B14 - MISO
// B13 - SCK
// B12 - LCD_CS

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx.h"

#define DRDY_Pin LL_GPIO_PIN_2
#define DRDY_GPIO_Port GPIOE
#define CS_I2C_SPI_Pin LL_GPIO_PIN_3
#define CS_I2C_SPI_GPIO_Port GPIOE
#define MEMS_INT3_Pin LL_GPIO_PIN_4
#define MEMS_INT3_GPIO_Port GPIOE
#define MEMS_INT4_Pin LL_GPIO_PIN_5
#define MEMS_INT4_GPIO_Port GPIOE
#define OSC32_IN_Pin LL_GPIO_PIN_14
#define OSC32_IN_GPIO_Port GPIOC
#define OSC32_OUT_Pin LL_GPIO_PIN_15
#define OSC32_OUT_GPIO_Port GPIOC
#define OSC_IN_Pin LL_GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOF
#define OSC_OUT_Pin LL_GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOF
#define SPI1_SCK_Pin LL_GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin LL_GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MISOA7_Pin LL_GPIO_PIN_7
#define SPI1_MISOA7_GPIO_Port GPIOA
#define DM_Pin LL_GPIO_PIN_11
#define DM_GPIO_Port GPIOA
#define DP_Pin LL_GPIO_PIN_12
#define DP_GPIO_Port GPIOA
#define SWDIO_Pin LL_GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin LL_GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SWO_Pin LL_GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define I2C1_SCL_Pin LL_GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin LL_GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB
#define MEMS_INT1_Pin LL_GPIO_PIN_0
#define MEMS_INT1_GPIO_Port GPIOE
#define MEMS_INT2_Pin LL_GPIO_PIN_1
#define MEMS_INT2_GPIO_Port GPIOE

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
