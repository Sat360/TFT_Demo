#ifndef __TFT_H__
#define __TFT_H__

#include "includes.h"
#include "fonts.h"
#include "GPIO.h"

void TFT_init(void);
void TFT_Idle(const uint8_t param);
void FillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, const uint16_t color);
void DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void DrawTriangleFill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color);
void WriteChar(char c, int16_t x, int16_t y, uint16_t color, uint16_t bgcolor);

void TP_Read(void);

#define TFT_WIDTH 240
#define TFT_HEIGHT 320
#define TFT_LIMITX(x) (((x) >= TFT_WIDTH) ? TFT_WIDTH - 1 : (x))
#define TFT_LIMITY(y) (((y) >= TFT_HEIGHT) ? TFT_HEIGHT - 1 : (y))
#define RGB5(r, g, b)	(uint16_t)((((uint16_t)r) << 11) | (((uint16_t)g) << 6) | ((uint16_t)b))	// 5-bit RGB
#define _ABS(x) ((x) > 0 ? (x) : (-x))

#define TP_ENA		// Enable touchpad
#define TFT_DMA		// Enable DMA for TFT
// DMA settings:
// RX (periph to mem): DMA1 ch4
// TX (mem to periph): DMA1 ch5
#define TFT_DMA_BUFF 1024
#define TFT_DR_8bit	*(__IO uint8_t*)&(TFT->DR)
#define TFT_DR_16bit	(TFT->DR)

struct sTFT
{
	uint8_t busy;
};

#ifdef TP_ENA
struct sTP
{
	uint16_t x;
	uint16_t y;
	uint8_t down;
};
#endif

// Define pins
#define TPIRQ_PORT 		GPIOD
#define TPIRQ_PIN 		11
#define TPCS_PORT 		GPIOD
#define TPCS_PIN 		10
#define TFTBL_PORT 		GPIOD
#define TFTBL_PIN 		12
#define TFTRESET_PORT 	GPIOD
#define TFTRESET_PIN 	9
#define TFTDC_PORT 		GPIOD
#define TFTDC_PIN 		8
#define TFTMOSI_PORT 	GPIOB
#define TFTMOSI_PIN 	15
#define TFTMISO_PORT 	GPIOB
#define TFTMISO_PIN 	14
#define TFTSCK_PORT 	GPIOB
#define TFTSCK_PIN 		13
#define TFTCS_PORT 		GPIOB
#define TFTCS_PIN 		12
#define TFT				SPI2

void TFT_init(void);
#define TFT_Sel() 	{Delayus(25);PortReset(TFTCS_PORT,TFTCS_PIN);} // Delay 19us minimum. Set 22us.
#define TFT_Free()	PortSet(TFTCS_PORT,TFTCS_PIN)
#define TP_Sel() 	{Delayus(25);PortReset(TPCS_PORT,TPCS_PIN);} // Delay 19us minimum. Set 22us.
#define TP_Free()	PortSet(TPCS_PORT,TPCS_PIN)
#define TFT_Wait()	{while(TFT->SR & SPI_SR_BSY){}}
#define TFT_Clear1(){(void)TFT->DR;(void)TFT->SR;}
#define TFT_Clear() {while(TFT->SR & SPI_SR_RXNE){(void)TFT->DR;}(void)TFT->SR;}
#define TFT_8bit()	{TFT->CR2 &= ~SPI_CR2_DS_3;}
#define TFT_16bit()	{TFT->CR2 |= SPI_CR2_DS_3;}

// ������� ������������ � ����� DMA � ������ �� ����.
// ������������ ����� � ����� ������� SendBuffer(tosend);
// ��������� ������������ ������ ������� ������ � ������� ������ ���� �����������
#define TFT_WRRAM_Start() {TFT_SendCmd(ST77XX_RAMWR, 0, 0); TFT_16bit(); TFT_Sel(); TFT->CR2 |= SPI_CR2_TXDMAEN;}
#define TFT_WRRAM_End() {TFT->CR2 &= ~SPI_CR2_TXDMAEN; TFT_Free(); TFT_8bit();}

// Commands. Default mode: DC = 0
// In this block all additional data writes in "DC=1"
#define ST77XX_NOP 0x00			// No-operation
#define ST77XX_SWRESET 0x01		// Software reset
#define ST77XX_RDDID 0x04		// Read display ID. 4 bytes(8b) to read in "DC=1": Dummy read, ID1 read, ID2 read, ID3 read
#define ST77XX_RDDST 0x09		// Read display status. 5 bytes(8b) to read DC=1: Dummy, b1, b2, b3, b4 (see page 156)
#define ST77XX_RDDPM 0x0A		// Read display power. 2 bytes to read: Dummy, b1 (see page 156)
#define ST77XX_RDD_MADCTL 0x0B	// Read display. 2 bytes to read: Dummy, b1 (MY, MX, MV, ML, RGB, MH, 0, 0)
#define ST77XX_RDD_COLMOD 0x0C	// Read display pixel. 2 bytes: Dummy, b1 (0, D6, D5, D4, 0, D2, D1, D0)
#define ST77XX_RDDIM 0x0D		// Read display image. 2 bytes: Dummy, b1 (VSSON, 0, INVON, 0, 0, GC2, GC1, GC0)
#define ST77XX_RDDSM 0x0E		// Read display signal. 2 bytes: Dummy, b1 (TEON, TEM, 0, 0, 0, 0, 0, 0)
#define ST77XX_RDDSDR 0x0F		// Read display self-diagnostic result. 2 bytes: Dummy, b1 (D7, D6, 0, 0, 0, 0, 0, 0)

#define ST77XX_SLPIN 0x10		// Sleep IN
#define ST77XX_SLPOUT 0x11		// Sleep OUT
#define ST77XX_PTLON 0x12		// Partial mode ON
#define ST77XX_NORON 0x13		// Partial mode OFF

// In this block all additional data writes in "DC=0"
#define ST77XX_INVOFF 0x20		// Inversion OFF
#define ST77XX_INVON 0x21		// Inversion ON
#define ST77XX_GAMSET 0x26		// Display inversion ON. 1 bytes to write: b2 (0, 0, 0, 0, GC3, GC2, GC1, GC0)
#define ST77XX_DISPOFF 0x28		// Display OFF
#define ST77XX_DISPON 0x29		// Display ON
#define ST77XX_CASET 0x2A		// X Address start. 4 bytes to write: Xstart(16b), Xend(16b). Start < End
#define ST77XX_RASET 0x2B		// Row address start. 4 bytes to write: Ystart(16b), Yend(16b). Start < End
#define ST77XX_RAMWR 0x2C		// Memmory write. 3 words(16b) to write: D1(16b), Dx(16b), Dn(16b)
#define ST77XX_RAMRD 0x2E		// Memory read. 3 words(16b) to read: Dummy(8b), D1(16b), Dx(16b), Dn(16b)

#define ST77XX_PTLAR 0x30		// Partial start/end address set. 4 bytes to write: Partial start addres (16b), Partial end address (16b)
#define ST77XX_VSCRDEF 0x33		// Vertial scroll definition. 6 bytes to write: TFA(16b), VSA(16b), BFA(16b)
#define ST77XX_TEOFF 0x34		// Tearing effect line OFF
#define ST77XX_TEON 0x35		// Tearing effect line ON. 1 byte to write: b1 (-,-,-,-,-,-,-,TEM)
#define ST77XX_MADCTL 0x36		// Memory data access control. 1 byte to write: b1 (MY, MX, MV, ML, RGB, 0, 0, 0)
#define ST77XX_VSCRSADD 0x37	// Vertical scrolling start address. 2 bytes to write: VSP(16b)
#define ST77XX_IDMOFF 0x38		// Idle mode OFF
#define ST77XX_IDMON 0x39		// Idle mode ON
#define ST77XX_COLMOD 0x3A		// Interface pixel format. 1 byte to write: b1 (0, D6, D5, D4, 0, D2, D1, D0) - interface format
#define ST77XX_RAMWRC 0x3C		// Memory write continue. 3 words to write: D1(16b), Dx(16b), Dn(16b)
#define ST77XX_RAMRDC 0x3E		// Memory read continue. 3 words to read: Dummy(8b), D1(16b), Dx(16b), Dn(16b)
#define ST77XX_TESCAN 0x44		// Set tear scanline. 2 write to write: N(16b)
#define ST77XX_RDTESCAN 0x45	// Get scanline
#define ST77XX_WRDISBV 0x51		// Write display brightness. 1 byte to write: DBV7(8b)
#define ST77XX_RDDISBV 0x52		// Read display brightness value
#define ST77XX_WRCTRLD 0x53		// Write CTRL display
#define ST77XX_RDCTRLD 0x54		// Read CTRL value display
#define ST77XX_WRCACE 0x55		// Write content adaptive brightness control and color anhancement
#define ST77XX_RDCABC 0x56		// Read ---
#define ST77XX_WRCABCMB 0x5E	// Write CABC minimum brightness
#define ST77XX_RDCABCB 0x5F		// Read ---
#define ST77XX_RDABCSDR 0x68	// Read automatic brightness control self-diagnostic result

#define ST77XX_MADCTL_MY 0x80
#define ST77XX_MADCTL_MX 0x40
#define ST77XX_MADCTL_MV 0x20
#define ST77XX_MADCTL_ML 0x10
#define ST77XX_MADCTL_RGB 0x00

#define ST77XX_SPI2EN 0xE7		// SPI2 Enable

#define ST77XX_RDID1 0xDA		// Read ID1. 1 word to read: Dummy(8b), ID(16b)
#define ST77XX_RDID2 0xDB
#define ST77XX_RDID3 0xDC
#define ST77XX_RDID4 0xDD 

#define TP_MEASCNT 16
#define TP_READX 0xD0
#define TP_READY 0x90
	
	
#endif // __TFT_H__