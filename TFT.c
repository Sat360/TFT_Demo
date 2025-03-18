#include "TFT.h"

#define TFT_CMD_CNT 9
uint8_t TFT_Params[6];

void ST77XX_init(void);
void SPI2_SendByte(uint8_t data);
void SPI2_SendCmd(uint8_t cmd, uint8_t params, uint8_t delay);

/*
static const uint8_t PROGMEM
  generic_st7789[] =  {                // Init commands for 7789 screens
    9,                              //  9 commands in list:
    ST77XX_SWRESET,   ST_CMD_DELAY, //  1: Software reset, no args, w/delay
      150,                          //     ~150 ms delay
    ST77XX_SLPOUT ,   ST_CMD_DELAY, //  2: Out of sleep mode, no args, w/delay
      10,                          //      10 ms delay
    ST77XX_COLMOD , 1+ST_CMD_DELAY, //  3: Set color mode, 1 arg + delay:
      0x55,                         //     16-bit color
      10,                           //     10 ms delay
    ST77XX_MADCTL , 1,              //  4: Mem access ctrl (directions), 1 arg:
      0x08,                         //     Row/col addr, bottom-top refresh
    ST77XX_CASET  , 4,              //  5: Column addr set, 4 args, no delay:
      0x00,
      0,        //     XSTART = 0
      0,
      240,  //     XEND = 240
    ST77XX_RASET  , 4,              //  6: Row addr set, 4 args, no delay:
      0x00,
      0,             //     YSTART = 0
      320>>8,
      320&0xFF,  //     YEND = 320
    ST77XX_INVON  ,   ST_CMD_DELAY,  //  7: hack
      10,
    ST77XX_NORON  ,   ST_CMD_DELAY, //  8: Normal display on, no args, w/delay
      10,                           //     10 ms delay
    ST77XX_DISPON ,   ST_CMD_DELAY, //  9: Main screen turn on, no args, delay
      10 };  
*/
/*
void Adafruit_ST77xx::displayInit(const uint8_t *addr) {

  uint8_t numCommands, cmd, numArgs;
  uint16_t ms;

  numCommands = pgm_read_byte(addr++); // Number of commands to follow
  while (numCommands--) {              // For each command...
    cmd = pgm_read_byte(addr++);       // Read command
    numArgs = pgm_read_byte(addr++);   // Number of args to follow
    ms = numArgs & ST_CMD_DELAY;       // If hibit set, delay follows args
    numArgs &= ~ST_CMD_DELAY;          // Mask out delay bit
    sendCommand(cmd, addr, numArgs);
    addr += numArgs;

    if (ms) {
      ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      if (ms == 255)
        ms = 500; // If 255, delay for 500 ms
      delay(ms);
    }
  }
}
*/

void test()
{
	//
	TFT_Params[0] = 0;
	TFT_Params[1] = 0;
	TFT_Params[2] = 100 >> 8;
	TFT_Params[3] = 100 & 0xFF;
	SPI2_SendCmd(ST77XX_CASET, 4, 0);

	TFT_Params[0] = 0;
	TFT_Params[1] = 0;
	TFT_Params[2] = 100 >> 8;
	TFT_Params[3] = 100 & 0xFF;
	SPI2_SendCmd(ST77XX_RASET, 4, 0);
	
	SPI2_SendCmd(ST77XX_RAMWR, 0, 0);
	
	TFT_Sel();
	for(uint16_t i = 0; i < 20000; i++)
		SPI2_SendByte(i);
	TFT_Free();
	
	//
	/*
	TFT_Params[0] = 0;
	TFT_Params[1] = 50;
	TFT_Params[2] = 210 >> 8;
	TFT_Params[3] = 210 & 0xFF;
	SPI2_SendCmd(ST77XX_CASET, 4, 0);

	TFT_Params[0] = 0;
	TFT_Params[1] = 50;
	TFT_Params[2] = 210 >> 8;
	TFT_Params[3] = 210 & 0xFF;
	SPI2_SendCmd(ST77XX_RASET, 4, 0);
	
	SPI2_SendCmd(ST77XX_RAMWR, 0, 0);
	
	TFT_Sel();
	for(uint16_t i = 0; i < 2*160*160; i++)
		SPI2_SendByte(i);
	TFT_Free();
	*/

	/*
	TFT_Params[0] = 0;
	TFT_Params[1] = 50;
	TFT_Params[2] = 210 >> 8;
	TFT_Params[3] = 210 & 0xFF;
	SPI2_SendCmd(ST77XX_CASET, 4, 0);

	TFT_Params[0] = 0;
	TFT_Params[1] = 50;
	TFT_Params[2] = 210 >> 8;
	TFT_Params[3] = 210 & 0xFF;
	SPI2_SendCmd(ST77XX_RASET, 4, 0);

	TFT_Params[0] = 0xFF;
	TFT_Params[1] = 0xFF;
	TFT_Params[2] = 0xFF;
	TFT_Params[3] = 0xFF;
	TFT_Params[4] = 0xFF;
	TFT_Params[5] = 0xFF;
	SPI2_SendCmd(ST77XX_RAMRD, 6, 0);
	*/
}

void TFT_init(void)
{
	uint16_t temp_var;

	// Init GPIO
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIODEN; 
	temp_var = RCC->AHBENR & (RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIODEN); 
	(void)temp_var;
	
	#ifdef TP_ENA
	InitPin(TPIRQ_PORT, TPIRQ_PIN, GPIO_MODE_INPUT, GPIO_TYPE_PP, GPIO_SPEED_LOW, GPIO_PUPD_NONE);
	InitPin(TPCS_PORT, TPCS_PIN, GPIO_MODE_OUTPUT, GPIO_TYPE_PP, GPIO_SPEED_HIGH, GPIO_PUPD_NONE);
	#endif // TP_ENA
	InitPin(TFTBL_PORT, TFTBL_PIN, GPIO_MODE_OUTPUT, GPIO_TYPE_PP, GPIO_SPEED_LOW, GPIO_PUPD_NONE);
	InitPin(TFTRESET_PORT, TFTRESET_PIN, GPIO_MODE_OUTPUT, GPIO_TYPE_PP, GPIO_SPEED_LOW, GPIO_PUPD_NONE);
	InitPin(TFTDC_PORT, TFTDC_PIN, GPIO_MODE_OUTPUT, GPIO_TYPE_PP, GPIO_SPEED_LOW, GPIO_PUPD_NONE);
	InitPin(TFTMOSI_PORT, TFTMOSI_PIN, GPIO_MODE_ALT, GPIO_TYPE_PP, GPIO_SPEED_HIGH, GPIO_PUPD_NONE);
	InitPin(TFTMISO_PORT, TFTMISO_PIN, GPIO_MODE_ALT, GPIO_TYPE_PP, GPIO_SPEED_HIGH, GPIO_PUPD_PU);
	InitPin(TFTSCK_PORT, TFTSCK_PIN, GPIO_MODE_ALT, GPIO_TYPE_PP, GPIO_SPEED_HIGH, GPIO_PUPD_NONE);
	InitPin(TFTCS_PORT, TFTCS_PIN, GPIO_MODE_OUTPUT, GPIO_TYPE_PP, GPIO_SPEED_LOW, GPIO_PUPD_NONE);
	SetAltPin(TFTMOSI_PORT, TFTMOSI_PIN, 5);
	SetAltPin(TFTMISO_PORT, TFTMISO_PIN, 5);
	SetAltPin(TFTSCK_PORT, TFTSCK_PIN, 5);
	TFT_Free();
	
	// Init SPI
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
	temp_var = RCC->APB1ENR & RCC_APB1ENR_SPI2EN;
	(void)temp_var;
	
	
	// full duplex, MSB first, CLK / 8, Master, CPOL = 1, CPHA = 1
	// CR1 = common config
	// CR2 = DMA config
	// SSM = software slave management
	// SSI = internal slave select (only with SSM)
	// BR[2:0] = baudrate. AHB / 2^(BR+1)
	// MSTR = master configuration
	// CPOL, CPHA = polarity and phase
	SPI2->CR1 = (SPI_CR1_SSM*1) | (SPI_CR1_SSI*1) | (SPI_CR1_BR_2*0) | (SPI_CR1_BR_1*1) | (SPI_CR1_BR_0*0) | SPI_CR1_MSTR | (SPI_CR1_CPOL*1) | (SPI_CR1_CPHA*1);
	// FRXTH - RXNE event on 8-bit receive
	// DS_X - bit size
	// TXEIE - TX empty interrupt enable
	// TXDMAEN/RXDMAEN - enable DMA buffers
	TFT_Clear();
	//SPI2->CR2 = (SPI_CR2_FRXTH | SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2);
	#ifdef TFT_DMA	
	SPI2->CR2 |= SPI_CR2_TXEIE | SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
	#endif
	
	// Clear bits
	TFT_8bit();
	TFT_Clear();
	SPI2->SR = 0;
	// Enable
	SPI2->CR1 |= SPI_CR1_SPE;

	// Reset TFT
	Delay(25);
	PortReset(TFTRESET_PORT, TFTRESET_PIN);
	Delay(50);
	PortSet(TFTRESET_PORT, TFTRESET_PIN);
	Delay(50);
	
	// Enable backlight
	PortSet(TFTBL_PORT, TFTBL_PIN);
	
	// Init
	ST77XX_init();
	Delay(50);
	
	test();
}

void SPI2_SendByte(uint8_t data)
{
	TFT_Sel();
	SPI2_DR_8bit = data;
	TFT_Wait();
	TFT_Clear();
	TFT_Free();
}

// Send command with 0 params
void SPI2_SendCmd(uint8_t cmd, uint8_t params, uint8_t delay)
{
	uint16_t d;
	PortReset(TFTDC_PORT, TFTDC_PIN);
	TFT_Sel();
	TFT_Clear();

	TFT_Wait();
	SPI2_DR_8bit = cmd;
	TFT_Wait();
	TFT_Clear();
	PortSet(TFTDC_PORT, TFTDC_PIN);
	
	for(uint8_t i = 0; i < params; i++)
	{
		SPI2_DR_8bit = TFT_Params[i];
		TFT_Wait();
		//TFT_Clear1();
		d = SPI2->DR;
		TFT_Clear();
		TFT_Params[i] = d;
	}
	TFT_Free();	
	
	if(delay)
		Delay(delay);
}

void ST77XX_init(void)
{
	// Software reset
	SPI2_SendCmd(ST77XX_SWRESET, 0, 150);
	
	// Out of sleep mode
	SPI2_SendCmd(ST77XX_SLPOUT, 0, 10);
	
	//SPI2_SendCmd(ST77XX_RDDID, 4, 0);
	
	// Set color mode.
	TFT_Params[0] = 0x55; // 16-bit color
	SPI2_SendCmd(ST77XX_COLMOD, 1, 10);
	
	// Memory access control (directions). Row/col addr, botto-top refresh
	TFT_Params[0] = 0x08; // 
	SPI2_SendCmd(ST77XX_MADCTL, 1, 0);
	
	// Column address set
	TFT_Params[0] = 0;
	TFT_Params[1] = 0;
	TFT_Params[2] = 240 >> 8;
	TFT_Params[3] = 240 & 0xFF;
	SPI2_SendCmd(ST77XX_CASET, 4, 0);
	
	// Row address set
	TFT_Params[0] = 0;
	TFT_Params[1] = 0;
	TFT_Params[2] = 320 >> 8;
	TFT_Params[3] = 320 & 0xFF;
	SPI2_SendCmd(ST77XX_RASET, 4, 0);
	
	// Hack
	SPI2_SendCmd(ST77XX_INVON, 0, 10);
	
	// Normal display ON
	SPI2_SendCmd(ST77XX_NORON, 0, 10);
	
	// Main display turn ON
	SPI2_SendCmd(ST77XX_DISPON, 0, 10);
	
	// Enable SPI2
	//TFT_Params[0] = 0x11; // Enable SPI2, Enable Read
	//SPI2_SendCmd(ST77XX_SPI2EN, 1, 0);
}
