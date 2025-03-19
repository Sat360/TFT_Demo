#include "TFT.h"

#define TFT_CMD_CNT 9
volatile uint8_t TFT_Params[6];
volatile uint16_t TFT_TX[TFT_DMA_BUFF];

volatile struct sTFT display;

void ST77XX_init(void);
void TFT_SendByte(uint8_t data);
void TFT_SendCmd(uint8_t cmd, uint8_t params, uint8_t delay);
void SetOutRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void SendBuffer(uint16_t count);

// ������ �������������, � ������� ����� ����������.
// ���������� - ������������, �� ���� � ����� x2,y2 ���� ����� ���� ���������
void SetOutRect(const uint16_t x1, const uint16_t y1, const uint16_t x2, const uint16_t y2)
{
	// must be x1 <= x2, y1 <= y2
	TFT_Params[0] = x1 >> 8;
	TFT_Params[1] = x1 & 0xFF;
	TFT_Params[2] = x2 >> 8;
	TFT_Params[3] = x2 & 0xFF;
	TFT_SendCmd(ST77XX_CASET, 4, 0);

	TFT_Params[0] = y1 >> 8;
	TFT_Params[1] = y1 & 0xFF;
	TFT_Params[2] = y2 >> 8;
	TFT_Params[3] = y2 & 0xFF;
	TFT_SendCmd(ST77XX_RASET, 4, 0);
}

// ��������� � DMA ����� ��������� �������
void SendBuffer(const uint16_t count)
{
	DMA1_Channel5->CMAR = (uint32_t)&TFT_TX;
	DMA1_Channel5->CNDTR = count;
	display.busy = 1;
	DMA1_Channel5->CCR |= DMA_CCR_EN;
	while(display.busy) {_wdr()};
	DMA1_Channel5->CCR &= ~DMA_CCR_EN;
}

// ������� �������������
void FillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	// Normalize order x1/x2 and y1/y2
	uint16_t dummy;
	if(x2 < x1)
	{
		dummy = x1;
		x1 = x2;
		x2 = dummy;
	}
	if(y2 < y1)
	{
		dummy = y1;
		y1 = y2;
		y2 = dummy;
	}

	SetOutRect(x1, y1, x2, y2);

	// ������ ������, ��������, � ��������� �������
	uint32_t DrawLeft = (x2 - x1 + 1) * (y2 - y1 + 1);
	// Init buffer
	if(DrawLeft <= TFT_DMA_BUFF)
	{
		for(uint16_t i = 0; i < DrawLeft; i++)
			TFT_TX[i] = color;
	}
	else
	{
		for(uint16_t i = 0; i < TFT_DMA_BUFF; i++)
			TFT_TX[i] = color;
	}
	// Send data
	TFT_WRRAM_Start();
	while(DrawLeft > 0)
	{
		// Calc size to send
		uint16_t tosend = DrawLeft;
		if(tosend > TFT_DMA_BUFF)
			tosend = TFT_DMA_BUFF;
		SendBuffer(tosend);
		DrawLeft -= tosend;
	}
	TFT_WRRAM_End();

}

void test(void)
{
	FillRect(10, 10, 40, 40, RGB5(31, 0, 0));
	FillRect(30, 30, 180, 180, RGB5(0, 31, 0));
	FillRect(10, 150, 50, 230, RGB5(0, 0, 31));
	
	TFT_SendCmd(ST77XX_IDMON, 0, 0);
}

void TFT_init(void)
{
	uint16_t temp_var;
	
	display.busy = 0;

	// Init GPIO
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIODEN; 
	temp_var = RCC->AHBENR & (RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIODEN); 
	(void)temp_var;
	#ifdef TFT_DMA
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	temp_var = RCC->AHBENR & RCC_AHBENR_DMA1EN;
	(void)temp_var;
	#endif
	
	#ifdef TP_ENA
	InitPin(TPIRQ_PORT, TPIRQ_PIN, GPIO_MODE_INPUT, GPIO_TYPE_PP, GPIO_SPEED_LOW, GPIO_PUPD_NONE);
	InitPin(TPCS_PORT, TPCS_PIN, GPIO_MODE_OUTPUT, GPIO_TYPE_PP, GPIO_SPEED_HIGH, GPIO_PUPD_NONE);
	#endif // TP_ENA
	InitPin(TFTBL_PORT, TFTBL_PIN, GPIO_MODE_OUTPUT, GPIO_TYPE_PP, GPIO_SPEED_LOW, GPIO_PUPD_NONE);
	InitPin(TFTRESET_PORT, TFTRESET_PIN, GPIO_MODE_OUTPUT, GPIO_TYPE_PP, GPIO_SPEED_LOW, GPIO_PUPD_NONE);
	InitPin(TFTDC_PORT, TFTDC_PIN, GPIO_MODE_OUTPUT, GPIO_TYPE_PP, GPIO_SPEED_LOW, GPIO_PUPD_NONE);
	InitPin(TFTMOSI_PORT, TFTMOSI_PIN, GPIO_MODE_ALT, GPIO_TYPE_PP, GPIO_SPEED_HIGH, GPIO_PUPD_NONE);
	//InitPin(TFTMISO_PORT, TFTMISO_PIN, GPIO_MODE_ALT, GPIO_TYPE_PP, GPIO_SPEED_HIGH, GPIO_PUPD_PU);
	InitPin(TFTSCK_PORT, TFTSCK_PIN, GPIO_MODE_ALT, GPIO_TYPE_PP, GPIO_SPEED_HIGH, GPIO_PUPD_NONE);
	InitPin(TFTCS_PORT, TFTCS_PIN, GPIO_MODE_OUTPUT, GPIO_TYPE_PP, GPIO_SPEED_LOW, GPIO_PUPD_NONE);
	SetAltPin(TFTMOSI_PORT, TFTMOSI_PIN, 5);
	//SetAltPin(TFTMISO_PORT, TFTMISO_PIN, 5);
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
	// BR[2:0] = baudrate. AHB / 2^(BR+1). Maximum freq = 18 MHz.
	// Maximum display freq = 16ns = 62.5 MHz
	// MSTR = master configuration
	// CPOL, CPHA = polarity and phase
	TFT->CR1 = (SPI_CR1_SSM*1) | (SPI_CR1_SSI*1) | (SPI_CR1_BR_2*0) | (SPI_CR1_BR_1*0) | (SPI_CR1_BR_0*1) | SPI_CR1_MSTR | (SPI_CR1_CPOL*1) | (SPI_CR1_CPHA*1);
	// FRXTH - RXNE event on 8-bit receive
	// DS_X - bit size
	// TXEIE - TX empty interrupt enable
	// TXDMAEN/RXDMAEN - enable DMA buffers
	TFT_Clear();
	TFT->CR2 = ((0*SPI_CR2_FRXTH) | SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2);
	#ifdef TFT_DMA	
	TFT->CR2 |= (0*SPI_CR2_TXEIE) | (0*SPI_CR2_TXDMAEN);	
	DMA1->IFCR = 0xF << DMA_IFCR_CGIF5_Pos; // Clear DMA Interrupt flags
	// PL - priority. 0/1/2/3 = low, med, high, vhigh
	// MSIZE - memory size: 0/1/2 = 8/16/32 bit
	// PSIZE - --==--
	// MINC  - memory increment enable
	// PINC = periph increment enable
	// DIR = direction (1 = from mem to periph)
	// TCIE = transfer complete interrupt enable
	// EN = channel enable
	DMA1_Channel5->CCR = (0*DMA_CCR_PL_0) | (1*DMA_CCR_PL_1) | (1*DMA_CCR_MSIZE_0) | (0*DMA_CCR_MSIZE_1) | (1*DMA_CCR_PSIZE_0) | (0*DMA_CCR_PSIZE_1) | DMA_CCR_MINC | (1*DMA_CCR_DIR) | DMA_CCR_TCIE;
	DMA1_Channel5->CPAR = (uint32_t)&(TFT->DR);
	NVIC_SetPriority(DMA1_Channel5_IRQn, TFT_DMA_Prio);	//set IRQ priority	
	NVIC_EnableIRQ(DMA1_Channel5_IRQn);		//enable timer6 int.
	#endif
	
	// Clear bits
	TFT_8bit();
	TFT_Clear();
	TFT->SR = 0;
	// Enable
	TFT->CR1 |= SPI_CR1_SPE;

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

void TFT_SendByte(const uint8_t data)
{
	TFT_Sel();
	TFT_DR_8bit = data;
	TFT_Wait();
	TFT_Clear();
	TFT_Free();
}

// Send command with 0 params
void TFT_SendCmd(const uint8_t cmd, const uint8_t params, const uint8_t delay)
{
	PortReset(TFTDC_PORT, TFTDC_PIN);
	TFT_Sel();
	TFT_Clear();

	TFT_Wait();
	TFT_DR_8bit = cmd;
	TFT_Wait();
	TFT_Clear();
	PortSet(TFTDC_PORT, TFTDC_PIN);
	
	for(uint8_t i = 0; i < params; i++)
	{
		TFT_DR_8bit = TFT_Params[i];
		TFT_Wait();
		TFT_Clear1();
	}
	TFT_Free();	
	
	if(delay)
		Delay(delay);
}

void ST77XX_init(void)
{
	// Software reset
	TFT_SendCmd(ST77XX_SWRESET, 0, 150);
	
	// Out of sleep mode
	TFT_SendCmd(ST77XX_SLPOUT, 0, 10);
	
	// Set color mode.
	// page 224
	// LO = control interface color format
	// HI = RGB interface color format
	// 3 = 12bit (RGB 4-4-4)
	// 5 = 16 bit (RGB 5-6-5)
	// 6 = 18 bit (RGB 6-6-6)
	// 7 = 24 bit truncated
	TFT_Params[0] = 0x55; // 16-bit color
	TFT_SendCmd(ST77XX_COLMOD, 1, 0);
	
	// Memory access control (directions).
	// Page 215, 124
	// Rotate -90 = MV+MY
	// Rotate +90 = MV+MX
	// Rotate 180 = MX+MY
	// 7 MY = Page address order. 0 = top to bottom, 1 = bottom to top (flip Y)
	// 6 MX = Column address order. 0 = left to right, 1 = right to left (flip X)
	// 5 MV = Page/Column order. 0 = normal mode, 1 = reverse mode (rotate -90 and flip X)
	// 4 ML = Line address order. 0 = LCD refresh top to bottom, 1 = refresh bottom to top
	// 3 RGB = RGB/BGR Order. 0 = RGB, 1 = BGR
	// 2 MH = Display data latch order. 0 = LCD refresh left to right, 1 = refresh right to left
	// 01 = reserved
	TFT_Params[0] = 0x00; // 
	TFT_SendCmd(ST77XX_MADCTL, 1, 0);
	
	// Hack (??)
	TFT_SendCmd(ST77XX_INVON, 0, 0);
	
	// Normal display ON
	TFT_SendCmd(ST77XX_NORON, 0, 0);
	
	// Main display turn ON
	TFT_SendCmd(ST77XX_DISPON, 0, 0);
	
	// Enable SPI2
	//TFT_Params[0] = 0x11; // Enable SPI2, Enable Read
	//TFT_SendCmd(ST77XX_SPI2EN, 1, 0);
}

void DMA1_Channel5_IRQHandler()
{
	DMA1->IFCR = DMA_IFCR_CGIF5 | DMA_IFCR_CTCIF5 | DMA_IFCR_CHTIF5 | DMA_IFCR_CTEIF5;
	display.busy = 0;
}