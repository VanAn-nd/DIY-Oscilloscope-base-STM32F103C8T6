
#include "main.h" 
#include "st7789.h"
#include <stdlib.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_dma.h"

#define ST7789_WIDTH  240
#define ST7789_HEIGHT 240
//#define UPSIDE_DOWN_LCD

#define DC_DAT     ST7789_DC_GPIO_Port ->BSRR |= (1<<11)      //HIGH
#define DC_CMD     ST7789_DC_GPIO_Port ->BSRR |= (1<<11)<<16         //LOW

#define RST_HIGH   ST7789_RST_GPIO_Port->BSRR |= (1<<12)     //HIGH
#define RST_LOW    ST7789_RST_GPIO_Port->BSRR |= (1<<12)<<16         //LOW

//#define CS_IDE     ST7789_CS_GPIO_Port ->BSRR |= (1<<15);      //HIGH
//#define CS_ACTIVE  ST7789_CS_GPIO_Port ->BSRR |= (1<<15)<<16         //LOW


#define ST_SWAP(a, b) {uint16_t temp; temp = a; a = b; b = temp;}
#define ABS(x) ((x) > 0 ? (x) : -(x))

static const uint8_t 
generic_st7789[] = {                // Init commands for 7789 screens
    9,                              //  9 commands in list:
    ST7789_SWRESET,   DELAY, //  1: Software reset, no args, w/delay
      150,                          //     ~150 ms delay
    ST7789_SLPOUT ,   DELAY, //  2: Out of sleep mode, no args, w/delay
      10,                          //      10 ms delay
    ST7789_COLMOD , 1+DELAY, //  3: Set color mode, 1 arg + delay:
      0x55,                         //     16-bit color
      10,                           //     10 ms delay
    ST7789_MADCTL , 1,              //  4: Mem access ctrl (directions), 1 arg:
      0x00,                         //     Row/col addr, bottom-top refresh 0x08/0x00
    ST7789_CASET  , 4,              //  5: Column addr set, 4 args, no delay:
      0x00,
      0,              //     XSTART = 0
      0,                                  
      240,            //     XEND = 240&0xFF
    ST7789_RASET  , 4,              //  6: Row addr set, 4 args, no delay:
      0x00,
      0,              //     YSTART = 0
      0,                                 
      240,            //     YEND = 240&0xFF
    ST7789_INVON  ,   DELAY,  //  7: hack
      10,
    ST7789_NORON  ,   DELAY, //  8: Normal display on, no args, w/delay
      10,                           //     10 ms delay
    ST7789_DISPON ,   DELAY, //  9: Main screen turn on, no args, delay
      10                           //    10 ms delay
};

static void ST7789_WriteSPI(uint8_t data) __attribute__((always_inline));
static void ST7789_WriteSPI(uint8_t data){
	
	#ifdef USE_SPI_DMA
	//HAL_SPI_Transmit_DMA(&ST7789_SPI_PORT, &data, 1);  //I dont know why it not work
	ST7789_SPI_DEVICE->DR = data; 
	uint8_t c=2; while(c--);           //trick
	while(ST7789_SPI_PORT.State == HAL_SPI_STATE_BUSY_TX);
  #else
	HAL_SPI_Transmit(&ST7789_SPI_PORT, &data, 1, HAL_MAX_DELAY);
  #endif
}
void __attribute__((weak)) st7789_WriteDMA(void *data, uint16_t length) {
	DMA1_Channel3->CCR =  (DMA_CCR_MINC | DMA_CCR_DIR); // Memory increment, direction to peripherial
	DMA1_Channel3->CMAR  = (uint32_t)data; // Source address
	DMA1_Channel3->CPAR  = (uint32_t)&SPI1->DR; // Destination address
	DMA1_Channel3->CNDTR = length;
	SPI1->CR1 &= ~(SPI_CR1_SPE);  // Disable SPI
	SPI1->CR2 |= SPI_CR2_TXDMAEN; // Enable DMA transfer
	SPI1->CR1 |= SPI_CR1_SPE;     // Enable SPI
	DMA1_Channel3->CCR |= DMA_CCR_EN;      // Start DMA transfer
	while(DMA1_Channel3->CNDTR);
}
static void ST7789_WriteCommand(uint8_t cmd) __attribute__((always_inline));
static void ST7789_WriteCommand(uint8_t cmd){
	DC_CMD;
	//ST7789_WriteSPI(cmd);
	HAL_SPI_Transmit(&ST7789_SPI_PORT, &cmd, 1, HAL_MAX_DELAY);
}
static void ST7789_WriteByteData(uint8_t data) __attribute__((always_inline));
static void ST7789_WriteByteData(uint8_t data){
	DC_DAT;
	ST7789_WriteSPI(data);
}

static void ST7789_WriteData(uint8_t* buff, size_t buff_size) __attribute__((always_inline));
static void ST7789_WriteData(uint8_t* buff, size_t buff_size){
	DC_DAT;
	#ifdef USE_SPI_DMA
	//HAL_SPI_Transmit(&ST7789_SPI_PORT, buff, buff_size, HAL_MAX_DELAY);
	//HAL_SPI_Transmit_DMA(&ST7789_SPI_PORT, buff, buff_size);
	while (buff_size--)
	{
		ST7789_SPI_DEVICE->DR = *buff++;
	  while(ST7789_SPI_PORT.State == HAL_SPI_STATE_BUSY_TX);
	}
  #else
	HAL_SPI_Transmit(&ST7789_SPI_PORT, buff, buff_size, HAL_MAX_DELAY);
  #endif
}

static void ST7789_ExecuteCommandList(const uint8_t *addr)
{
    uint8_t numCommands, numArgs;
    uint16_t ms;

    numCommands = *addr++;
    while(numCommands--)
    {
    	uint8_t cmd = *addr++;
        ST7789_WriteCommand(cmd);

        numArgs = *addr++;
        // If high bit set, delay follows args
        ms = numArgs & DELAY;
        numArgs &= ~DELAY;
        if(numArgs)
        {
						DC_DAT;
						HAL_SPI_Transmit(&ST7789_SPI_PORT, (uint8_t*)addr, numArgs, HAL_MAX_DELAY);
            //ST7789_WriteData((uint8_t*)addr, numArgs);
            addr += numArgs;
        }

        if(ms)
        {
            ms = *addr++;
            if(ms == 255) ms = 500;
					  LL_mDelay(ms);
            //HAL_Delay(ms);
        }
    }
}

static void ST7789_SetAddressWindow(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
  
    #ifdef USE_SPI_DMA
//    // column address set
//		uint8_t CASET[] = {(uint8_t)(x1 >> 8), (uint8_t)x1, (uint8_t)(x2 >> 8), (uint8_t)x2};
//    ST7789_WriteCommand(ST7789_CASET);
//    ST7789_WriteData(CASET, 4);
//    // row address set
//		uint8_t RASET[] = {(uint8_t)(y1 >> 8), (uint8_t)y1, (uint8_t)(y2 >> 8), (uint8_t)y2};
//    ST7789_WriteCommand(ST7789_RASET);
//    ST7789_WriteData(RASET, 4);
//    // write to RAM
//    ST7789_WriteCommand(ST7789_RAMWR);
	
	
//    ST7789_WriteCommand(ST7789_CASET);
//		DC_DAT;
//		ST7789_WriteSPI((uint8_t)(x1 >> 8));
//		ST7789_WriteSPI((uint8_t)x1);
//		ST7789_WriteSPI((uint8_t)(x2 >> 8));
//		ST7789_WriteSPI((uint8_t)x2);
//    // row address set
//    ST7789_WriteCommand(ST7789_RASET);
//    DC_DAT;
//		ST7789_WriteSPI((uint8_t)(y1 >> 8));
//		ST7789_WriteSPI((uint8_t)y1);
//		ST7789_WriteSPI((uint8_t)(y2 >> 8));
//		ST7789_WriteSPI((uint8_t)y2);
//    // write to RAM
//    ST7789_WriteCommand(ST7789_RAMWR);
		#ifdef USE_LOW_MODE
		uint8_t xa[4] = {0x00,(uint8_t)x1, 0x00,(uint8_t)x2};
		#ifdef UPSIDE_DOWN_LCD
		uint8_t ya[4] = {(y1+80)>>8,(uint8_t)(y1+80)&0xFF, (y2+80)>>8,(uint8_t)(y2+80)&0xFF}; //UPSIDE DOWN
		#else
		uint8_t ya[4] = {0x00,(uint8_t)y1, 0x00,(uint8_t)y2}; //UPSIDE DOWN
		#endif
		uint8_t cmd = ST7789_CASET;
		DC_CMD;
    HAL_SPI_Transmit(&ST7789_SPI_PORT, &cmd, 1, HAL_MAX_DELAY);
    DC_DAT;
    HAL_SPI_Transmit(&ST7789_SPI_PORT, xa, 4, HAL_MAX_DELAY);
		
		cmd = ST7789_RASET;
		DC_CMD;
		HAL_SPI_Transmit(&ST7789_SPI_PORT, &cmd, 1, HAL_MAX_DELAY);
    DC_DAT;
    HAL_SPI_Transmit(&ST7789_SPI_PORT, ya, 4, HAL_MAX_DELAY);
    
		cmd = ST7789_RAMWR;
		DC_CMD;
    HAL_SPI_Transmit(&ST7789_SPI_PORT, &cmd, 1, HAL_MAX_DELAY);
		#else
		ST7789_WriteCommand(ST7789_CASET);
		DC_DAT;
		ST7789_WriteSPI(0x00);
		ST7789_WriteSPI((uint8_t)x1);
		ST7789_WriteSPI(0x00);
		ST7789_WriteSPI((uint8_t)x2);
    // row address set
    ST7789_WriteCommand(ST7789_RASET);
    DC_DAT;
		ST7789_WriteSPI(0x00);
		ST7789_WriteSPI((uint8_t)y1);
		ST7789_WriteSPI(0x00);
		ST7789_WriteSPI((uint8_t)y2);
    // write to RAM
    ST7789_WriteCommand(ST7789_RAMWR);
		#endif

    #else

    uint8_t xa[4] = {0x00,(uint8_t)x1, 0x00,(uint8_t)x2};
    uint8_t ya[4] = {0x00,(uint8_t)y1, 0x00,(uint8_t)y2};
    ST7789_WriteCommand(ST7789_CASET);
    DC_DAT;
    HAL_SPI_Transmit(&ST7789_SPI_PORT, xa, 4, HAL_MAX_DELAY);
    ST7789_WriteCommand(ST7789_RASET);
    DC_DAT;
    HAL_SPI_Transmit(&ST7789_SPI_PORT, ya, 4, HAL_MAX_DELAY);
    // write to RAM
    ST7789_WriteCommand(ST7789_RAMWR);

    #endif
}

extern char* ltoa( long value, char *string, int radix )
{
  char tmp[33];
  char *tp = tmp;
  long i;
  unsigned long v;
  int sign;
  char *sp;

  if ( string == NULL )
  {
    return 0 ;
  }

  if (radix > 36 || radix <= 1)
  {
    return 0 ;
  }

  sign = (radix == 10 && value < 0);
  if (sign)
  {
    v = -value;
  }
  else
  {
    v = (unsigned long)value;
  }

  while (v || tp == tmp)
  {
    i = v % radix;
    v = v / radix;
    if (i < 10)
      *tp++ = i+'0';
    else
      *tp++ = i + 'a' - 10;
  }

  sp = string;

  if (sign)
    *sp++ = '-';
  while (tp > tmp)
    *sp++ = *--tp;
  *sp = 0;

  return string;
}

void ST7789_SetRotation(uint8_t m){
  ST7789_WriteCommand(ST7789_MADCTL); // MADCTL
  switch (m) {
  case 0:
    ST7789_WriteByteData(ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_RGB);
    break;
  case 1:
    ST7789_WriteByteData(ST7789_MADCTL_MY | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);
    break;
  case 2:
    ST7789_WriteByteData(ST7789_MADCTL_RGB);
    break;
  case 3:
    ST7789_WriteByteData(ST7789_MADCTL_MX | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);
    break;
  default:
    break;
  }
}

void ST7789_FillScreen(uint16_t color){
	uint16_t i, j;
	//CS_ACTIVE;
	ST7789_SetAddressWindow(0, 0, ST7789_WIDTH - 1, ST7789_HEIGHT - 1);
	for (i = 0; i < ST7789_WIDTH; i++)
		for (j = 0; j < ST7789_HEIGHT; j++) {
			uint8_t data[] = {color >> 8, color & 0xFF};
			ST7789_WriteData(data, sizeof(data));
		}
	//CS_IDE;
}

void ST7789_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
	#ifdef USE_LOW_MODE
    if ((x >= ST7789_WIDTH) || (y >= ST7789_HEIGHT))	return;
	#endif
    //CS_ACTIVE;
    ST7789_SetAddressWindow(x, y, x, y);
    uint8_t data[] = { color >> 8, color & 0xFF };
    ST7789_WriteData(data, 2);
    //CS_IDE;
}

void ST7789_DrawLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
	int16_t steep = abs(y2 - y1) > abs(x2 - x1);
	if (steep)
	{
		ST_SWAP(x1, y1);
		ST_SWAP(x2, y2);
	}

	if (x1 > x2)
	{
		ST_SWAP(x1, x2);
		ST_SWAP(y1, y2);
	}

	int16_t dx, dy;
	dx = x2 - x1;
	dy = abs(y2 - y1);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y1 < y2)
	{
		ystep = 1;
	}
	else
	{
		ystep = -1;
	}

	for (; x1<=x2; x1++)
	{
		if (steep)
		{
			//if (x1<240 && y1<240)           //osilo - binh thuong thi k can "if (x1<240)"
			ST7789_DrawPixel(y1, x1, color);
		}
		else
		{
			//if (x1<240 && y1<240)            //osilo - binh thuong thi k can "if (x1<240)"
			ST7789_DrawPixel(x1, y1, color);
		}
		err -= dy;
		if (err < 0)
		{
			y1 += ystep;
			err += dx;
		}
	}
}
void ST7789_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
	#ifdef USE_LOW_MODE
    // clipping
    if ((x >= ST7789_WIDTH) || (y >= ST7789_HEIGHT))
        return;
	#endif
    if ((x + w - 1) >= ST7789_WIDTH)
        w = ST7789_WIDTH - x;
    if ((y + h - 1) >= ST7789_HEIGHT)
        h = ST7789_HEIGHT - y;

    //CS_ACTIVE;
    ST7789_SetAddressWindow(x, y, x + w - 1, y + h - 1);

    uint8_t data[2] = { color >> 8, color & 0xFF };
    DC_DAT;

#ifdef USE_SPI_DMA
//    uint8_t tbuf[w*2];
//    for (y = h; y > 0; y--) {
//        for (int x = w * 2; x >= 0; x -= 2) {
//            tbuf[x] = data[0];
//            tbuf[x + 1] = data[1];
//        }
//        HAL_SPI_Transmit_DMA(&ST7789_SPI_PORT, tbuf, sizeof(tbuf));
//        while (hspi1.State == HAL_SPI_STATE_BUSY_TX) {
//        };
//    }
		for (y = h; y > 0; y--) {
        for (x = w; x > 0; x--) {
            ST7789_WriteData( data, sizeof(data));
        }
    }
#else
    for (y = h; y > 0; y--) {
        for (x = w; x > 0; x--) {
            HAL_SPI_Transmit(&ST7789_SPI_PORT, data, sizeof(data), HAL_MAX_DELAY);
        }
    }
#endif

    //CS_IDE;
}

void ST7789_DrawFastVLine(int16_t x, int16_t y1, int16_t y2, uint16_t color)
{
	#ifdef USE_SPI_DMA
  ST7789_SetAddressWindow(x, y1, x, y2);
	uint8_t data[2] = { color >> 8, color & 0xFF };
	for (int u=0; u<= y2-y1; u++) ST7789_WriteData( data, 2);
  #else
  ST7789_DrawLine(x, y1, x, y2, color);
  #endif
}
void ST7789_DrawFastHLine(int16_t x1, int16_t x2, int16_t y, uint16_t color)
{
	#ifdef USE_SPI_DMA
  ST7789_SetAddressWindow(x1, y, x2, y);
	uint8_t data[2] = { color >> 8, color & 0xFF };
	for (int u=0; u<= x2-x1; u++) ST7789_WriteData( data, 2);
  #else
  ST7789_DrawLine(x1, y, x2, y, color);
  #endif
}
void ST7789_DrawVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
	#ifdef USE_LOW_MODE
  if ((x >= ST7789_WIDTH) || (y >= ST7789_HEIGHT)) return;
	#endif
	
  if ((y + h - 1) >= ST7789_HEIGHT) h = ST7789_HEIGHT - y;
  #ifdef USE_SPI_DMA
  ST7789_FillRectangle(x, y, 1, h, color);
  #else
  ST7789_DrawLine(x, y, x, y + h - 1, color);
  #endif
}

void ST7789_DrawHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
	#ifdef USE_LOW_MODE
  if ((x >= ST7789_WIDTH) || (y >= ST7789_HEIGHT)) return;
	#endif
	
  if ((x + w - 1) >= ST7789_WIDTH)  w = ST7789_WIDTH - x;
  #ifdef USE_SPI_DMA
  ST7789_FillRectangle(x, y, w, 1, color);
  #else
  ST7789_DrawLine(x, y, x + w - 1, y, color);
  #endif
}

void ST7789_WriteChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor)
{
    uint32_t i, b, j;

    ST7789_SetAddressWindow(x, y, x+font.width-1, y+font.height-1);

    for(i = 0; i < font.height; i++)
    {
        b = font.data[(ch - 32) * font.height + i];
        for(j = 0; j < font.width; j++)
        {
            if((b << j) & 0x8000)
            {
                uint8_t data[] = { color >> 8, color & 0xFF };
                ST7789_WriteData(data, sizeof(data));
            }
            else
            {
                uint8_t data[] = { bgcolor >> 8, bgcolor & 0xFF };
                ST7789_WriteData(data, sizeof(data));
            }
        }
    }
}
void ST7789_WriteNumber(uint16_t x, uint16_t y, int num, uint16_t color, uint16_t bgcolor)
{
	if      (num==0) ST7789_WriteString(x, y, "0", Font_7x10, color, bgcolor);
	else if (num==1) ST7789_WriteString(x, y, "1", Font_7x10, color, bgcolor);
	else if (num==2) ST7789_WriteString(x, y, "2", Font_7x10, color, bgcolor);
	else if (num==3) ST7789_WriteString(x, y, "3", Font_7x10, color, bgcolor);
	else if (num==4) ST7789_WriteString(x, y, "4", Font_7x10, color, bgcolor);
	else if (num==5) ST7789_WriteString(x, y, "5", Font_7x10, color, bgcolor);
	else if (num==6) ST7789_WriteString(x, y, "6", Font_7x10, color, bgcolor);
	else if (num==7) ST7789_WriteString(x, y, "7", Font_7x10, color, bgcolor);
	else if (num==8) ST7789_WriteString(x, y, "8", Font_7x10, color, bgcolor);
	else if (num==9) ST7789_WriteString(x, y, "9", Font_7x10, color, bgcolor);
}
void ST7789_WriteInt(uint16_t x, uint16_t y, int num, uint16_t color, uint16_t bgcolor)
{
	int z=sizeof(num); int dec=1; for (;z>1;z--) dec*=10;
	while (dec>0)
	{
		ST7789_WriteNumber(x,y,num/dec,color,bgcolor);
		dec/=10;x+=7;
	}
}
void ST7789_WriteString(uint16_t x, uint16_t y, const char *str, FontDef font, uint16_t color, uint16_t bgcolor)
{
	//CS_ACTIVE;
	while (*str) {
		if (x + font.width >= ST7789_WIDTH) {
			x = 0;
			y += font.height;
			if (y + font.height >= ST7789_HEIGHT) {
				break;
			}

			if (*str == ' ') {
				// skip spaces in the beginning of the new line
				str++;
				continue;
			}
		}
		ST7789_WriteChar(x, y, *str, font, color, bgcolor);
		x += font.width;
		str++;
	}
	//CS_IDE;
}

void ST7789_DrawCircle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  //CS_ACTIVE;
  ST7789_DrawPixel(x0, y0 + r, color);
  ST7789_DrawPixel(x0, y0 - r, color);
  ST7789_DrawPixel(x0 + r, y0, color);
  ST7789_DrawPixel(x0 - r, y0, color);

  while (x < y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    ST7789_DrawPixel(x0 + x, y0 + y, color);
    ST7789_DrawPixel(x0 - x, y0 + y, color);
    ST7789_DrawPixel(x0 + x, y0 - y, color);
    ST7789_DrawPixel(x0 - x, y0 - y, color);

    ST7789_DrawPixel(x0 + y, y0 + x, color);
    ST7789_DrawPixel(x0 - y, y0 + x, color);
    ST7789_DrawPixel(x0 + y, y0 - x, color);
    ST7789_DrawPixel(x0 - y, y0 - x, color);
  }
  //CS_IDE;
}

void ST7789_DrawNumber(long long_num, int16_t x, int16_t y, FontDef font, uint16_t color, uint16_t bgcolor)
{
  char str[12];
  ltoa(long_num, str, 10);
  ST7789_WriteString(x, y, str, font, color, bgcolor);
}
void ST7789_DrawFloat(float floatNumber, int dp, int16_t x, int16_t y, FontDef font, uint16_t color, uint16_t bgcolor)
{
	char str[14];         // Array to contain decimal string
  uint8_t ptr = 0;      // Initialise pointer for array
  int8_t digits = 1;    // Count the digits to avoid array overflow
  float rounding = 0.5; // Round up down delta

  if (dp > 7)
    dp = 7; // Limit the size of decimal portion

  // Adjust the rounding value
  for (uint8_t i = 0; i < dp; ++i)
    rounding /= 10.0;

  if (floatNumber < -rounding) // add sign, avoid adding - sign to 0.0!
  {
    str[ptr++] = '-';           // Negative number
    str[ptr] = 0;               // Put a null in the array as a precaution
    digits = 0;                 // Set digits to 0 to compensate so pointer value can be used later
    floatNumber = -floatNumber; // Make positive
  }

  floatNumber += rounding; // Round up or down

  // For error put ... in string and return (all ST7789 library fonts contain . character)
  if (floatNumber >= 2147483647)
  {
    strcpy(str, "...");
		ST7789_WriteString(x, y, str, font, color, bgcolor);
  }
	else
	{
		// No chance of overflow from here on

  // Get integer part
  unsigned long temp = (unsigned long)floatNumber;

  // Put integer part into array
  ltoa(temp, str + ptr, 10);

  // Find out where the null is to get the digit count loaded
  while ((uint8_t)str[ptr] != 0)
    ptr++;       // Move the pointer along
  digits += ptr; // Count the digits

  str[ptr++] = '.'; // Add decimal point
  str[ptr] = '0';   // Add a dummy zero
  str[ptr + 1] = 0; // Add a null but don't increment pointer so it can be overwritten

  // Get the decimal portion
  floatNumber = floatNumber - temp;

  // Get decimal digits one by one and put in array
  // Limit digit count so we don't get a false sense of resolution
  uint8_t i = 0;
  while ((i < dp) && (digits < 9)) // while (i < dp) for no limit but array size must be increased
  {
    i++;
    floatNumber *= 10;  // for the next decimal
    temp = floatNumber; // get the decimal
    ltoa(temp, str + ptr, 10);
    ptr++;
    digits++;            // Increment pointer and digits count
    floatNumber -= temp; // Remove that digit
  }

  // Finally we can plot the string and return pixel length
	ST7789_WriteString(x, y, str, font, color, bgcolor);
	}
}

/* Basic functions. */
void ST7789_Init(void){
  LL_mDelay(100);
  DC_CMD;
  ST7789_WriteSPI(0x10);
  LL_mDelay(200);
  DC_DAT;
  //CS_ACTIVE;
  ST7789_WriteSPI(0x00);ST7789_WriteSPI(0x00);ST7789_WriteSPI(0x00);ST7789_WriteSPI(0x00);ST7789_WriteSPI(0x00);
  RST_LOW; LL_mDelay(100); RST_HIGH;  //Hard-reset
  ST7789_ExecuteCommandList(generic_st7789);
	#ifdef UPSIDE_DOWN_LCD
  ST7789_SetRotation(0); //UPSIDE DOWN 
	#else
	ST7789_SetRotation(2);
	#endif
	LL_mDelay(10);
  //CS_IDE;
}



