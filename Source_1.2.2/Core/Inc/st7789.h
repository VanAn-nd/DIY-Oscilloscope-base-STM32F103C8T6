/* vim: set ai et ts=4 sw=4: */
#ifndef __ST7789_H__
#define __ST7789_H__


//#include <stdint.h>
#include "main.h"
#include "fonts.h"
#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_spi.h"
#include "stm32f1xx_hal_dma.h"


/*** Redefine if necessary ***/
#define USE_SPI_DMA			//if used DMA for SPI bus
#define USE_LOW_MODE
#define ST7789_SPI_DEVICE		    SPI1
#define ST7789_SPI_PORT         hspi1	            //hspi1, hspi2, hspi3...
extern  SPI_HandleTypeDef       ST7789_SPI_PORT;

#define ST7789_DC_Pin        	GPIO_PIN_11
#define ST7789_DC_GPIO_Port  	GPIOA
#define ST7789_RST_Pin       	GPIO_PIN_12
#define ST7789_RST_GPIO_Port 	GPIOA
#define ST7789_CS_Pin        	GPIO_PIN_15
#define ST7789_CS_GPIO_Port  	GPIOA


#define DELAY      0x80

#define ST7789_NOP     0x00
#define ST7789_SWRESET 0x01
#define ST7789_RDDID   0x04
#define ST7789_RDDST   0x09

#define ST7789_SLPIN   0x10
#define ST7789_SLPOUT  0x11
#define ST7789_PTLON   0x12
#define ST7789_NORON   0x13

#define ST7789_INVOFF  0x20
#define ST7789_INVON   0x21
#define ST7789_DISPOFF 0x28
#define ST7789_DISPON  0x29
#define ST7789_CASET   0x2A
#define ST7789_RASET   0x2B
#define ST7789_RAMWR   0x2C
#define ST7789_RAMRD   0x2E

#define ST7789_PTLAR   0x30
#define ST7789_TEOFF   0x34
#define ST7789_TEON    0x35
#define ST7789_MADCTL  0x36
#define ST7789_COLMOD  0x3A

#define ST7789_RDID1   0xDA
#define ST7789_RDID2   0xDB
#define ST7789_RDID3   0xDC
#define ST7789_RDID4   0xDD

#define ST7789_MADCTL_MY  0x80
#define ST7789_MADCTL_MX  0x40
#define ST7789_MADCTL_MV  0x20
#define ST7789_MADCTL_ML  0x10
#define ST7789_MADCTL_RGB 0x00

#define WHITE      0xFFFF
#define BLACK      0x0000
#define BLUE       0x001F
#define RED        0xF800
#define MAGENTA    0xF81F
#define GREEN      0x07E0
#define CYAN       0x7FFF
#define YELLOW     0xFFE0
#define GRAY       0X8430
#define BRED       0XF81F
#define GRED       0XFFE0
#define GBLUE      0X07FF
#define BROWN      0XBC40
#define BRRED      0XFC07
#define DARKBLUE   0X01CF
#define LIGHTBLUE  0X7D7C
#define GRAYBLUE   0X5458
#define LIGHTGREEN 0X841F
#define LGRAY      0XC618
#define LGRAYBLUE  0XA651
#define LBBLUE     0X2B12
#define TRACE      0X861
#define TRACE_CTR  0x3186
/*
// Online C compiler to run C program online: https://www.programiz.com/c-programming/online-compiler/

==============================================================================
#include <stdio.h>
int main() {
							//(((R  & 0xF8) << 8) | ((G  & 0xFC) << 3) | ((B  & 0xF8) >> 3));
    int color = (((50 & 0xF8) << 8) | ((50 & 0xFC) << 3) | ((50 & 0xF8) >> 3)); 
    printf("%d",color);
    return 0;
}
==============================================================================

*/


/* Spi functions. */


/* Basic functions. */
//static void ST7789_WriteData(uint8_t* buff, size_t buff_size);
extern char* ltoa( long value, char *string, int radix );
void ST7789_Init(void);
void ST7789_SetRotation(uint8_t m);
void ST7789_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ST7789_DrawLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void ST7789_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ST7789_FillScreen(uint16_t color);
void ST7789_DrawVLine(int16_t x, int16_t y, int16_t h, uint16_t color); 
void ST7789_DrawHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
void ST7789_DrawFastVLine(int16_t x, int16_t y1, int16_t y2, uint16_t color);
void ST7789_DrawFastHLine(int16_t x1, int16_t x2, int16_t y, uint16_t color);
void ST7789_WriteNumber(uint16_t x, uint16_t y, int num, uint16_t color, uint16_t bgcolor);
void ST7789_WriteInt(uint16_t x, uint16_t y, int num, uint16_t color, uint16_t bgcolor);
void ST7789_WriteChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor);
void ST7789_WriteString(uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor);
void ST7789_DrawCircle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color);
void ST7789_DrawNumber(long long_num, int16_t x, int16_t y, FontDef font, uint16_t color, uint16_t bgcolor);
void ST7789_DrawFloat(float floatNumber, int dp, int16_t x, int16_t y, FontDef font, uint16_t color, uint16_t bgcolor);


#endif // __ST7789_H__


