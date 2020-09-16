#ifndef __LCD
#define __LCD

/*Library author Vladimir Medintsev
 * Rewrited, and upgraded by AdamFull
 * 9.09.2020
*/

#include "stm32f1xx_hal.h"
#include "stdbool.h"

#define COLOR_BLACK 1
#define COLOR_WHITE 0

#define STE2007_CMD_ONOFF 0xAE
#define STE2007_MASK_ONOFF 0x01

#define STE2007_CMD_DPYREV 0xA6
#define STE2007_MASK_DPYREV 0x01

#define STE2007_CMD_DPYALLPTS 0xA4
#define STE2007_MASK_DPYALLPTS 0x01

#define STE2007_CMD_LINE 0xB0
#define STE2007_MASK_LINE 0x0F

#define STE2007_CMD_COLMSB 0x10
#define STE2007_MASK_COLMSB 0x07

#define STE2007_CMD_COLLSB 0x00
#define STE2007_MASK_COLLSB 0x0F

#define STE2007_CMD_DPYSTARTLINE 0x40
#define STE2007_MASK_DPYSTARTLINE 0x3F

#define STE2007_CMD_SEGMENTDIR 0xA0
#define STE2007_MASK_SEGMENTDIR 0x01

#define STE2007_CMD_COMDIR 0xC0
#define STE2007_MASK_COMDIR 0x08

#define STE2007_CMD_PWRCTL 0x28
#define STE2007_MASK_PWRCTL 0x07

#define STE2007_CMD_VORANGE 0x40
#define STE2007_MASK_VORANGE 0x07

#define STE2007_CMD_ELECTVOL 0x80
#define STE2007_MASK_ELECTVOL 0x1F

#define STE2007_CMD_RESET 0xE2
#define STE2007_MASK_RESET 0x00

#define STE2007_CMD_NOP 0xE3
#define STE2007_MASK_NOP 0x00

/* Compound commands--sent by sending the command first, then value as a 2nd 9-bit byte. */
// VOP is set by submitting CMD_VOP, then the value as the next 9-bit byte.
#define STE2007_CMD_VOP 0xE1
#define STE2007_MASK_VOP 0xFF

// VLCD Slope is similar; submit CMD_VLCDSLOPE, then the value in the next 9-bit byte.
#define STE2007_CMD_VLCDSLOPE 0x38
#define STE2007_MASK_VLCDSLOPE 0x07

// Charge Pump similar.
#define STE2007_CMD_CHARGEPUMP 0x3D
#define STE2007_MASK_CHARGEPUMP 0x03

// Refresh Rate similar.
#define STE2007_CMD_REFRESHRATE 0xEF
#define STE2007_MASK_REFRESHRATE 0x03

// Bias ratio is a normal CMD|DATA operation. (not a compound command)
#define STE2007_CMD_SETBIAS 0x30
#define STE2007_MASK_SETBIAS 0x07

// N-Line inversion is a compound command (send CMD_NLINEINV, then value)
#define STE2007_CMD_NLINEINV 0xAD
#define STE2007_MASK_NLINEINV 0x1F

// Number of Lines is a normal CMD|DATA operation. (not a compound command)
#define STE2007_CMD_NUMLINES 0xD0
#define STE2007_MASK_NUMLINES 0x07

// Image Location is a compound command (send CMD_IMAGELOC, then value)
#define STE2007_CMD_IMAGELOC 0xAC
#define STE2007_MASK_IMAGELOC 0x07

// Icon Mode is a normal CMD|DATA operation. (not a compound command)
#define STE2007_CMD_ICONMODE 0xF8
#define STE2007_MASK_ICONMODE 0x01

#define LCD_X        96
#define LCD_Y        68
#define LCD_String    9
#define LCD_D         1		// Данные
#define LCD_C         0   // Команда
#define SetYAddr   0xB0
#define SetXAddr4  0x00
#define SetXAddr3  0x10

#define swap(a, b) {uint8_t t = a; a = b; b = t; }

typedef struct {
	GPIO_TypeDef *MOSIPORT;
	uint16_t      MOSIPIN;

	GPIO_TypeDef *SCKPORT;
	uint16_t      SCKPIN;

	GPIO_TypeDef *RESPORT;
	uint16_t      RESPIN;

	GPIO_TypeDef *CSPORT;
	uint16_t      CSPIN;

	uint8_t lcd_contrast;
} LCD_CONFIG;

void LCD_SendByte(uint8_t mode, uint8_t c);
void LCD_Clear(void);
void LCD_Update(void);
void LCD_Init(LCD_CONFIG config);
void LCD_SetContrast(uint8_t value);
void LCD_SetPowerSave(bool value);
void LCD_Flip();
void LCD_SetPower(bool value);
void LCD_SetInvert(bool value);
void LCD_SetCharSize(uint8_t size);
void LCD_SetTextColor(uint8_t c, uint8_t b);
void LCD_SetCursor(uint8_t x, uint8_t y);
void LCD_AddToCursor(uint8_t x, uint8_t y);

void LCD_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void LCD_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color);
void LCD_DrawFastVLine(uint8_t x, uint8_t y, uint8_t h, uint8_t color);
void LCD_DrawFastHLine(uint8_t x, uint8_t y, uint8_t w, uint8_t color);
void LCD_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
void LCD_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
void LCD_FillScreen(uint8_t color);
void LCD_DrawChar(uint8_t x, uint8_t y, uint8_t color, uint8_t bg, uint8_t size, unsigned char c);
void LCD_print(const char *str);
void LCD_write(float num, bool as_float);
void LCD_DrawBitmap(uint8_t x, uint8_t y, const char *bitmap, uint8_t w, uint8_t h, uint8_t color);

const char *toString(uint32_t value, uint16_t size);


#endif
