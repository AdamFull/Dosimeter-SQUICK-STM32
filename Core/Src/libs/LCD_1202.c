#include "libs/LCD_1202.h"
#include "libs/GFX_font.h"
#include "stdlib.h"
#include "string.h"
#include "util.h"

uint16_t _LCD_RAM[LCD_X*LCD_String]; // Память нашего LCD

static LCD_CONFIG lcd_config;

uint16_t textsize = 1, textcolor = 1, textbgcolor = 0;
uint16_t cursor_x = 0, cursor_y = 0;

uint16_t getCommand(uint16_t cmd, uint16_t arg, uint16_t mask){
	return (cmd | (arg & mask));
}

uint16_t getComCmd(uint16_t arg, uint16_t mask){
	return (arg & mask);
}

const char *toString(uint32_t value, uint16_t size){
	//static char *buf = malloc(sizeof(char) * size);
	//return buf;
}

// Отправляем байт данных дисплею
void LCD_SendByte(uint16_t mode, uint16_t c)
{
  // Опускаем ножку CS для дисплея
	//LL_GPIO_ResetOutputPin(GPIOx, PinMask);
	//LL_GPIO_SetOutputPin(GPIOx, PinMask);
	LL_GPIO_ResetOutputPin(lcd_config.CSPORT, lcd_config.CSPIN);
  // Формируем первый передаваемый бит - выбор память-команда
	if (mode) {
		LL_GPIO_SetOutputPin(lcd_config.MOSIPORT, lcd_config.MOSIPIN);
	}
	else {
		LL_GPIO_ResetOutputPin(lcd_config.MOSIPORT, lcd_config.MOSIPIN);
	};
	// Проталкиваем тактовым импульсом
	LL_GPIO_SetOutputPin(lcd_config.SCKPORT, lcd_config.SCKPIN);
	// В цикле передаем остальные биты
	for(uint16_t i=0; i<8; i++) {
    // Сбрасываем тактовую ножку
		LL_GPIO_ResetOutputPin(lcd_config.SCKPORT, lcd_config.SCKPIN);
    // Выставляем бит данных
		if (c & 0x80) {
			LL_GPIO_SetOutputPin(lcd_config.MOSIPORT, lcd_config.MOSIPIN);
		}
		else {
			LL_GPIO_ResetOutputPin(lcd_config.MOSIPORT, lcd_config.MOSIPIN);
		};
		// Тактом ее, тактом
		LL_GPIO_SetOutputPin(lcd_config.SCKPORT, lcd_config.SCKPIN);
		// Сдвигаем данные
    c <<= 1;
	};
	LL_GPIO_ResetOutputPin(lcd_config.SCKPORT, lcd_config.SCKPIN);
}

// Очистка памяти дисплея
void LCD_Clear(void) {
  for (int index = 0; index < 864 ; index++){
    _LCD_RAM[index] = (0x00);
  }
}

// Обновляем данные на экране
void LCD_Update(void) {
  for(uint16_t p = 0; p < 9; p++) {
    LCD_SendByte(LCD_C, SetYAddr | p);
    LCD_SendByte(LCD_C, SetXAddr4);
    LCD_SendByte(LCD_C, SetXAddr3);
    for(uint16_t col=0; col < LCD_X; col++){
      LCD_SendByte(LCD_D, _LCD_RAM[(LCD_X * p) + col]);
    }
  }
}

// Рисование пикселя по координатам и цвету
void LCD_DrawPixel (uint16_t x, uint16_t y, uint16_t color) {
  if ((x < 0) || (x >= LCD_X) || (y < 0) || (y >= LCD_Y)) return;

  if (color) _LCD_RAM[x+ (y/8)*LCD_X] |= 1<<(y%8);
  else       _LCD_RAM[x+ (y/8)*LCD_X] &= ~(1<<(y%8));
}

// Рисование линии
void LCD_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color) {
  int steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    swap(x0, y0);
    swap(x1, y1);
  }
  if (x0 > x1) {
    swap(x0, x1);
    swap(y0, y1);
  }
  int dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);
  int err = dx / 2;
  int ystep;
  if (y0 < y1) {ystep = 1;}
  else {ystep = -1;};
  for ( ; x0 <= x1; x0++) {
    if (steep) {LCD_DrawPixel(y0, x0, color);}
    else {LCD_DrawPixel(x0, y0, color);};
		err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

// Рисование вертикальной линии
void LCD_DrawFastVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color) {
  LCD_DrawLine(x, y, x, y+h-1, color);
}

// Рисование горизонтальной линии
void LCD_DrawFastHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color) {
  LCD_DrawLine(x, y, x+w-1, y, color);
}

// Рисование прямоугольника
void LCD_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
  LCD_DrawFastHLine(x, y, w, color);
  LCD_DrawFastHLine(x, y+h-1, w, color);
  LCD_DrawFastVLine(x, y, h, color);
  LCD_DrawFastVLine(x+w-1, y, h, color);
}

// Рисование залитый прямоугольник
void LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
  for (int16_t i=x; i<x+w; i++) {
    LCD_DrawFastVLine(i, y, h, color);
  }
}

// Заливка экрана
void LCD_FillScreen(uint16_t color) {
  LCD_FillRect(0, 0, LCD_X, LCD_Y, color);
}

void LCD_JustDrawChar(unsigned char c){
	LCD_DrawChar(cursor_x, cursor_y, textcolor, textbgcolor, textsize, c);
	cursor_x=cursor_x+textsize*5;;
}

// Нарисовать букву
void LCD_DrawChar(uint16_t x, uint16_t y, uint16_t color, uint16_t bg, uint16_t size, unsigned char c) {
	if((x >= LCD_X) ||(y >= LCD_Y) || ((x + 4) < 0) || ((y + 7) < 0)) return;
	if(c<128)            c = c-32;
	if(c>=144 && c<=175) c = c-48;
	if(c>=128 && c<=143) c = c+16;
	if(c>=176 && c<=191) c = c-48;
	if(c>191)  return;
	for (uint16_t i=0; i<6; i++ ) {
		uint16_t line;
		if (i == 5) {line = 0x00;}
		else {
			line = font[(c*5)+i];
			for (uint16_t j = 0; j<8; j++)
			{
				if (line & 0x01) {
					if(size == 1){
						LCD_DrawPixel(x+i, y+j, color);
					}else{
						LCD_FillRect(x+(i*size), y+(j*size), size, size, color);
					}
				}else if(bg != color){
					if(size == 1){
						LCD_DrawPixel(x+i, y+j, bg);
					}else{
						LCD_FillRect(x+(i*size), y+(j*size), size, size, bg);
					}
				}
				//cursor_x += textsize*6;
				line >>= 1;
			}
		}
	}
}

// Вывод строки
void LCD_print(const char *str) {
  unsigned char type = *str;
  if(type>=128) cursor_x=cursor_x-3;
  while(*str){
    LCD_DrawChar(cursor_x, cursor_y, textcolor, textbgcolor, textsize, *str++);
    unsigned char type = *str;
    if (type>=128) {cursor_x=cursor_x+textsize*3;}
    else {cursor_x=cursor_x+textsize*5;};
  }
}

// Вывод числовых значений
void LCD_write(float num, bool as_float){
  char c[32];
  if(!as_float){
	  sprintf(c, "%d", (int)num);
  }else{
	  ftoa(num, c, 2);
  }
  LCD_print(c);
}

// Вывод картинки
void LCD_DrawBitmap(uint16_t x, uint16_t y, const char *bitmap, uint16_t w, uint16_t h, uint16_t color) {
  for (int16_t j=0; j<h; j++) {
    for (int16_t i=0; i<w; i++ ) {
      if (bitmap[i + (j/8)*w] & 1<<(j%8)) { LCD_DrawPixel(x+i, y+j, color); }
    }
  }
}

void LCD_SetContrast(uint16_t value){
	if (value > 31)  value = 31;
	LCD_SendByte(LCD_C, (STE2007_CMD_ELECTVOL | (value & STE2007_MASK_ELECTVOL)));
}

void LCD_PowerOn(bool state){
	uint16_t power_byte = state ? 0x01 : 0x00;
	LCD_SendByte(LCD_C, getCommand(STE2007_CMD_ONOFF, power_byte, STE2007_MASK_ONOFF));
}

void LCD_PowerSave(bool state){
	uint16_t power_byte = state ? 0x01 : 0x00;
	LCD_SendByte(LCD_C, getCommand(STE2007_CMD_DPYALLPTS, power_byte, STE2007_MASK_DPYALLPTS));
}

void LCD_Flip(){
	LCD_SendByte(LCD_C, getCommand(STE2007_CMD_SEGMENTDIR, 1, STE2007_MASK_SEGMENTDIR));	//Display on
	LCD_SendByte(LCD_C, getCommand(STE2007_CMD_COMDIR, 8, STE2007_MASK_COMDIR));
}

void LCD_SetInvert(bool value){
	uint16_t invert_byte = value ? 0x01 : 0x00;
	LCD_SendByte(LCD_C, getCommand(STE2007_CMD_DPYREV, invert_byte, STE2007_MASK_DPYREV));
}

void LCD_SetCharSize(uint16_t size){
	textsize = (size > 0) ? size : 1;
}

void LCD_SetTextColor(uint16_t c, uint16_t b){
	textcolor   = c;
	textbgcolor = b;
}

void LCD_SetCursor(uint16_t x, uint16_t y){
	cursor_x = x;
	cursor_y = y;
}

void LCD_AddToCursor(uint16_t x, uint16_t y){
	cursor_x += x;
	cursor_y += y;
}

// Инициализируем дисплей
void LCD_Init(LCD_CONFIG config) {
  // Инициализация дисплея
	lcd_config = config;
	LL_GPIO_ResetOutputPin(lcd_config.RESPORT, lcd_config.RESPIN); // Активируем ресет
	LL_mDelay(5);

	LL_GPIO_SetOutputPin(lcd_config.RESPORT, lcd_config.RESPIN);   // Деактивируем ресет
	LL_GPIO_ResetOutputPin(lcd_config.SCKPORT, lcd_config.SCKPIN);     // Тактовый вывод низкое состояние
	LL_GPIO_ResetOutputPin(lcd_config.MOSIPORT, lcd_config.MOSIPIN);   // Вывод данных в низкое состояние
	LL_GPIO_ResetOutputPin(lcd_config.CSPORT, lcd_config.CSPIN);			 // Выбираем дисплей
	// Задержка
	LL_mDelay(5);
	LL_GPIO_SetOutputPin(lcd_config.CSPORT, lcd_config.CSPIN);			 // Выбираем дисплей

	LCD_SendByte(LCD_C, getCommand(STE2007_CMD_RESET, 0, STE2007_MASK_RESET));  // Reset chip
	LL_mDelay(5);

	LCD_SendByte(LCD_C, getCommand(STE2007_CMD_DPYALLPTS, 0, STE2007_MASK_DPYALLPTS)); // Powersave ALLPOINTS-ON mode turned OFF
	LCD_SendByte(LCD_C, getCommand(STE2007_CMD_PWRCTL, 7, STE2007_MASK_PWRCTL));		// Power control set to max

	LCD_SendByte(LCD_C, getCommand(STE2007_CMD_ONOFF, 1, STE2007_MASK_ONOFF));	//Display on
 	LCD_SendByte(LCD_C, getCommand(STE2007_CMD_COMDIR, 0, STE2007_MASK_COMDIR));		// Display common driver = NORMAL

 	LCD_SendByte(LCD_C, getCommand(STE2007_CMD_SEGMENTDIR, 0, STE2007_MASK_SEGMENTDIR));	// Lines start at the left

	LCD_SendByte(LCD_C, getCommand(STE2007_CMD_ELECTVOL, lcd_config.lcd_contrast, STE2007_MASK_ELECTVOL));	// Electronic volume set to user specified contrast

	LCD_Clear();

	LCD_SendByte(LCD_C, STE2007_CMD_REFRESHRATE);
	LCD_SendByte(LCD_C, getComCmd(3, STE2007_MASK_REFRESHRATE));	// Refresh rate = 65Hz
	LCD_SendByte(LCD_C, STE2007_CMD_CHARGEPUMP);
	LCD_SendByte(LCD_C, getComCmd(0, STE2007_MASK_CHARGEPUMP));	// Charge Pump multiply factor = 5x
	LCD_SendByte(LCD_C, getCommand(STE2007_CMD_SETBIAS, 6, STE2007_MASK_SETBIAS)); // Bias ratio = 1/4
	LCD_SendByte(LCD_C, STE2007_CMD_VOP);
	LCD_SendByte(LCD_C, getComCmd(0, STE2007_MASK_VOP));
	LCD_SendByte(LCD_C, getCommand(STE2007_CMD_DPYREV, 0, STE2007_MASK_DPYREV));	// Display normal (not inverted)
	LL_mDelay(10);

	LCD_Update();
}
