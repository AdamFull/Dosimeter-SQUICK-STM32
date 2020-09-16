#include "libs/LCD_1202.h"
#include "libs/GFX_font.h"
#include "stdlib.h"
#include "string.h"
#include "util.h"

uint8_t _LCD_RAM[LCD_X*LCD_String]; // Память нашего LCD

static LCD_CONFIG lcd_config;

uint8_t textsize = 1, textcolor = 1, textbgcolor = 0;
uint8_t cursor_x = 0, cursor_y = 0;

uint8_t getCommand(uint8_t cmd, uint8_t arg, uint8_t mask){
	return (cmd | (arg & mask));
}

uint8_t getComCmd(uint8_t arg, uint8_t mask){
	return (arg & mask);
}

const char *toString(uint32_t value, uint16_t size){
	char *buf = malloc(sizeof(char) * size);
}

// Отправляем байт данных дисплею
void LCD_SendByte(uint8_t mode, uint8_t c)
{
  // Опускаем ножку CS для дисплея
	HAL_GPIO_WritePin(lcd_config.CSPORT, lcd_config.CSPIN, GPIO_PIN_RESET);
  // Формируем первый передаваемый бит - выбор память-команда
	if (mode) {
		HAL_GPIO_WritePin(lcd_config.MOSIPORT, lcd_config.MOSIPIN, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(lcd_config.MOSIPORT, lcd_config.MOSIPIN, GPIO_PIN_RESET);
	};
	// Проталкиваем тактовым импульсом
	HAL_GPIO_WritePin(lcd_config.SCKPORT, lcd_config.SCKPIN, GPIO_PIN_SET);
	// В цикле передаем остальные биты
	for(uint8_t i=0; i<8; i++) {
    // Сбрасываем тактовую ножку
    HAL_GPIO_WritePin(lcd_config.SCKPORT, lcd_config.SCKPIN, GPIO_PIN_RESET);
    // Выставляем бит данных
		if (c & 0x80) {
			HAL_GPIO_WritePin(lcd_config.MOSIPORT, lcd_config.MOSIPIN, GPIO_PIN_SET);
		}
		else {
			HAL_GPIO_WritePin(lcd_config.MOSIPORT, lcd_config.MOSIPIN, GPIO_PIN_RESET);
		};
		// Тактом ее, тактом
		HAL_GPIO_WritePin(lcd_config.SCKPORT, lcd_config.SCKPIN, GPIO_PIN_SET);
		// Сдвигаем данные
    c <<= 1;
	};
	HAL_GPIO_WritePin(lcd_config.SCKPORT, lcd_config.SCKPIN, GPIO_PIN_RESET);
}

// Очистка памяти дисплея
void LCD_Clear(void) {
  for (int index = 0; index < 864 ; index++){
    _LCD_RAM[index] = (0x00);
  }
}

// Обновляем данные на экране
void LCD_Update(void) {
  for(uint8_t p = 0; p < 9; p++) {
    LCD_SendByte(LCD_C, SetYAddr | p);
    LCD_SendByte(LCD_C, SetXAddr4);
    LCD_SendByte(LCD_C, SetXAddr3);
    for(uint8_t col=0; col < LCD_X; col++){
      LCD_SendByte(LCD_D, _LCD_RAM[(LCD_X * p) + col]);
    }
  }
}

// Рисование пикселя по координатам и цвету
void LCD_DrawPixel (uint8_t x, uint8_t y, uint8_t color) {
  if ((x < 0) || (x >= LCD_X) || (y < 0) || (y >= LCD_Y)) return;

  if (color) _LCD_RAM[x+ (y/8)*LCD_X] |= 1<<(y%8);
  else       _LCD_RAM[x+ (y/8)*LCD_X] &= ~(1<<(y%8));
}

// Рисование линии
void LCD_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color) {
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
void LCD_DrawFastVLine(uint8_t x, uint8_t y, uint8_t h, uint8_t color) {
  LCD_DrawLine(x, y, x, y+h-1, color);
}

// Рисование горизонтальной линии
void LCD_DrawFastHLine(uint8_t x, uint8_t y, uint8_t w, uint8_t color) {
  LCD_DrawLine(x, y, x+w-1, y, color);
}

// Рисование прямоугольника
void LCD_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color) {
  LCD_DrawFastHLine(x, y, w, color);
  LCD_DrawFastHLine(x, y+h-1, w, color);
  LCD_DrawFastVLine(x, y, h, color);
  LCD_DrawFastVLine(x+w-1, y, h, color);
}

// Рисование залитый прямоугольник
void LCD_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color) {
  for (int16_t i=x; i<x+w; i++) {
    LCD_DrawFastVLine(i, y, h, color);
  }
}

// Заливка экрана
void LCD_FillScreen(uint8_t color) {
  LCD_FillRect(0, 0, LCD_X, LCD_Y, color);
}

// Нарисовать букву
void LCD_DrawChar(uint8_t x, uint8_t y, uint8_t color, uint8_t bg, uint8_t size, unsigned char c) {
	if((x >= LCD_X) ||(y >= LCD_Y) || ((x + 4) < 0) || ((y + 7) < 0)) return;
	if(c<128)            c = c-32;
	if(c>=144 && c<=175) c = c-48;
	if(c>=128 && c<=143) c = c+16;
	if(c>=176 && c<=191) c = c-48;
	if(c>191)  return;
	for (uint8_t i=0; i<6; i++ ) {
		uint8_t line;
		if (i == 5) {line = 0x00;}
		else {
			line = font[(c*5)+i];
			for (uint8_t j = 0; j<8; j++)
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
void LCD_DrawBitmap(uint8_t x, uint8_t y, const char *bitmap, uint8_t w, uint8_t h, uint8_t color) {
  for (int16_t j=0; j<h; j++) {
    for (int16_t i=0; i<w; i++ ) {
      if (bitmap[i + (j/8)*w] & 1<<(j%8)) { LCD_DrawPixel(x+i, y+j, color); }
    }
  }
}

void LCD_SetContrast(uint8_t value){
	if (value > 31)  value = 31;
	LCD_SendByte(LCD_C, (STE2007_CMD_ELECTVOL | (value & STE2007_MASK_ELECTVOL)));
}

void LCD_SetPowerSave(bool value){
	uint8_t power_byte = value ? 0x01 : 0x00;
	LCD_SendByte(LCD_C, getCommand(STE2007_CMD_DPYALLPTS, power_byte, STE2007_MASK_DPYALLPTS));
}

void LCD_Flip(){
	LCD_SendByte(LCD_C, getCommand(STE2007_CMD_SEGMENTDIR, 1, STE2007_MASK_SEGMENTDIR));	//Display on
	LCD_SendByte(LCD_C, getCommand(STE2007_CMD_COMDIR, 8, STE2007_MASK_COMDIR));
}

void LCD_SetPower(bool value){
	uint8_t power_byte = value ? 0x00 : 0x01;
	LCD_SendByte(LCD_C, getCommand(STE2007_CMD_ONOFF, power_byte, STE2007_MASK_ONOFF));
}

void LCD_SetInvert(bool value){
	uint8_t invert_byte = value ? 0x01 : 0x00;
	LCD_SendByte(LCD_C, getCommand(STE2007_CMD_DPYREV, invert_byte, STE2007_MASK_DPYREV));
}

void LCD_SetCharSize(uint8_t size){
	textsize = (size > 0) ? size : 1;
}

void LCD_SetTextColor(uint8_t c, uint8_t b){
	textcolor   = c;
	textbgcolor = b;
}

void LCD_SetCursor(uint8_t x, uint8_t y){
	cursor_x = x;
	cursor_y = y;
}

void LCD_AddToCursor(uint8_t x, uint8_t y){
	cursor_x += x;
	cursor_y += y;
}

// Инициализируем дисплей
void LCD_Init(LCD_CONFIG config) {
  // Инициализация дисплея
	lcd_config = config;
	HAL_GPIO_WritePin(lcd_config.RESPORT, lcd_config.RESPIN, GPIO_PIN_RESET); // Активируем ресет
	HAL_Delay(5);
	HAL_GPIO_WritePin(lcd_config.RESPORT, lcd_config.RESPIN, GPIO_PIN_SET);   // Деактивируем ресет
	HAL_GPIO_WritePin(lcd_config.SCKPORT, lcd_config.SCKPIN, GPIO_PIN_RESET);     // Тактовый вывод низкое состояние
	HAL_GPIO_WritePin(lcd_config.MOSIPORT, lcd_config.MOSIPIN, GPIO_PIN_RESET);   // Вывод данных в низкое состояние
	HAL_GPIO_WritePin(lcd_config.CSPORT, lcd_config.CSPIN, GPIO_PIN_RESET);			 // Выбираем дисплей
	// Задержка
	HAL_Delay(5);
	HAL_GPIO_WritePin(lcd_config.CSPORT, lcd_config.CSPIN, GPIO_PIN_SET);			 // Выбираем дисплей

	LCD_SendByte(LCD_C, getCommand(STE2007_CMD_RESET, 0, STE2007_MASK_RESET));  // Reset chip
	HAL_Delay(5);

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
	HAL_Delay(10);

	LCD_Update();
}
