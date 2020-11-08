/*
 * configuration.h
 *
 *  Created on: Aug 9, 2020
 *      Author: logot
 */

#ifndef INC_CONFIGURATION_H_
#define INC_CONFIGURATION_H_

#define VERSION "b1.0.0.0"

/************************PINOUTS***********************/

//Display
#define DISPLAY_MOSI_PORT 					GPIOA
#define DISPLAY_SCK_PORT 					GPIOA
#define DISPLAY_CS_PORT 					GPIOA
#define DISPLAY_RST_PORT 					GPIOB
#define DISPLAY_MOSI_PIN 					LL_GPIO_PIN_7
#define DISPLAY_SCK_PIN 					LL_GPIO_PIN_5
#define DISPLAY_CS_PIN 						LL_GPIO_PIN_4
#define DISPLAY_RST_PIN 					LL_GPIO_PIN_0

//Buttons
#define BUTTON_SET_PORT 					GPIOB
#define BUTTON_RESET_PORT 					GPIOB
#define BUTTON_SET_PIN						GPIO_IDR_IDR5
#define BUTTON_RESET_PIN					GPIO_IDR_IDR4

/************************SETTINGS***********************/

#define LCD_X_SIZE 96
#define LCD_Y_SIZE 68

#define LANGUAGE_RU
//#define LANGUAGE_EN

#define PAGES 9

//Time before screen will turn off
#define SCREEN_SAVER_TIME 60000

/***********************CONSTANTS***********************/
//High voltage convertation coefficient
#define HV_COEF 6.5f

#define BACKLIGHT_MAX 255
#define BACKLIGHT_NORMAL 200
#define BACKLIGHT_MIN 150

#define BACKLIGHT_INTENSITY BACKLIGHT_NORMAL

//Battery minimum and maximum values (min = 3.6v, max = 4.2v)
#define BAT_ADC_MIN 2234
#define BAT_ADC_MAX 2606

//#define DEBUG

#endif /* INC_CONFIGURATION_H_ */
