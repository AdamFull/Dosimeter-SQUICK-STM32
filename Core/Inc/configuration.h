/*
 * configuration.h
 *
 *  Created on: Aug 9, 2020
 *      Author: logot
 */

#ifndef INC_CONFIGURATION_H_
#define INC_CONFIGURATION_H_

#define VERSION "b1.1.4.0"

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

#define MEAN_MEAS_TIME 32

#define MAXIMUM_RAD_BUFEER_LEN 450

#define TRANSMITION_DELAY 10

#define LCD_X_SIZE 96
#define LCD_Y_SIZE 68

//Select language
//#define LANGUAGE_RU
#define LANGUAGE_EN

#ifdef LANGUAGE_RU
#undef LANGUAGE_EN
#elif defined(LANGUAGE_EN)
#undef LANGUAGE_RU
#else
#define LANGUAGE_EN
#endif

#define PAGES 9

//Time before screen will turn off
#define SCREEN_SAVER_TIME 30000

#define MAX_PARTICLES 0xffffffffffffffffULL
#define MAX_PARTICLES_PER_SEC

/***********************CONSTANTS***********************/
//High voltage convertation coefficient
#define HV_MULTIPLIER 5.75f

#define BACKLIGHT_MAX 255
#define BACKLIGHT_NORMAL 200
#define BACKLIGHT_MIN 150

#define BUZZER_NORMAL 200

#define BACKLIGHT_INTENSITY BACKLIGHT_NORMAL

//Battery minimum and maximum values (min = 3.3v, max = 4.2v)
#define BAT_ADC_MIN 2048
#define BAT_ADC_MAX 2590

//Select bluetooth
#define BLUETOOTH_SUPPORT
//#define BLE_SUPPORT

#ifdef BLUETOOTH_SUPPORT
#undef BLE_SUPPORT
#elif defined(BLE_SUPPORT)
#undef BLUETOOTH_SUPPORT
#endif

//#define DEBUG

#endif /* INC_CONFIGURATION_H_ */
