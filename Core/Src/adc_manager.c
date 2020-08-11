/*
 * adc_manager.c
 *
 *  Created on: Aug 9, 2020
 *      Author: logot
 */


#include "adc_manager.h"

#include "stdbool.h"
#include "stm32f1xx_hal.h"

uint16_t battery_bank[5];
bool first_mean = true;

uint8_t avgFactor = 5;
uint16_t batValue = 0;
uint16_t hvValue = 0;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;


void adc_init(){

}

uint16_t get_battery_voltage(){
	return 0;
}

uint16_t get_hv(){
	return 0;
}

void pwm_transformer(uint8_t pwm){
	TIM2->CCR1 = pwm;
}

void pwm_backlight(uint8_t pwm){
	TIM2->CCR2 = pwm;
}

void pwm_tone(uint8_t pwm){
	TIM2->CCR3 = pwm;
}

uint16_t adc_battery_channel_read(){
	uint16_t adc;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	adc = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return adc;
}

uint16_t adc_feedback_channel_read(){
	uint16_t adc;
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 100);
	adc = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);
	return adc;
}
