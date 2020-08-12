/*
 * adc_manager.c
 *
 *  Created on: Aug 9, 2020
 *      Author: logot
 */


#include <meaning_manager.h>
#include "stdbool.h"
#include "stm32f1xx_hal.h"

#define BANK_SIZE 5

uint16_t battery_bank[BANK_SIZE];
uint16_t hv_bank[BANK_SIZE];
bool is_first_meaning_b = true, is_first_meaning_h = true;

uint8_t avgFactor = 5;
uint16_t batValue = 0;
uint16_t hvValue = 0;

bool is_first_meaning = true;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;


void adc_init(){

}

uint16_t get_battery_voltage(){
	uint16_t resulting_value = 0;
	for(int i = 0; i < 30; i++){
		batValue = (batValue * (avgFactor - 1) + adc_battery_channel_read()) / avgFactor;
	}
	if(is_first_meaning_b){
		for(uint8_t i = 0; i < BANK_SIZE; i++) battery_bank[i] = batValue;
		is_first_meaning_b = false;
	}else{
		battery_bank[0] = batValue;
		for(uint8_t k = BANK_SIZE-1; k>0; k--) battery_bank[k]=battery_bank[k-1]; //перезапись массива
	}
	for(uint8_t i = 0; i < BANK_SIZE; i++) resulting_value += battery_bank[i];
	resulting_value = resulting_value/BANK_SIZE;

	return resulting_value;
}

uint16_t get_high_voltage(){
	uint16_t resulting_value = 0;
	for(int i = 0; i < 30; i++){
		hvValue = (hvValue * (avgFactor - 1) + adc_feedback_channel_read()) / avgFactor;
	}
	if(is_first_meaning_h){
		for(uint8_t i = 0; i < BANK_SIZE; i++) hv_bank[i] = hvValue;
		is_first_meaning_h = false;
	}else{
		hv_bank[0] = hvValue;
		for(uint8_t k = BANK_SIZE-1; k>0; k--) hv_bank[k]=hv_bank[k-1]; //перезапись массива
	}
	for(uint8_t i = 0; i < BANK_SIZE; i++) resulting_value += hv_bank[i];
	resulting_value = resulting_value/BANK_SIZE;

	return resulting_value;
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
	HAL_ADC_PollForConversion(&hadc1, 50);
	adc = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return adc;
}

uint16_t adc_feedback_channel_read(){
	uint16_t adc;
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 50);
	adc = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);
	return adc;
}
