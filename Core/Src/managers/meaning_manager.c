/*
 * adc_manager.c
 *
 *  Created on: Aug 9, 2020
 *      Author: logot
 */


#include <managers/meaning_manager.h>
#include "stdbool.h"
#include "stm32f1xx_ll_adc.h"

#define BANK_SIZE 16

uint16_t battery_bank[BANK_SIZE];
uint16_t hv_bank[BANK_SIZE];
bool is_first_meaning_b = true, is_first_meaning_h = true;

uint8_t avgFactor = 5;
uint16_t batValue = 0;
uint16_t hvValue = 0;

uint16_t battery_adc_value, high_voltage_adc_value;


void adc_init(){
	uint16_t adc_calibration_index = 0;

	LL_ADC_DeInit(ADC1);
	LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_CONTINUOUS);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_0, LL_ADC_SAMPLINGTIME_239CYCLES_5);
	LL_ADC_SetSequencersScanMode(ADC1, LL_ADC_SEQ_SCAN_ENABLE);
	LL_ADC_INJ_SetTrigAuto(ADC1, LL_ADC_INJ_TRIG_FROM_GRP_REGULAR);
	LL_ADC_Enable(ADC1);
	adc_calibration_index = ((LL_ADC_DELAY_ENABLE_CALIB_ADC_CYCLES * 32) >> 1);
	while(adc_calibration_index != 0){ adc_calibration_index--; }
	LL_ADC_StartCalibration(ADC1);
	LL_ADC_StartCalibration(ADC1);
	while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0) {}
	LL_ADC_EnableIT_JEOS(ADC1);
	LL_ADC_INJ_StartConversionSWStart(ADC1);

	LL_ADC_DeInit(ADC2);
	LL_ADC_REG_SetContinuousMode(ADC2, LL_ADC_REG_CONV_CONTINUOUS);
	LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_9, LL_ADC_SAMPLINGTIME_239CYCLES_5);
	LL_ADC_SetSequencersScanMode(ADC2, LL_ADC_SEQ_SCAN_ENABLE);
	LL_ADC_INJ_SetTrigAuto(ADC2, LL_ADC_INJ_TRIG_FROM_GRP_REGULAR);
	LL_ADC_Enable(ADC2);
	adc_calibration_index = ((LL_ADC_DELAY_ENABLE_CALIB_ADC_CYCLES * 32) >> 1);
	while(adc_calibration_index != 0){ adc_calibration_index--; }
	LL_ADC_StartCalibration(ADC2);
	LL_ADC_StartCalibration(ADC2);
	while (LL_ADC_IsCalibrationOnGoing(ADC2) != 0) {}
	LL_ADC_EnableIT_JEOS(ADC2);
	LL_ADC_INJ_StartConversionSWStart(ADC2);
}

uint16_t get_battery_voltage(){
	uint16_t resulting_value = 0;
	for(int i = 0; i < 30; i++){
		batValue = (batValue * (avgFactor - 1) + battery_adc_value) / avgFactor;
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
		hvValue = (hvValue * (avgFactor - 1) + high_voltage_adc_value) / avgFactor;
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

void adc_enable_reading(){
}
