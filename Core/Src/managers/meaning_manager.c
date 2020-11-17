/*
 * adc_manager.c
 *
 *  Created on: Aug 9, 2020
 *      Author: logot
 */


#include <managers/meaning_manager.h>
#include "stdbool.h"
#include "stm32f1xx_ll_adc.h"
#include "stm32f1xx_ll_dma.h"

#define BANK_SIZE 16

uint16_t battery_bank[BANK_SIZE];
uint16_t hv_bank[BANK_SIZE];
bool is_first_meaning_b = true, is_first_meaning_h = true;

uint8_t avgFactor = 5;
uint16_t batValue = 0;
uint16_t hvValue = 0;

__IO uint16_t adc_values[2] = {0};
bool tc_flag;

void ADC_DMA_TransferComplete_Callback(){
	tc_flag = true;
}


void adc_init(){
	LL_DMA_ConfigTransfer(DMA1,
	                        LL_DMA_CHANNEL_1,
	                        LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
	                        LL_DMA_MODE_CIRCULAR              |
	                        LL_DMA_PERIPH_NOINCREMENT         |
	                        LL_DMA_MEMORY_INCREMENT           |
	                        LL_DMA_PDATAALIGN_HALFWORD        |
	                        LL_DMA_MDATAALIGN_HALFWORD        |
	                        LL_DMA_PRIORITY_HIGH               );
	LL_DMA_ConfigAddresses(DMA1,
	                         LL_DMA_CHANNEL_1,
	                         LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
	                         (uint32_t)&adc_values,
	                         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 2);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

	LL_ADC_Enable(ADC1);
	uint32_t wait_loop_index = ((LL_ADC_DELAY_ENABLE_CALIB_ADC_CYCLES * 32) >> 1);
	while(wait_loop_index != 0) wait_loop_index--;
	LL_ADC_StartCalibration(ADC1);
	while(LL_ADC_IsCalibrationOnGoing(ADC1) != 0);
}

uint16_t get_battery_voltage(){
	LL_ADC_REG_StartConversionSWStart(ADC1);
	uint16_t resulting_value = 0;
	for(int i = 0; i < 30; i++){
		batValue = (batValue * (avgFactor - 1) + adc_values[0]) / avgFactor;
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
	LL_ADC_REG_StartConversionSWStart(ADC1);
	uint16_t resulting_value = 0;
	for(int i = 0; i < 30; i++){
		hvValue = (hvValue * (avgFactor - 1) + adc_values[1]) / avgFactor;
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
