/*
 * adc_manager.c
 *
 *  Created on: Aug 9, 2020
 *      Author: logot
 */


#include "adc_manager.h"

#include "stdbool.h"

uint16_t battery_bank[5];
bool first_mean = true;

uint8_t avgFactor = 5;
uint16_t batValue = 0;
uint16_t hvValue = 0;


void adc_init(){

}

uint16_t get_battery_voltage(){

}

uint16_t get_hv(){

}

void pwm_transformer(uint8_t pwm){

}

void pwm_backlight(uint8_t pwm){

}

void pwm_tone(uint8_t pwm){

}

uint16_t adc_battery_channel_read(){

}

uint16_t adc_feedback_channel_read(){

}
