/*
 * adc_manager.h
 *
 *  Created on: Aug 9, 2020
 *      Author: logot
 */

#ifndef INC_MEANING_MANAGER_H_
#define INC_MEANING_MANAGER_H_

#include "stdint.h"

void adc_init();
uint16_t get_battery_voltage();
uint16_t get_hv();

void pwm_transformer(uint8_t pwm);
void pwm_backlight(uint8_t pwm);
void pwm_tone(uint8_t pwm);

void adc_enable_reading();

#endif /* INC_MEANING_MANAGER_H_ */
