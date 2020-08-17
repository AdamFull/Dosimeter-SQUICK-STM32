/*
 * data_manager.h
 *
 *  Created on: Aug 9, 2020
 *      Author: logot
 */

#ifndef INC_DATA_MANAGER_H_
#define INC_DATA_MANAGER_H_

#include "stdbool.h"
#include "stdint.h"
#include "stm32f1xx.h"

typedef const char* string;

typedef struct {
	uint8_t GEIGER_ERROR;
	uint8_t GEIGER_TIME;
	uint16_t GEIGER_VOLTAGE;

	uint8_t LCD_CONTRAST;
	uint16_t LCD_BACKLIGHT;

	uint16_t BUZZER_TONE;

	uint8_t ACTIVE_COUNTERS;	//0 - external, 1 - first, 2 - second, 3 - first + second together
	uint8_t SAVE_DOSE_INTERVAL;
	uint8_t ALARM_THRESHOLD;

} geiger_settings;

typedef struct {
	uint8_t time_min_old;
	volatile uint8_t stat_time;
	volatile uint8_t time_min, time_sec;
	volatile uint8_t sum_old;

	uint16_t real_geigertime;
	uint16_t transformer_pwm;
	volatile uint16_t timer_time, timer_remain;

	uint64_t rad_dose_old;
	volatile uint64_t rad_sum, rad_back, rad_max, rad_dose;

	unsigned long alarm_timer;

	uint16_t *rad_buff;
	uint32_t *stat_buff;		//Buffer for contain current stat values
} geiger_work;

typedef struct {
	uint16_t current_battery_voltage, current_high_voltage;
	float mean, std;
} geiger_meaning;

typedef struct {
	uint8_t counter_mode;
	uint8_t means_times;

} geiger_mode;

typedef struct {
	uint8_t page;
	uint8_t counter;
	uint8_t menu_page;
	uint8_t cursor;
	uint16_t editable;

	volatile uint8_t mass[84];
	volatile uint8_t x_p;
} geiger_ui;

typedef struct {
	bool stop_timer;
	bool next_step;
	bool no_alarm;
	bool do_alarm;
	bool is_sleeping;
	bool is_editing_mode;
	bool is_alarm;
	bool is_muted;
	bool is_low_voltage;
	bool is_charging;
	bool is_charged;
	bool is_detected;
	bool is_memory_initialized;
	bool active_hv_gen;

	bool is_mean_mode;
} geiger_flags;

typedef  enum {
	NO_ERROR,
	FLASH_MEMORY_ERROR,
	HEAP_INITIALIZATION_ERROR,

} DMGRESULT;

void Initialize_variables();
void Initialize_data();
void Update_rad_buffer();

void Save_dose();
void Save_tone();
void Save_backlight();
void Save_contrast();
void Save_geiger_time();
void Save_geiger_error();
void Save_dose_save_interval();
void Save_alarm_threshold();
void Reset_to_defaults();

bool Init_memory();
bool Setup_memory();
string Read_memory(string file_name);
bool Write_memory(string file_name, string file_data);
bool is_memory_valid();

bool Read_configuration();
bool Write_configuration();


void Reset_dose();

void Reset_activity_test();

volatile void Calculate_std();

#endif /* INC_DATA_MANAGER_H_ */
