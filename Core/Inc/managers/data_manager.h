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
#include "configuration.h"

#define FLASH_START_ADDR ((uint32_t)0x08000000)
#define RAM_START_ADDR ((uint32_t)0x20000000)
#define FLASH_CONFIG_START_ADDR ((uint32_t)0x0800F000)
#define FLASH_CONFIG_END_ADDR FLASH_CONFIG_START_ADDR + FLASH_PAGE_SIZE
#define GOOD_CONFIG_KEY 0x2000CA32

typedef const char* string;

typedef struct {
	uint32_t CONFIG_KEY;
	uint32_t GEIGER_TIME;
	uint32_t GEIGER_ERROR;
	uint32_t GEIGER_VOLTAGE;

	uint32_t LCD_CONTRAST;
	uint32_t LCD_BACKLIGHT;

	uint32_t BUZZER_TONE;

	uint32_t ACTIVE_COUNTERS;	//0 - external, 1 - first, 2 - second, 3 - first + second together
	uint32_t SAVE_DOSE_INTERVAL;
	uint32_t ALARM_THRESHOLD;

	uint32_t w25qxx_address;

	volatile uint32_t rad_sum;
	uint32_t sensor_area;
	uint32_t log_save_period;

	unsigned session_number;

} geiger_settings;

typedef union {
	geiger_settings GSETTING;
	uint32_t data32[15];
} NVRAM;

typedef struct {
	uint32_t voltage_req;
	uint8_t time_min_old;
	volatile uint8_t time_min, time_sec;
	volatile uint16_t sum_old;

	uint16_t real_geigertime;
	uint16_t transformer_pwm;
	uint16_t timer_time;
	volatile uint16_t timer_remain;

	unsigned rad_dose_old;
	volatile unsigned rad_back, rad_max, rad_dose, rad_back_old;

	unsigned long alarm_timer;

	uint16_t rad_buff[MAXIMUM_RAD_BUFEER_LEN];
	uint32_t stat_buff[MEAN_MEAS_TIME];
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
	int16_t editable;
	bool editable_bool;

	bool update_required;

	uint8_t mass[96];
	volatile uint8_t x_p;
} geiger_ui;

typedef struct {
	volatile bool stop_timer;
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
	volatile bool is_detected;
	bool is_memory_initialized;
	bool is_tracking_enabled;
	bool log_mutex;
	bool is_monitor_enabled;
	bool is_flash_initialized;
	bool calculate_dose;
	bool is_sleep_mode;

	bool is_particle_mode;
	bool is_mean_mode;
	bool is_particle_per_sec_mode;
} geiger_flags;

typedef  enum {
	INIT_COMPLETE,
	EXT_MEMORY_INIT_ERROR,
	EXT_MEMORY_IS_OVERFLOW,
	WRITE_CONFIG_ERROR,
	HEAP_INIT_ERROR,

} DINITSTATUS;

void voltage_required();

void Initialize_variables();
void Initialize_data();
void Update_rad_buffer();

void Set_setting(uint32_t *value, uint32_t new_value);
void Accept_settings();
void Reset_to_defaults();

uint32_t GetRamFree();
uint32_t GetRomFree();

bool Read_configuration();
bool Write_configuration();

bool Init_w25qxx();
bool Write_string_w25qxx(uint8_t* str);
bool Read_string_w25qxx(uint32_t addr);
bool Erase_w25qxx();

void Save_dose();

void transmit_log();

void Reset_dose();
void Reset_settings();
void Clear_memory();
void Erase_memory();

void Reset_activity_test();

volatile void Calculate_std();
void activity_test_timer_ticker();
void geiger_counter_ticker();
void sleep();
void update_selected_counter();
void interrupts_handler();

#endif /* INC_DATA_MANAGER_H_ */
