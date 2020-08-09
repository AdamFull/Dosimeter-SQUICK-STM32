/*
 * data_manager.c
 *
 *  Created on: Aug 9, 2020
 *      Author: logot
 */


#include "data_manager.h"
#include "stdint.h"
#include "stdlib.h"
#include "math.h"
#include "fatfs.h"

//+++++++++++++++++++++VARIABLES+++++++++++++++++++++

uint32_t *Stat_buff;		//Buffer for contain current stat values
uint8_t Stat_time;

uint8_t Geiger_error;
uint8_t GEIGER_TIME;
uint16_t Transformer_pwm;

uint8_t LCD_contrast;
uint16_t LCD_backlight;
uint16_t Buzzer_tone;

uint8_t Save_dose_interval;

//-----------------------FLAGS-----------------------
bool stop_timer = false;
bool next_step = false;
bool no_alarm = false;
bool do_alarm = false;
bool is_sleeping = false;
bool is_editing_mode = false;
bool is_alarm = false;
bool is_muted = false;
bool is_low_voltage = false;
bool is_charging = false;
bool is_charged = false;
bool is_detected = false;

uint16_t *rad_buff;
uint32_t rad_sum, rad_back, rad_max, rad_dose, rad_dose_old;
uint8_t time_min_old, time_min, time_sec;
uint16_t timer_time, timer_remain;
uint8_t sum_old;
unsigned long alarm_timer;

uint8_t mass[84];
uint8_t x_p;

float mean;
float std;

extern FATFS USERFatFS;

void Initialize_data(){
	Read_memory();
	Reset_activity_test();
	Update_rad_buffer();
}

void Update_rad_buffer(){
	free(rad_buff);
	free(Stat_buff);
	rad_buff = malloc(GEIGER_TIME);
	Stat_buff = malloc(GEIGER_TIME);
	for(unsigned i = 0; i < GEIGER_TIME; i++){ rad_buff[i] = 0;}
	for(unsigned i = 0; i < GEIGER_TIME; i++){ Stat_buff[i] = 0;}
	rad_back = rad_max = 0;
	rad_dose = rad_dose_old;
	time_sec = time_min = 0;
	time_min = 1;
	for(unsigned i = 0; i < 83; i++) mass[i] = 0;
}

void Save_dose(){

}

void Save_tone(){

}

void Save_backlight(){

}

void Save_contrast(){

}

void Save_geiger_time(){

}

void Save_geiger_error(){

}

void Save_dose_save_interval(){

}

void Save_alarm_threshold(){

}

void Reset_to_defaults(){

}

bool Init_memory(){
	return f_mount(&USERFatFS, "", 1);
}

void Setup_memory(){

}

void Read_memory(){

}

bool is_memory_valid(){
	return 0;
}

void Reset_dose(){

}

void Reset_activity_test(){
	is_alarm = false;
	rad_max = 0;
	rad_back = 0;
	stop_timer = false;
	//if(means_times == 0) next_step = true;
	//else next_step = false;
	time_sec = 0;
	//menu_page = 0;
	//counter_mode = 1;
	//page = 1;
}

void Calculate_std(){
	uint64_t _sum = 0;
	for(unsigned i = 0; i < GEIGER_TIME; i++) _sum+=Stat_buff[i];
	mean = (float)_sum/GEIGER_TIME;
	_sum = 0;
	for(unsigned i = 0; i < GEIGER_TIME; i++) _sum+=pow(Stat_buff[i] - mean, 2);
	std = (float)_sum/(float)(GEIGER_TIME-1);
}
