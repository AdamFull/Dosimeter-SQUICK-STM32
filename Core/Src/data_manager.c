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
#include "string.h"

//+++++++++++++++++++++VARIABLES+++++++++++++++++++++

uint32_t *Stat_buff;		//Buffer for contain current stat values
uint8_t Stat_time;

uint8_t Geiger_error;
uint8_t GEIGER_TIME, Real_geigertime;
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
bool is_memory_initialized = false;
bool active_hv_gen = false;

uint8_t active_counters;	//0 - external, 1 - first, 2 - second, 3 - first + second together
uint16_t *rad_buff;
uint32_t rad_sum, rad_back, rad_max, rad_dose, rad_dose_old;
uint8_t time_min_old, time_min, time_sec;
uint16_t timer_time, timer_remain;
uint8_t sum_old;
unsigned long alarm_timer;

uint8_t counter_mode;

uint8_t mass[84];
uint8_t x_p;

float mean;
float std;

uint8_t page;

DMGRESULT error_detector;

extern uint8_t retUSER;
extern FATFS USERFatFS;
extern FIL USERFile;
extern FATFS *pfs;
extern FRESULT fres;
extern DWORD fre_clust;
extern uint32_t total_memory, free_memory;

void Initialize_variables(){
	Stat_time = Geiger_error = GEIGER_TIME = Real_geigertime = LCD_contrast = Save_dose_interval =
			active_counters = time_min_old = time_min = time_sec = sum_old = 0;
	Transformer_pwm = LCD_backlight = Buzzer_tone = timer_time = timer_remain = 0;
	rad_sum = rad_back = rad_max = rad_dose = rad_dose_old = 0;

}

void Initialize_data(){
	Initialize_variables();
	is_memory_initialized = Init_memory();
	if(is_memory_initialized){
		Read_memory("config.ini");
		Reset_activity_test();
		Update_rad_buffer();
	}else{
		error_detector = FLASH_MEMORY_ERROR;
	}

}

void Update_rad_buffer(){
	free(rad_buff);
	free(Stat_buff);
	//Придумать как в любом режиме считать в одну переменную

	if(active_counters == 3) Real_geigertime = GEIGER_TIME/2;
	else Real_geigertime = GEIGER_TIME;
	rad_buff = malloc(Real_geigertime);
	Stat_buff = malloc(Real_geigertime);
	for(unsigned i = 0; i < Real_geigertime; i++){ rad_buff[i] = 0;}
	for(unsigned i = 0; i < Real_geigertime; i++){ Stat_buff[i] = 0;}
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
	FRESULT mount_status;
	FRESULT space_status = mount_status  = FR_INT_ERR;

	MX_FATFS_Init();
	if(retUSER == 0) {
		mount_status = f_mount(&USERFatFS, "", 1);
		if(mount_status == FR_OK){
			space_status = f_getfree("", &fre_clust, &pfs);
			total_memory = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
			free_memory = (uint32_t)(fre_clust * pfs->csize * 0.5);
		}
	}

	return mount_status == FR_OK;

}

bool Setup_memory(){
	if(free_memory < 1);
	return 0;
}

string Read_memory(string file_name){
	FRESULT open_status = FR_DENIED;
	if(free_memory < 1){
		open_status = f_open(&USERFile, file_name, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
		//f_read(fp, buff, btr, br);
	}

	return "";
}

bool Write_memory(string file_name, string file_data){
	FRESULT open_status = FR_DENIED;
	if(free_memory < 1){
		open_status = f_open(&USERFile, file_name, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
		if(open_status == FR_OK){
			//f_write(&USERFile, file_data, btw, bw)
		}
	}
	return open_status == FR_OK ? true : false;
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
