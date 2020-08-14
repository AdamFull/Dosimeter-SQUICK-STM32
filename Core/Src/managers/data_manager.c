/*
 * data_manager.c
 *
 *  Created on: Aug 9, 2020
 *      Author: logot
 */


#include "managers/data_manager.h"
#include "stdint.h"
#include "stdlib.h"
#include "math.h"
#include "fatfs.h"
#include "string.h"

#include "parser.h"


//+++++++++++++++++++++VARIABLES+++++++++++++++++++++

uint32_t *stat_buff;		//Buffer for contain current stat values
volatile uint8_t stat_time = 0;

uint8_t Geiger_error = 2;
uint8_t GEIGER_TIME = 21, Real_geigertime = 21;
uint16_t Transformer_pwm = 60;

uint8_t LCD_contrast = 60;
uint16_t LCD_backlight = 0;
uint16_t Buzzer_tone = 20;

uint8_t Save_dose_interval = 20;

uint8_t Alarm_threshold = 100;

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
volatile bool is_detected = false;
bool is_memory_initialized = false;
bool active_hv_gen = false;

bool is_mean_mode = false;

uint8_t active_counters = 1;	//0 - external, 1 - first, 2 - second, 3 - first + second together
uint16_t *rad_buff;
uint64_t rad_dose_old;
volatile uint64_t rad_sum, rad_back, rad_max, rad_dose;
uint8_t time_min_old;
volatile uint8_t time_min = 1, time_sec = 1;
volatile uint16_t timer_time, timer_remain;
volatile uint8_t sum_old;
unsigned long alarm_timer;

uint16_t current_battery_voltage, current_high_voltage;

uint8_t counter_mode = 0;

volatile uint8_t mass[84];
volatile uint8_t x_p = 0;

float mean, std;

uint8_t page = 0;

DMGRESULT error_detector;

extern uint8_t retUSER;
extern FATFS USERFatFS;
extern FIL USERFile;
extern FATFS *pfs;
extern FRESULT fres;
extern DWORD fre_clust;
extern uint32_t total_memory, free_memory;

char configdata[] = "GEIGER_ERROR = 28\nGEIGER_TIME = 21\nTRANSFORMER_PWM = 60\nLCD_CONTRAST = 60\nLCD_BACKLIGHT = 0\nBUZZER_TONE = 50\nACTIVE_COUNTERS = 1\nCUMULATIVE_DOSE = 0\nCOUNTER_MODE = 0\nSAVE_DOSE_INTERVAL = 20\nALARM_THRESHOLD = 100\n\0";

void Initialize_variables(){

	//uint8_t init
	stat_time = 0;
	Geiger_error = 2;
	GEIGER_TIME = 21;
	Real_geigertime = 21;
	LCD_contrast = 60;
	Save_dose_interval = 20;
	active_counters = 1;
	time_min_old = 0;
	time_min = 1;
	time_sec = 0;
	sum_old = 0;

	//uint16_t init
	Transformer_pwm = 60;
	LCD_backlight = 0;
	Buzzer_tone = 200;
	timer_time = 0;
	timer_remain = 0;

	//uint32_t init
	rad_sum = 0;
	rad_back = 0;
	rad_max = 0;
	rad_dose = 0;
	rad_dose_old = 0;

}

void Initialize_data(){
	Initialize_variables();
	is_memory_initialized = Init_memory();
	if(is_memory_initialized){
		Read_configuration();
	}else{
		error_detector = FLASH_MEMORY_ERROR;
	}
	Reset_activity_test();
	Update_rad_buffer();
}

void Update_rad_buffer(){
	free(rad_buff);
	free(stat_buff);
	//Придумать как в любом режиме считать в одну переменную

	if(active_counters == 3) Real_geigertime = GEIGER_TIME/2;
	else Real_geigertime = GEIGER_TIME;
	rad_buff = calloc(Real_geigertime, sizeof(uint16_t));
	stat_buff = calloc(Real_geigertime, sizeof(uint32_t));

	if(rad_buff != NULL && stat_buff != NULL){
		rad_back = rad_max = 0;
		rad_dose = rad_dose_old;
		time_sec = time_min = 0;
		time_min = 1;
		for(unsigned i = 0; i < 83; i++) mass[i] = 0;
	}else{
		error_detector = HEAP_INITIALIZATION_ERROR;
	}
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

	if(retUSER == 0) {
		mount_status = f_mount(&USERFatFS, "", 1);
		if(mount_status == FR_OK){
			space_status = f_getfree("", &fre_clust, &pfs);
			total_memory = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
			free_memory = (uint32_t)(fre_clust * pfs->csize * 0.5);
			mount_status = mount_status & space_status;
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
	uint32_t file_size;
	if(free_memory < 1){
		open_status = f_open(&USERFile, file_name, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
		file_size = f_size(&USERFile);
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

bool Read_configuration(){
	configfile config;

	Read_memory("config.ini");

	open_config(&config, configdata);
	tokenize_config(&config);
	Geiger_error = get_token_by_name(&config, "GEIGER_ERROR").token_value;
	GEIGER_TIME = get_token_by_name(&config, "GEIGER_TIME").token_value;
	Transformer_pwm = get_token_by_name(&config, "TRANSFORMER_PWM").token_value;
	LCD_contrast = get_token_by_name(&config, "LCD_CONTRAST").token_value;
	LCD_backlight = get_token_by_name(&config, "LCD_BACKLIGHT").token_value;
	Buzzer_tone = get_token_by_name(&config, "BUZZER_TONE").token_value;
	active_counters = get_token_by_name(&config, "ACTIVE_COUNTERS").token_value;
	rad_sum = get_token_by_name(&config, "CUMULATIVE_DOSE").token_value;
	counter_mode = get_token_by_name(&config, "COUNTER_MODE").token_value;
	Save_dose_interval = get_token_by_name(&config, "SAVE_DOSE_INTERVAL").token_value;
	Alarm_threshold = get_token_by_name(&config, "ALARM_THRESHOLD").token_value;
	close_config(&config);
	return 0;
}

bool Write_configuration(){
	configfile config;

	//Read_memory("config.ini");

	open_config(&config, configdata);
	tokenize_config(&config);
	edit_token(&config, "GEIGER_ERROR", Geiger_error);
	edit_token(&config, "GEIGER_TIME", GEIGER_TIME);
	edit_token(&config, "TRANSFORMER_PWM", Transformer_pwm);
	edit_token(&config, "LCD_CONTRAST", LCD_contrast);
	edit_token(&config, "LCD_BACKLIGHT", LCD_backlight);
	edit_token(&config, "BUZZER_TONE", Buzzer_tone);
	edit_token(&config, "ACTIVE_COUNTERS", active_counters);
	edit_token(&config, "CUMULATIVE_DOSE", rad_sum);
	edit_token(&config, "COUNTER_MODE", counter_mode);
	edit_token(&config, "SAVE_DOSE_INTERVAL", Save_dose_interval);
	edit_token(&config, "ALARM_THRESHOLD", Alarm_threshold);
	close_config(&config);
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
	for(unsigned i = 0; i < Real_geigertime; i++) _sum+=stat_buff[i];
	mean = (float)_sum/Real_geigertime;
	_sum = 0;
	for(unsigned i = 0; i < Real_geigertime; i++) _sum+=pow(stat_buff[i] - mean, 2);
	std = (float)_sum/(float)(Real_geigertime-1);
}
