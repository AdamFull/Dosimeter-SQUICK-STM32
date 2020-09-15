/*
 * data_manager.c
 *
 *  Created on: Aug 9, 2020
 *      Author: logot
 */


#include "managers/data_manager.h"
#include "stdlib.h"
#include "math.h"
#include "fatfs.h"
#include "string.h"

#include "parser.h"

geiger_settings GSETTING;
geiger_work GWORK;
geiger_meaning GMEANING;
geiger_flags GFLAGS;
geiger_mode GMODE;
geiger_ui GUI;

DMGRESULT error_detector;

extern uint8_t retUSER;
extern FATFS USERFatFS;
extern FIL USERFile;
extern FATFS *pfs;
extern FRESULT fres;
extern DWORD fre_clust;
extern uint32_t total_memory, free_memory;

char configdata[512] = "GEIGER_ERROR = 28\nGEIGER_TIME = 21\nTRANSFORMER_PWM = 60\nLCD_CONTRAST = 60\nLCD_BACKLIGHT = 0\nBUZZER_TONE = 50\nACTIVE_COUNTERS = 1\nCUMULATIVE_DOSE = 0\nCOUNTER_MODE = 0\nSAVE_DOSE_INTERVAL = 20\nALARM_THRESHOLD = 100\n\0";

void Initialize_variables(){
	GSETTING.GEIGER_ERROR = 2;
	GSETTING.GEIGER_TIME = 21;
	GSETTING.GEIGER_VOLTAGE = 400;
	GSETTING.LCD_CONTRAST = 60;
	GSETTING.LCD_BACKLIGHT = 0;
	GSETTING.BUZZER_TONE = 200;
	GSETTING.ACTIVE_COUNTERS = 1;
	GSETTING.SAVE_DOSE_INTERVAL = 20;
	GSETTING.ALARM_THRESHOLD = 100;

	GWORK.time_min_old = 0;
	GWORK.time_min = 1;
	GWORK.time_sec = 0;
	GWORK.stat_time = 0;
	GWORK.sum_old = 0;
	GWORK.real_geigertime = 0;
	GWORK.transformer_pwm = 0;
	GWORK.timer_time = 0;
	GWORK.timer_remain = 0;
	GWORK.rad_dose_old = 0;
	GWORK.rad_sum = 0;
	GWORK.rad_back = 0;
	GWORK.rad_max = 0;
	GWORK.rad_dose = 0;
	GWORK.alarm_timer = 0;

	GMEANING.current_battery_voltage = 0;
	GMEANING.current_high_voltage = 0;
	GMEANING.mean = 0.f;
	GMEANING.std = 0.f;

	GMODE.counter_mode = 0;

	GFLAGS.stop_timer = false;
	GFLAGS.next_step = false;
	GFLAGS.no_alarm = false;
	GFLAGS.do_alarm = false;
	GFLAGS.is_sleeping = false;
	GFLAGS.is_editing_mode = false;
	GFLAGS.is_alarm = false;
	GFLAGS.is_muted = false;
	GFLAGS.is_low_voltage = false;
	GFLAGS.is_charging = false;
	GFLAGS.is_charged = false;
	GFLAGS.is_detected = false;
	GFLAGS.is_memory_initialized = false;
	GFLAGS.active_hv_gen = false;
	GFLAGS.is_mean_mode = false;

	GUI.counter = 0;
	GUI.cursor = 0;
	GUI.editable = 0;
	GUI.menu_page = 0;
	GUI.page = 0;

}

void Initialize_data(){
	Initialize_variables();
	GFLAGS.is_memory_initialized = Init_memory();

	printf("w25qxx memory init status: %s\n", GFLAGS.is_memory_initialized ? "true" : "false");

	if(GFLAGS.is_memory_initialized){
		Read_configuration();
		Write_configuration();
	}else{
		error_detector = FLASH_MEMORY_ERROR;
	}
	Reset_activity_test();
	Update_rad_buffer();

	memset(configdata, 0, strlen(configdata));
}

void Update_rad_buffer(){

	if(GSETTING.ACTIVE_COUNTERS == 3) GWORK.real_geigertime = GSETTING.GEIGER_TIME/2;
	else GWORK.real_geigertime = GSETTING.GEIGER_TIME;
	free(GWORK.rad_buff);
	GWORK.rad_buff = (uint16_t*)calloc(GWORK.real_geigertime, sizeof(uint16_t));
	free(GWORK.stat_buff);
	GWORK.stat_buff = (uint32_t*)calloc(GWORK.real_geigertime, sizeof(uint32_t));

	printf("Allocated %d bytes for rad buffer on address %p\n", GWORK.real_geigertime * sizeof(uint16_t), GWORK.rad_buff);
	printf("Allocated %d bytes for stats buffer on address %p\n", GWORK.real_geigertime * sizeof(uint32_t), GWORK.stat_buff);

	if(GWORK.rad_buff != NULL && GWORK.stat_buff != NULL){
		GWORK.rad_back = GWORK.rad_max = 0;
		GWORK.rad_dose = GWORK.rad_dose_old;
		GWORK.time_sec = GWORK.time_min = 0;
		GWORK.time_min = 1;
		for(unsigned i = 0; i < 83; i++) GUI.mass[i] = 0;
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
	GSETTING.GEIGER_ERROR = get_token_by_name(&config, "GEIGER_ERROR").token_value;
	GSETTING.GEIGER_TIME = get_token_by_name(&config, "GEIGER_TIME").token_value;
	GSETTING.GEIGER_VOLTAGE = get_token_by_name(&config, "TRANSFORMER_PWM").token_value;
	GSETTING.LCD_CONTRAST = get_token_by_name(&config, "LCD_CONTRAST").token_value;
	GSETTING.LCD_BACKLIGHT = get_token_by_name(&config, "LCD_BACKLIGHT").token_value;
	GSETTING.BUZZER_TONE = get_token_by_name(&config, "BUZZER_TONE").token_value;
	GSETTING.ACTIVE_COUNTERS = get_token_by_name(&config, "ACTIVE_COUNTERS").token_value;
	GWORK.rad_sum = get_token_by_name(&config, "CUMULATIVE_DOSE").token_value;
	GMODE.counter_mode = get_token_by_name(&config, "COUNTER_MODE").token_value;
	GSETTING.SAVE_DOSE_INTERVAL = get_token_by_name(&config, "SAVE_DOSE_INTERVAL").token_value;
	GSETTING.ALARM_THRESHOLD = get_token_by_name(&config, "ALARM_THRESHOLD").token_value;
	close_config(&config);
	return 0;
}

bool Write_configuration(){
	configfile config;

	//Read_memory("config.ini");

	open_config(&config, configdata);
	tokenize_config(&config);
	edit_token(&config, "GEIGER_ERROR", GSETTING.GEIGER_ERROR);
	edit_token(&config, "GEIGER_TIME", GSETTING.GEIGER_TIME);
	edit_token(&config, "TRANSFORMER_PWM", GSETTING.GEIGER_VOLTAGE);
	edit_token(&config, "LCD_CONTRAST", GSETTING.LCD_CONTRAST);
	edit_token(&config, "LCD_BACKLIGHT", GSETTING.LCD_BACKLIGHT);
	edit_token(&config, "BUZZER_TONE", GSETTING.BUZZER_TONE);
	edit_token(&config, "ACTIVE_COUNTERS", GSETTING.ACTIVE_COUNTERS);
	edit_token(&config, "CUMULATIVE_DOSE", GWORK.rad_sum);
	edit_token(&config, "COUNTER_MODE", GMODE.counter_mode);
	edit_token(&config, "SAVE_DOSE_INTERVAL", GSETTING.SAVE_DOSE_INTERVAL);
	edit_token(&config, "ALARM_THRESHOLD", GSETTING.ALARM_THRESHOLD);
	write_config(&config);
	close_config(&config);
	return 0;
}

void Reset_dose(){

}

void Reset_activity_test(){
	GFLAGS.is_alarm = false;
	GWORK.rad_max = 0;
	GWORK.rad_back = 0;
	GFLAGS.stop_timer = false;
	if(GMODE.means_times == 0) GFLAGS.next_step = true;
	else GFLAGS.next_step = false;
	GWORK.time_sec = 0;
	GUI.menu_page = 0;
	GMODE.counter_mode = 1;
	GUI.page = 1;
}

void Calculate_std(){
	uint64_t _sum = 0;
	for(unsigned i = 0; i < GWORK.real_geigertime; i++) _sum+=GWORK.stat_buff[i];
	GMEANING.mean = (float)_sum/GWORK.real_geigertime;
	_sum = 0;
	for(unsigned i = 0; i < GWORK.real_geigertime; i++) _sum+=pow(GWORK.stat_buff[i] - GMEANING.mean, 2);
	GMEANING.std = (float)_sum/(float)(GWORK.real_geigertime-1);
}
