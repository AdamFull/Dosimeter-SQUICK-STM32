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
#include "libs/LCD_1202.h"

//#include "parser.h"

//geiger_settings GSETTING;
NVRAM DevNVRAM;
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

size_t free_rom, free_ram;

void Initialize_variables(){

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
	Init_configuration();
	GFLAGS.is_memory_initialized = Init_memory();
	Reset_activity_test();
	Update_rad_buffer();
	free_ram = GetRamFree();
	free_rom = GetRomFree();
}

void Update_rad_buffer(){
	if(DevNVRAM.GSETTING.ACTIVE_COUNTERS == 3) GWORK.real_geigertime = DevNVRAM.GSETTING.GEIGER_TIME/2;
	else GWORK.real_geigertime = DevNVRAM.GSETTING.GEIGER_TIME;
	free(GWORK.rad_buff);
	GWORK.rad_buff = (uint16_t*)calloc(GWORK.real_geigertime, sizeof(uint16_t));
	free(GWORK.stat_buff);
	GWORK.stat_buff = (uint32_t*)calloc(GWORK.real_geigertime, sizeof(uint32_t));

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

void Set_setting(uint32_t *value, uint32_t new_value){ *value = new_value; }

void Accept_settings(){
	if(Write_configuration()){
		LCD_SetContrast(DevNVRAM.GSETTING.LCD_CONTRAST);
		GFLAGS.is_muted = !(bool)DevNVRAM.GSETTING.BUZZER_TONE;
		Update_rad_buffer();
	}
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

size_t GetRamFree(){
	size_t clear_blocks = 0;
	uint32_t l_Address = RAM_START_ADDR;
	while(l_Address < 0x20005000){
		if(*(__IO uint32_t *)l_Address == 0x00000000 || *(__IO uint32_t *)l_Address == 0xFFFFFFFF) clear_blocks++;
		l_Address = l_Address + 4;
	}
	return clear_blocks * 4;
}

size_t GetRomFree(){
	size_t clear_blocks = 0;
	uint32_t l_Address = FLASH_START_ADDR;
	while(l_Address < 0x08010000){
		if(*(__IO uint32_t *)l_Address == 0x00000000 || *(__IO uint32_t *)l_Address == 0xFFFFFFFF) clear_blocks++;
		l_Address = l_Address + 4;
	}
	return clear_blocks * 4;
}

bool Read_configuration(){
	volatile uint32_t readed_mem;

	volatile uint32_t l_Address, l_Error, l_Index;

	l_Address = FLASH_CONFIG_START_ADDR;
	l_Error = 0x00;
	l_Index = 0x00;
	while(l_Address < FLASH_CONFIG_END_ADDR){
		readed_mem = *(__IO uint32_t *)l_Address;
		DevNVRAM.data32[l_Index] = readed_mem;
		l_Index = l_Index+1;
		l_Address = l_Address + 4;
	}

	if(DevNVRAM.GSETTING.CONFIG_KEY != GOOD_CONFIG_KEY){
		memset(DevNVRAM.data32, 0, sizeof(DevNVRAM.data32));

		DevNVRAM.GSETTING.CONFIG_KEY = GOOD_CONFIG_KEY;
		DevNVRAM.GSETTING.GEIGER_TIME = 21;
		DevNVRAM.GSETTING.GEIGER_ERROR = 2;
		DevNVRAM.GSETTING.GEIGER_VOLTAGE = 400;
		DevNVRAM.GSETTING.LCD_CONTRAST = 15;
		DevNVRAM.GSETTING.LCD_BACKLIGHT = 1;
		DevNVRAM.GSETTING.BUZZER_TONE = 1;
		DevNVRAM.GSETTING.ACTIVE_COUNTERS = 1;
		DevNVRAM.GSETTING.SAVE_DOSE_INTERVAL = 10;
		DevNVRAM.GSETTING.ALARM_THRESHOLD = 100;
		DevNVRAM.GSETTING.rad_sum = 0;
	}
}

bool Write_configuration(){
	static FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = FLASH_CONFIG_START_ADDR;
	EraseInitStruct.NbPages = 0x01;

	volatile uint32_t l_Address, l_Error, l_Index;

	l_Address = FLASH_CONFIG_START_ADDR;
	l_Error = 0x00;
	l_Index = 0x00;
	while(l_Address < FLASH_CONFIG_END_ADDR){
		if(DevNVRAM.data32[l_Index] != *(__IO uint32_t*)l_Address) l_Error += 1;
		l_Index = l_Index+1;
		l_Address = l_Address + 4;
	}

	if(l_Error > 0){
		HAL_FLASH_Unlock();
		HAL_FLASHEx_Erase(&EraseInitStruct, &l_Error);

		l_Address = FLASH_CONFIG_START_ADDR;
		l_Error = 0x00;
		l_Index = 0x00;
		DevNVRAM.sector.NWrite = DevNVRAM.sector.NWrite + 1;
		while(l_Address < FLASH_CONFIG_END_ADDR){
			//LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);
			LL_mDelay(1);
			if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, l_Address, DevNVRAM.data32[l_Index]) == HAL_OK){
				l_Index = l_Index+1;
				l_Address = l_Address + 4;
			}
			//LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);
			//LL_mDelay(10);
		}
		HAL_FLASH_Lock();
	}else{
		return false;
	}

	return true;
}

void Init_configuration(){
	Read_configuration();
	Write_configuration();
}

void Reset_dose(){
	DevNVRAM.GSETTING.rad_sum = 0;
	ReadWrite_configuration();
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
