/*
 * data_manager.c
 *
 *  Created on: Aug 9, 2020
 *      Author: logot
 */


#include "managers/data_manager.h"
#include "stdlib.h"
#include "math.h"
#include "string.h"
#include "libs/LCD_1202.h"
#include "libs/w25qxx.h"
#include "configuration.h"
#include "stm32f1xx_ll_exti.h"

NVRAM DevNVRAM;
geiger_work GWORK;
geiger_meaning GMEANING;
geiger_flags GFLAGS;
geiger_mode GMODE;
geiger_ui GUI;

uint32_t flash_init_attempts = 0;

DINITSTATUS device_status;

uint8_t current_hour = 0;
uint8_t current_minutes = 0;
uint8_t current_seconds = 0;
unsigned long my_ticker = 0;

extern RTC_HandleTypeDef hrtc;
extern bool screen_saver_state;
RTC_TimeTypeDef current_time;
RTC_DateTypeDef current_date;

uint32_t awake_time;

/*****************************************************************************************************************/
void voltage_required(){
	GWORK.voltage_req = DevNVRAM.GSETTING.GEIGER_VOLTAGE*HV_MULTIPLIER;
}

/*****************************************************************************************************************/
void Initialize_variables(){

	GWORK.voltage_req = 0;
	GWORK.time_min_old = 0;
	GWORK.time_min = 1;
	GWORK.time_sec = 0;
	GWORK.sum_old = 0;
	GWORK.real_geigertime = 0;
	GWORK.transformer_pwm = 120;
	GWORK.timer_time = 0;
	GWORK.timer_remain = 0;
	GWORK.rad_dose_old = 0;
	GWORK.rad_back = 0;
	GWORK.rad_back_old = 0;
	GWORK.rad_max = 0;
	GWORK.rad_dose = 0;
	GWORK.alarm_timer = 0;

	GMEANING.current_battery_voltage = 0;
	GMEANING.current_high_voltage = 0;
	GMEANING.mean = 0.f;
	GMEANING.std = 0.f;

	GMODE.counter_mode = 0;
	GMODE.means_times = 0;

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
	GFLAGS.is_tracking_enabled = true;
	GFLAGS.is_mean_mode = false;
	GFLAGS.is_flash_initialized = false;
	GFLAGS.is_particle_mode = false;
	GFLAGS.calculate_dose = true;
	GFLAGS.is_sleep_mode;

	GUI.counter = 0;
	GUI.cursor = 0;
	GUI.editable = 0;
	GUI.menu_page = 0;
	GUI.page = 0;

}

/*****************************************************************************************************************/
uint8_t* separate_uint32_t(uint32_t value){
	static uint8_t bytes[4];
	bytes[0] = (value >> 24) & 0xFF;
	bytes[1] = (value >> 16) & 0xFF;
	bytes[2] = (value >> 8) & 0xFF;
	bytes[3] = value & 0xFF;
	return bytes;
}

/*****************************************************************************************************************/
bool Write_4byte(uint32_t value, uint32_t start_address){
	uint8_t *bytes = separate_uint32_t(value);
	uint32_t current_addr = start_address;
	for(size_t i = 0; i < 4; i++) {
		W25qxx_WriteByte(bytes[i], current_addr);
		current_addr++;
	}
	return true;
}

/*****************************************************************************************************************/
uint32_t Read_4byte(uint32_t start_address){
	uint32_t value;
	uint8_t bytes[4];
	uint32_t current_addr = start_address;
	for(size_t i = 0; i < 4; i++) {
		W25qxx_ReadByte(&bytes[i], current_addr);
		current_addr++;
	}
	value = (uint32_t) (bytes[0] << 24 | bytes[1] << 16 | bytes[2] << 8 | bytes[3]);
	return value;
}

/*****************************************************************************************************************/
bool Init_w25qxx(){
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12);
	uint32_t timeout = 0;
	while(!W25qxx_Init()){
		if(timeout < 30){
			LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
			LL_mDelay(100);
			timeout++;
		}else{
			return false;
		}
	}
	Read_configuration();
	Write_configuration();
	GFLAGS.is_flash_initialized = true;
	return true;
}

/*****************************************************************************************************************/
bool Write_string_w25qxx(uint8_t* str){
	if(!GFLAGS.log_mutex){
		if((w25qxx.CapacityInKiloByte*1024) - (DevNVRAM.GSETTING.w25qxx_address + strlen(str)) > 0){
			for(size_t i = 0; i < strlen(str); i++){
				W25qxx_WriteByte(str[i], DevNVRAM.GSETTING.w25qxx_address);
				DevNVRAM.GSETTING.w25qxx_address++;
			}
			Write_configuration();
			return true;
		}else{		//Memory not enougth
			device_status = EXT_MEMORY_IS_OVERFLOW;
			return false;
		}
	}
}

/*****************************************************************************************************************/
bool Read_string_w25qxx(uint32_t addr){

}

/*****************************************************************************************************************/
bool Erase_w25qxx(){
	uint8_t first_byte;
	W25qxx_EraseChip();
	Write_configuration();
	W25qxx_ReadByte(&first_byte, 0x00);
	return (first_byte == 0x0 || first_byte == 0xFF);
}

/*****************************************************************************************************************/
void Initialize_data(){
	if(Init_w25qxx()){
		Initialize_variables();
		Reset_activity_test();
		Update_rad_buffer();
		voltage_required();
		device_status = INIT_COMPLETE;
	}else{
		device_status = EXT_MEMORY_INIT_ERROR;
	}
}

/*****************************************************************************************************************/
void Update_rad_buffer(){
	GFLAGS.log_mutex = true;
	if(DevNVRAM.GSETTING.ACTIVE_COUNTERS == 3) GWORK.real_geigertime = DevNVRAM.GSETTING.GEIGER_TIME/2;
	else GWORK.real_geigertime = DevNVRAM.GSETTING.GEIGER_TIME;
	for(uint32_t i = 0; i < MAXIMUM_RAD_BUFEER_LEN; i++) GWORK.rad_buff[i] = 0;
	for(uint32_t i = 0; i < MEAN_MEAS_TIME; i++) GWORK.stat_buff[i] = 0;

	if(GWORK.rad_buff != NULL){
		GWORK.rad_back = GWORK.rad_max = 0;
		GWORK.rad_dose = GWORK.rad_dose_old;
		GWORK.time_sec = GWORK.time_min = 0;
		GWORK.time_min = 1;
		for(unsigned i = 0; i < 83; i++) GUI.mass[i] = 0;
	}else{
		device_status = HEAP_INIT_ERROR;
	}
	GFLAGS.log_mutex = false;
}

/*****************************************************************************************************************/
void Set_setting(uint32_t *value, uint32_t new_value){ *value = new_value; }

/*****************************************************************************************************************/
void Accept_settings(){
	if(Write_configuration()){
		LCD_SetContrast(DevNVRAM.GSETTING.LCD_CONTRAST);
		GFLAGS.is_muted = !(bool)DevNVRAM.GSETTING.BUZZER_TONE;
		Update_rad_buffer();
	}else{

	}
}

/*****************************************************************************************************************/
void Reset_to_defaults(){

}

/*****************************************************************************************************************/
uint32_t GetRamFree(){
	size_t clear_blocks = 0;
	uint32_t l_Address = RAM_START_ADDR;
	while(l_Address < 0x20005000){
		if(*(__IO uint32_t *)l_Address == 0x00000000 || *(__IO uint32_t *)l_Address == 0xFFFFFFFF) clear_blocks++;
		l_Address = l_Address + 4;
	}
	return clear_blocks * 4;
}

/*****************************************************************************************************************/
uint32_t GetRomFree(){
	uint32_t clear_blocks = 0;
	uint32_t l_Address = FLASH_START_ADDR;
	while(l_Address < 0x08010000){
		if(*(__IO uint32_t *)l_Address == 0x00000000 || *(__IO uint32_t *)l_Address == 0xFFFFFFFF) clear_blocks++;
		l_Address = l_Address + 4;
	}
	return clear_blocks * 4;
}

/*****************************************************************************************************************/
bool Read_configuration(){
	uint32_t readed_mem;
	uint8_t array_len = 15;

	uint32_t l_Address, l_Index;

	l_Address = 0x00;
	l_Index = 0x00;
	while(l_Address < 0x04*array_len){
		readed_mem = Read_4byte(l_Address);
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
		DevNVRAM.GSETTING.w25qxx_address = 0x00001000;
		DevNVRAM.GSETTING.sensor_area = 7;
		DevNVRAM.GSETTING.log_save_period = 5;
		DevNVRAM.GSETTING.session_number = 0;

		Write_configuration();
	}
}

/*****************************************************************************************************************/
bool Write_configuration(){
	uint32_t l_Address, l_Index, l_Error;
	uint8_t array_len = 15;

	l_Address = 0x00;
	l_Index = 0x00;
	l_Error = 0x00;
	while(l_Address < 0x04*array_len){
		if(DevNVRAM.data32[l_Index] != Read_4byte(l_Address)) l_Error++;
		l_Index = l_Index+1;
		l_Address = l_Address + 4;
	}

	l_Address = 0x00;
	l_Index = 0x00;
	if(l_Error > 0){
		W25qxx_EraseSector(0x0);
		while(l_Address < 0x04*array_len){
			if(Write_4byte(DevNVRAM.data32[l_Index], l_Address)){
				l_Index = l_Index+1;
				l_Address = l_Address + 4;
			}
		}
	}

	return true;
}

/*****************************************************************************************************************/
/*********************************************Reset accumulated dose**********************************************/
/*****************************************************************************************************************/
void Reset_dose(){
	GWORK.rad_dose_old = 0;
	DevNVRAM.GSETTING.rad_sum = 0;
	Write_configuration();
}

/*****************************************************************************************************************/
/*******************************************Reset settings to defaults********************************************/
/*****************************************************************************************************************/
void Reset_settings(){
	W25qxx_EraseSector(0x0);
	Read_configuration();
	Update_rad_buffer();
}

/*****************************************************************************************************************/
/**************************************************Clear log memory***********************************************/
/*****************************************************************************************************************/
void Clear_memory(){
	GFLAGS.log_mutex = true;
	uint32_t current_sector = 0x1;
	do{
		W25qxx_EraseSector(current_sector);
		current_sector++;
	}while(current_sector < 0x1 + (DevNVRAM.GSETTING.w25qxx_address/0x1000));

	DevNVRAM.GSETTING.w25qxx_address = 0x1000;
	DevNVRAM.GSETTING.session_number = 0;
	Write_configuration();
	GFLAGS.log_mutex = false;
}

void Erase_memory(){
	W25qxx_EraseChip();
	Read_configuration();
}

/*****************************************************************************************************************/
/********************************************Reset activity test data*********************************************/
/*****************************************************************************************************************/
void Reset_activity_test(){
	for(uint32_t i = 0; i < MAXIMUM_RAD_BUFEER_LEN; i++) GWORK.rad_buff[i] = 0;
	GFLAGS.is_mean_mode = false;
	GFLAGS.is_particle_mode = false;
	GFLAGS.is_particle_per_sec_mode = false;
	GFLAGS.calculate_dose = false;
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

/*****************************************************************************************************************/
/****************************************Standart deviation calculations******************************************/
/*****************************************************************************************************************/
void Calculate_std(){
	uint64_t _sum = 0;
	for(unsigned i = 0; i < MEAN_MEAS_TIME; i++) _sum+=GWORK.stat_buff[i];
	GMEANING.mean = (float)_sum/MEAN_MEAS_TIME;
	_sum = 0;
	for(unsigned i = 0; i < MEAN_MEAS_TIME; i++) _sum+=pow(GWORK.stat_buff[i] - GMEANING.mean, 2);
	GMEANING.std = (float)_sum/(float)(MEAN_MEAS_TIME-1);
}

/*****************************************************************************************************************/
void transmit_log(){

}

/*****************************************************************************************************************/
void activity_test_timer_ticker(){
	if(!GFLAGS.stop_timer){
		if(GWORK.time_min != 0 && GWORK.time_sec == 0){
			--GWORK.time_min;
			GWORK.time_sec=60;
		}
		if(GWORK.time_sec != 0){ --GWORK.time_sec; }
		GWORK.timer_remain--;
		if(GWORK.timer_remain == 0){
			GFLAGS.stop_timer = true;
			GFLAGS.do_alarm = true;
		}
	}
}

/*****************************************************************************************************************/
void geiger_counter_ticker(){
	HAL_RTC_GetTime(&hrtc, &current_time, RTC_FORMAT_BIN);
	current_hour = current_time.Hours;
	current_minutes = current_time.Minutes;
	current_seconds = current_time.Seconds;
	/*if(current_hour<99) {
		if(++current_seconds>59){
			if(++current_minutes>59){
				if(++current_hour>99) current_hour=99; //часы
					current_minutes=0;
			}
			current_seconds=0;
		}
	}*/
}

/*****************************************************************************************************************/
void sleep(){
	RTC_AlarmTypeDef sAlarm;
	geiger_counter_ticker();
	sAlarm.AlarmTime.Hours = current_hour+1;
	sAlarm.AlarmTime.Minutes = current_minutes;
	sAlarm.AlarmTime.Seconds = current_seconds+10;
	sAlarm.Alarm = RTC_ALARM_A;
	if(HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) == HAL_OK){
		screen_saver_state = true;
		display_power_off();
		pwm_backlight(0);
		GFLAGS.log_mutex = true;
		awake_time = 30;

		HAL_SuspendTick();
		HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		HAL_ResumeTick();
		display_power_on();
		GFLAGS.log_mutex = false;
		screen_saver_state = false;
	}
}

void sleep_ticker(){
	if(--awake_time < 1){
		sleep();
	}
}

/*****************************************************************************************************************/
void update_selected_counter(){
	switch(DevNVRAM.GSETTING.ACTIVE_COUNTERS){
		case 0:
			LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_1);
			LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_2);
			LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_3);
			break;
		case 1:
			LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_1);
			LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_2);
			LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_3);
			break;
		case 2:
			LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_1);
			LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_2);
			LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_3);
			break;
		case 3:
			LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_1);
			LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_2);
			LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_3);
			break;
	}
}


void interrupts_handler(){
	if(GWORK.rad_buff[0]!=65535) GWORK.rad_buff[0]++;
	if(GFLAGS.calculate_dose)if(++DevNVRAM.GSETTING.rad_sum>MAX_PARTICLES*3600/GWORK.real_geigertime) DevNVRAM.GSETTING.rad_sum=MAX_PARTICLES*3600/GWORK.real_geigertime; //общая сумма импульсов
	if(GUI.page == 1 && !GFLAGS.do_alarm){ GFLAGS.is_detected = true; }
	if(GMODE.counter_mode == 1) GWORK.rad_back++;
}
