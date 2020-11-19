#include "managers/power_manager.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_spi.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_cortex.h"
#include "managers/output_manager.h"
#include "managers/meaning_manager.h"
#include "managers/data_manager.h"
#include "main.h"

uint32_t awake_time;
uint8_t sleepy_deepy;

extern RTC_HandleTypeDef hrtc;
extern bool screen_saver_state;
extern RTC_TimeTypeDef current_time;
extern RTC_DateTypeDef current_date;

extern uint8_t current_hour;
extern uint8_t current_minutes;
extern uint8_t current_seconds;

extern geiger_flags GFLAGS;
extern NVRAM DevNVRAM;
extern geiger_ui GUI;
extern geiger_work GWORK;

unsigned long screen_saver_millis = 0;
unsigned long go_to_sleep_millis = 0;
bool screen_saver_state = false;

/*****************************************************************************************************************/
bool seconds_to_time(uint64_t seconds){
	RTC_AlarmTypeDef sAlarm;
	geiger_counter_ticker();
	uint32_t hours, mins, secs;
	hours = (seconds/3600);
	mins = (seconds -(3600*hours))/60;
	secs = (seconds -(3600*hours)-(mins*60));

	sAlarm.AlarmTime.Hours = current_hour + hours;
	sAlarm.AlarmTime.Minutes = current_minutes + mins;
	sAlarm.AlarmTime.Seconds = current_seconds + secs;

	if(sAlarm.AlarmTime.Seconds > 59){
		sAlarm.AlarmTime.Minutes += sAlarm.AlarmTime.Seconds/60;
		sAlarm.AlarmTime.Seconds = sAlarm.AlarmTime.Seconds%60;
	}

	if(sAlarm.AlarmTime.Minutes > 59){
		sAlarm.AlarmTime.Hours += sAlarm.AlarmTime.Minutes/60;
		sAlarm.AlarmTime.Minutes = sAlarm.AlarmTime.Minutes%60;
	}

	if(sAlarm.AlarmTime.Hours > 23){
		sAlarm.AlarmTime.Hours = sAlarm.AlarmTime.Hours%24;
	}

	sAlarm.Alarm = RTC_ALARM_A;

	return (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) == HAL_OK);
}

/*****************************************************************************************************************/
void screen_saver_update(){
	if(!GFLAGS.is_sleep_mode){
		if(screen_saver_state){
			screen_saver_state = false;
			display_power_on();
		}
		screen_saver_millis = GetTick();
		go_to_sleep_millis = GetTick();
	}
}

/*****************************************************************************************************************/
void screen_saver_ticker(){
	if(!screen_saver_state){
		if(GetTick() - screen_saver_millis > SCREEN_SAVER_TIME){
			screen_saver_millis = GetTick();
			screen_saver_state = true;
			display_power_off();
			pwm_backlight(0);
		}
	}

	if(!GFLAGS.is_sleep_mode){
		if(GetTick() - go_to_sleep_millis > DevNVRAM.GSETTING.time_to_sleep*60000){
			go_to_sleep_millis = GetTick();
			enter_to_sleep_mode();
		}
	}
}

/*****************************************************************************************************************/
void disable_all_interrupts(){
	LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_1);
	LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_2);
	LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_3);

	/*LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_1);
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_2);
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_3);*/

	LL_SYSTICK_DisableIT();
	LL_TIM_DisableIT_UPDATE(TIM1);
	LL_TIM_DisableCounter(TIM1);
	LL_TIM_DisableCounter(TIM2);
	LL_SPI_Disable(SPI2);
}

/*****************************************************************************************************************/
void enable_all_interrupts(){
	update_selected_counter();
	LL_SYSTICK_EnableIT();
	LL_TIM_EnableIT_UPDATE(TIM1);
	LL_TIM_EnableCounter(TIM1);
	LL_TIM_EnableCounter(TIM2);
	LL_SPI_Enable(SPI2);
}

/*****************************************************************************************************************/
void disable_all_peripherial(){

}

/*****************************************************************************************************************/
void enable_all_peripherial(){

}

/*****************************************************************************************************************/
void enter_to_sleep_mode(){
	screen_saver_state = true;
	GFLAGS.log_mutex = true;
	GFLAGS.is_sleep_mode = true;
	GFLAGS.no_alarm = true;
	GFLAGS.is_muted = true;
	GUI.page = 1;
	sleepy_deepy = 0;
	sleep();
}

void exit_sleep_mode(){
	GFLAGS.is_sleep_mode = false;
	GFLAGS.no_alarm = false;
	GFLAGS.is_muted = false;
	screen_saver_update();
}

/*****************************************************************************************************************/
void sleep(){
	if(seconds_to_time(DevNVRAM.GSETTING.sleep_time*60)){
		TIM2->CCR1 = 0;
		TIM2->CCR2 = 0;
		TIM2->CCR3 = 0;

		Update_rad_buffer();
		display_power_off();
		disable_all_interrupts();
		disable_all_peripherial();

		awake_time = GWORK.real_geigertime;

		HAL_SuspendTick();
		HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		HAL_ResumeTick();

		enable_all_interrupts();
		enable_all_peripherial();
		display_power_on();
		GFLAGS.log_mutex = false;
	}
}

/*****************************************************************************************************************/
void sleep_ticker(){
	uint8_t buffer[64];
	if(awake_time-- < 1){
		float sleep_accumulated_dose = ((float)(DevNVRAM.GSETTING.sleep_time*60)/(float)GWORK.real_geigertime)*(float)GWORK.rad_back;
		DevNVRAM.GSETTING.rad_sum += (uint32_t)sleep_accumulated_dose;
		Write_configuration();
		memset(buffer, 0, sizeof(buffer));
		sprintf(buffer, "%u,%u,%u,%u,%u,%u\n", DevNVRAM.GSETTING.session_number, GWORK.rad_back, GWORK.rad_dose, current_hour, current_minutes, current_seconds);
		Write_string_w25qxx(buffer);
		sleep();
	}
}

void enter_to_stop_mode(){
	screen_saver_state = true;
	GFLAGS.log_mutex = true;
	GFLAGS.is_stop_mode = true;
	GFLAGS.no_alarm = true;
	GFLAGS.is_muted = true;
	GUI.page = 1;
	sleepy_deepy = 0;
	stop_mode();
}

void exit_stop_mode(){
	GFLAGS.is_stop_mode = false;
	GFLAGS.no_alarm = false;
	GFLAGS.is_muted = false;
	screen_saver_update();
}

void stop_mode(){
	TIM2->CCR1 = 0;
	TIM2->CCR2 = 0;
	TIM2->CCR3 = 0;

	Update_rad_buffer();
	display_power_off();
	disable_all_interrupts();
	disable_all_peripherial();

	awake_time = 5;

	HAL_SuspendTick();
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	SystemClock_Config();
	HAL_ResumeTick();

	enable_all_interrupts();
	enable_all_peripherial();
	display_power_on();
	GFLAGS.log_mutex = false;
}

/*****************************************************************************************************************/
void stop_ticker(){
	if(awake_time-- < 1){
		stop_mode();
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
