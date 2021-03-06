/*
 * output_manager.c
 *
 *  Created on: 15 авг. 2020 г.
 *      Author: VeAnIlAn
 */

#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "managers/output_manager.h"
#include "managers/data_manager.h"
#include "stm32f1xx_ll_usart.h"

#include "language.h"
#include "libs/GFX_font.h"
#include "util.h"

#include "libs/w25qxx.h"

#include "configuration.h"

extern geiger_ui GUI;
extern geiger_mode GMODE;
extern geiger_meaning GMEANING;
extern geiger_work GWORK;
extern NVRAM DevNVRAM;
extern geiger_flags GFLAGS;

extern uint8_t submode_cursor;
extern uint8_t current_hour;
extern uint8_t current_minutes;

unsigned long beep_timer = 0;
LCD_CONFIG lcd_config_g;

uint8_t strbuffer[64] = {0};
char rx_buffer[32];

void draw_simple_menu_page(const char**, uint32_t);
void draw_editable_menu_page(const char**, int16_t*, char*, uint32_t);
void draw_checkbox_menu_page(const char**, bool*, bool*, uint32_t);

/*****************************************************************************************************************/
void init_outputs(){
	lcd_config_g.MOSIPORT = DISPLAY_MOSI_PORT;
	lcd_config_g.MOSIPIN = DISPLAY_MOSI_PIN;
	lcd_config_g.SCKPORT = DISPLAY_SCK_PORT;
	lcd_config_g.SCKPIN = DISPLAY_SCK_PIN;
	lcd_config_g.CSPORT = DISPLAY_CS_PORT;
	lcd_config_g.CSPIN = DISPLAY_CS_PIN;
	lcd_config_g.RESPORT = DISPLAY_RST_PORT;
	lcd_config_g.RESPIN = DISPLAY_RST_PIN;

	LCD_Init(lcd_config_g);
	LCD_SetContrast(10);
	LCD_Flip();
}

/*****************************************************************************************************************/
void send_report(){
#ifdef BLUETOOTH_SUPPORT
	char buffer[32] = {0};
	sprintf(buffer, "%u,%u,%u\n", GWORK.rad_back, GWORK.rad_max, GWORK.rad_dose);
	for(uint32_t i = 0; i < strlen(buffer); i++){
		*(volatile uint8_t *)&USART1->DR = (uint8_t)buffer[i];
	}
#elif defined(BLE_SUPPORT)

#endif
}

/*****************************************************************************************************************/
int getNumOfDigits(uint32_t number){
	int digits=1; uint32_t num = number;
	while ((num/=10) > 0) digits++;
	return digits;
}

/*****************************************************************************************************************/
void display_power_off(){
	LCD_Clear();
	LCD_Update();
	LCD_PowerOn(false);
}

void display_power_on(){
	LCD_PowerOn(true);
}

/*****************************************************************************************************************/
void beep() { //индикация каждой частички звуком светом
    if(!GFLAGS.is_muted && !GFLAGS.do_alarm){
        if(GetTick() - beep_timer > 1){
            beep_timer = GetTick();
            if(GFLAGS.is_detected){
            	pwm_tone(50);
            	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
            }else{
            	pwm_tone(0);
            	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
            }
            GFLAGS.is_detected = false;
        }
    }
}

/*****************************************************************************************************************/
void SendMessage(char *message){
	CDC_Transmit_FS(message, strlen(message));
}

/*****************************************************************************************************************/
void uart_transmition_handler(){
	if(strlen(rx_buffer) > 0) {																//If cbc received string, check what is it
		if(strcmp(rx_buffer, "rdlog") == 0 && !GFLAGS.log_mutex){						//If it was rdlog, start transmit log from flash
			GFLAGS.log_mutex = true;
			GFLAGS.is_monitor_enabled = false;
			uint32_t l_Address = 0x00001000;
			uint32_t l_Index = 0;
			char to_append = '\0';
			sprintf(strbuffer, "%d\n", DevNVRAM.GSETTING.w25qxx_address - l_Address);
			strncat(strbuffer, &to_append, sizeof(char));
			CDC_Transmit_FS(strbuffer, strlen(strbuffer));
			LL_mDelay(100);
			while(l_Address < DevNVRAM.GSETTING.w25qxx_address){
				l_Index = 0;
				memset(strbuffer, 0, sizeof(strbuffer));
				while(strbuffer[abs(l_Index-1)] != '\n'){
					W25qxx_ReadByte(&strbuffer[l_Index], l_Address);
					l_Address++; l_Index++;
				}
				strncat(strbuffer, &to_append, sizeof(char));
				if(strbuffer != NULL) CDC_Transmit_FS(strbuffer, strlen(strbuffer));
				LL_mDelay(TRANSMITION_DELAY);
			}
			LL_mDelay(TRANSMITION_DELAY);
			SendMessage("done\r\n");
			Clear_memory();
			GFLAGS.log_mutex = false;
			}else if(strcmp(rx_buffer, "clmem\n") == 0){											//If received clmem, start cleaning flash
				SendMessage("Erasing chip.\r\n");
				Erase_memory();
				SendMessage("done\0");
				LL_mDelay(TRANSMITION_DELAY);
				SendMessage("Please reconnect device to computer.\r\n");
			}else if(strcmp(rx_buffer, "monitor\n") == 0){										//If received monitor, let's enable work log transfer
				GFLAGS.is_monitor_enabled = !GFLAGS.is_monitor_enabled;
			}else if(strcmp(rx_buffer, "rdcfg\n") == 0){
				//Read settings
			}
		memset(rx_buffer, 0, sizeof(rx_buffer));
	}
}

/*****************************************************************************************************************/
void draw_update(){
	if(GUI.update_required && !GFLAGS.is_sleep_mode){
		switch (GUI.page){
			case 0: draw_logo(); GUI.page = 1; break;
			case 1: draw_main(); break;
			case 2: draw_menu(); break;
			case 3: draw_bat_low(); break;
		}
	GUI.update_required  = false;
	}
}

/*****************************************************************************************************************/
void update_request(){
	GUI.update_required = true;
}

/*****************************************************************************************************************/
void draw_logo(){

}

/*****************************************************************************************************************/
void draw_statusbar(const char** bitmaps, bool* bitmap_enabled, uint32_t size){
	uint32_t cur_index = 0;
	for(uint32_t i = 0; i < size; i++){
		if(bitmap_enabled[i]){
			LCD_DrawBitmap(LCD_X_SIZE-9, 9+9*cur_index, bitmaps[i], 8, 8, COLOR_BLACK);
			cur_index++;
		}
	}
}

/*****************************************************************************************************************/
void draw_main(){

	LCD_Clear();
	int coeff = map(GMEANING.current_battery_voltage, BAT_ADC_MIN, BAT_ADC_MAX, 0, 8);             //Значение сдвига пикселей для визуализации заряда аккумулятора
	bool show_battery = false;

	//отрисовка этой части занимает 1.87 кб

	if(!GFLAGS.is_charging && !show_battery){
		if(!GFLAGS.is_charging){
			LCD_DrawBitmap(LCD_X_SIZE-11, 1, battery_bitmap, 10, 8, COLOR_BLACK);
			if(GMEANING.current_battery_voltage >= BAT_ADC_MIN) LCD_FillRect(LCD_X_SIZE-10, 3, coeff, 4, COLOR_BLACK);
		}else{
			LCD_DrawBitmap(LCD_X_SIZE-11, 1, charge_bitmap, 10, 8, COLOR_BLACK);
		}
	}

	const char* bitmap_array[] = {unmuted_bitmap, bell_bitmap, backlight_bitmap, satellite_bitmap, death_bitmap};
	bool bitmap_status[] = {!GFLAGS.is_muted, !GFLAGS.no_alarm, (bool)(DevNVRAM.GSETTING.LCD_BACKLIGHT), GFLAGS.is_tracking_enabled, GFLAGS.do_alarm };
	draw_statusbar(bitmap_array, bitmap_status, 5);

	if(GMODE.counter_mode == 0){
		LCD_SetCursor(0, 0);
		switch(submode_cursor){
		case 0: LCD_print("BKG"); break;
		case 1: LCD_print("PCM"); break;
		case 2: LCD_print("MN"); break;
		case 3: LCD_print("CPS"); break;
		}

		LCD_SetCursor(LCD_X_SIZE/2 - 5*2.5 , 0);
		if(current_hour < 10) LCD_JustDrawChar('0');
		LCD_write(current_hour, false);
		LCD_JustDrawChar(':');
		if(current_minutes < 10) LCD_JustDrawChar('0');
		LCD_write(current_minutes, false);

		LCD_SetTextColor(COLOR_BLACK, COLOR_WHITE);
		uint16_t deviation = map(100-(GMEANING.mean/(GMEANING.mean+GMEANING.std))*100, 0, 100, DevNVRAM.GSETTING.GEIGER_ERROR, 100);
		if(deviation > 100) deviation = 100;

		LCD_SetCharSize(2);
		LCD_SetCursor(0, 8);
		if(!GFLAGS.is_particle_mode && !GFLAGS.is_particle_per_sec_mode){
			if(GFLAGS.is_mean_mode){
				if(GMEANING.mean > 1000) LCD_write(GMEANING.mean/1000, true);
				else if(GMEANING.mean > 1000000) LCD_write(GMEANING.mean/1000000, true);
				else LCD_write(GMEANING.mean, false);
				LCD_SetCharSize(0);
			}else{
				if(GWORK.rad_back > 1000) LCD_write((float)GWORK.rad_back/1000, true);
				else if(GWORK.rad_back > 1000000) LCD_write((float)GWORK.rad_back/1000000, true);
				else LCD_write(GWORK.rad_back, false);
				 //LCD_write(GMEANING.current_high_voltage, false);
				//4.01+3

				LCD_SetCharSize(0);
				LCD_AddToCursor(0, 3);
				LCD_JustDrawChar(0x60);
				LCD_write(deviation, false);
				LCD_print("%");
			}

			LCD_SetCursor(0, 23);
			if(GWORK.rad_back > 1000) LCD_print(T_MRH);
			else if(GWORK.rad_back > 1000000) LCD_print(T_RH);
			else LCD_print(T_URH);
		}else{
			LCD_write(GWORK.rad_back, false);
			LCD_SetCharSize(0);
			LCD_SetCursor(0, 23);
			if(GFLAGS.is_particle_per_sec_mode) LCD_print(T_CPS);
			else LCD_print(T_PCCM2);
		}

		LCD_SetCharSize(0);
		LCD_SetCursor(0, 33);
		LCD_write(GWORK.rad_max, false);
		//LCD_write(GWORK.voltage_req, false);
		LCD_SetCursor(0, 40);
		LCD_print(T_MAX);

		LCD_DrawFastHLine(0,31,20,COLOR_BLACK);
		LCD_DrawFastHLine(0,48,20,COLOR_BLACK);

		if(!GFLAGS.is_particle_per_sec_mode){
			LCD_SetCursor(0, 50);
			if(GWORK.rad_dose > 1000) LCD_write((float)GWORK.rad_dose/1000, true);
			else if(GWORK.rad_dose > 1000000) LCD_write((float)GWORK.rad_dose/1000000, true);
			else LCD_write(GWORK.rad_dose, false);
			//LCD_write(GWORK.transformer_pwm, false);

			LCD_SetCursor(0, 58);
			if(GWORK.rad_dose > 1000) LCD_print(T_MR);
			else if(GWORK.rad_dose > 1000000) LCD_print(T_R);
			else LCD_print(T_UR);
		}else{
			LCD_SetCursor(0, 50);
			LCD_write(GWORK.rad_back_old, false);
			LCD_SetCursor(0, 58);
			LCD_print(T_OLD);
		}

		draw_graph();
	}else if(GMODE.counter_mode == 1){
		LCD_SetTextColor(COLOR_BLACK, COLOR_WHITE);
		LCD_SetCursor(0, 0);
		if(!GFLAGS.next_step) LCD_print(S_BACKGROUND);
		else LCD_print(S_SAMPLE);
		LCD_SetCharSize(2);
		LCD_SetCursor(0, 8);
		if(GFLAGS.stop_timer && GFLAGS.next_step) LCD_write(abs((int)GWORK.rad_max - (int)GWORK.rad_back), false);
		else LCD_write(GWORK.rad_back, false);
		LCD_SetCharSize(0);
		LCD_SetCursor((LCD_X_SIZE/2) - (getNumOfDigits(GWORK.time_min)+getNumOfDigits(GWORK.time_sec)+1)*3, 40);
		LCD_write(GWORK.time_min, false);
		LCD_print(":");
		LCD_write(GWORK.time_sec, false);
		LCD_FillRect(0, 50, map(GWORK.timer_remain, GWORK.timer_time, 0, 0, LCD_X_SIZE), 16, COLOR_BLACK);
		LCD_SetTextColor(COLOR_WHITE, COLOR_BLACK);
		LCD_SetCursor((LCD_X_SIZE/2) - strlen(S_PRESSSET)*2.5, 54);
		if(GFLAGS.stop_timer && !GFLAGS.next_step) LCD_print(S_PRESSSET);
		LCD_SetCursor((LCD_X_SIZE/2) - strlen(S_SUCCESS)*2.5, 54);
		if(GFLAGS.stop_timer && GFLAGS.next_step) LCD_print(S_SUCCESS);
	}
	LCD_Update();
}

/*****************************************************************************************************************/
void draw_simple_menu_page(const char** punct_names, uint32_t punct_count){
	for(uint32_t i = 0; i < punct_count; i++){
		LCD_SetCursor(0, 10 + 9*i);
		if (GUI.cursor==i) LCD_print(T_CURSOR);
		LCD_print(punct_names[i]);
	}
}

/*****************************************************************************************************************/
void draw_editable_menu_page(const char** punct_names, int16_t* punct_values, char* postfixes, uint32_t punct_count){
	for(uint32_t i = 0; i < punct_count; i++){
		LCD_SetCursor(0, 10+9*i);
		if (GUI.cursor==i) LCD_print(T_CURSOR);
		LCD_print(punct_names[i]);
		if(GUI.cursor==i && GFLAGS.is_editing_mode){
			LCD_SetCursor(LCD_X_SIZE - (getNumOfDigits(GUI.editable)+1)*6, 10+9*i);
			LCD_SetTextColor(COLOR_WHITE, COLOR_BLACK);
			LCD_write(GUI.editable, false);
		}else{
			LCD_SetCursor(LCD_X_SIZE - (getNumOfDigits(punct_values[i])+1)*6, 10+9*i);
			LCD_write(punct_values[i], false);
		}
		LCD_JustDrawChar(postfixes[i]);
		LCD_SetTextColor(COLOR_BLACK, COLOR_WHITE);
	}
}

/*****************************************************************************************************************/
void draw_checkbox_menu_page(const char** punct_names, bool* punct_values, bool* skip_flags, uint32_t punct_count){
	for(uint32_t i = 0; i < punct_count; i++){
		LCD_SetCursor(0, 10 + 9*i);
		LCD_SetTextColor(COLOR_BLACK, COLOR_WHITE);
		if (GUI.cursor==i) LCD_print(T_CURSOR);
		LCD_print(punct_names[i]);
		if(!skip_flags[i]){
			const char* curflag = punct_values[i] ? S_YES : S_NO;
			LCD_SetCursor(LCD_X_SIZE - strlen(curflag)*6, 10 + 9*i);
			LCD_print(curflag);
		}
		LCD_SetTextColor(COLOR_BLACK, COLOR_WHITE);
	}
}

/*****************************************************************************************************************/
void draw_graph(){
	for(uint8_t i=0;i < LCD_X_SIZE-1;i++){
		uint8_t x_coord = map(i, 0, LCD_X_SIZE-1, 21, LCD_X_SIZE-1);
		uint8_t interpolated = (uint8_t)lerp(LCD_Y-1-GUI.mass[i], LCD_Y-1-GUI.mass[i+1], 0.5);
		LCD_DrawLine(x_coord, LCD_Y - 1, x_coord, interpolated, COLOR_BLACK);
	}
}

/*****************************************************************************************************************/
void draw_menu(){
#ifndef DEBUG
	const char* current_page_name[PAGES] = {S_MENU, S_MODE, S_SETTINGS, S_RESET, S_ACTIVITY, S_PMAN, S_GCOUNTER, S_ADVANCED, S_ABOUT};
	LCD_SetCharSize(0);
	const char *page_name = current_page_name[GUI.menu_page];
	#if defined(LANGUAGE_RU)
	    uint32_t header_size = strlen(page_name)/1.5;
	#else
	    uint32_t header_size = strlen(page_name);
	#endif
	LCD_Clear();
	LCD_SetTextColor(COLOR_WHITE, COLOR_BLACK);
	LCD_FillRect(0, 0, LCD_X_SIZE, 8, COLOR_BLACK);
	LCD_SetCursor((17 - header_size)*6/2, 0);
	LCD_print(page_name);
	LCD_DrawFastHLine(0,8,LCD_X_SIZE,COLOR_BLACK);
	LCD_SetCursor(0, 10);
	LCD_SetTextColor(COLOR_BLACK, COLOR_WHITE);

	//отрисовка меню занимает 1.56 кб, без оптимизации.

	switch (GUI.menu_page){
		case 0:{
			const char* current_page_puncts[] = {S_MODE, S_SETTINGS, S_RESET, S_PMAN, S_ABOUT};
			draw_simple_menu_page(current_page_puncts, 5);
		}break;

		case 1:{
			const char* current_page_puncts[] = {S_BACKGROUND, S_ACTIVITY};
			draw_simple_menu_page(current_page_puncts, 2);
		}break;
		//Меню настроек
		case 2:{
			const char* current_page_puncts[] = {S_GCOUNTER, S_ADVANCED, S_TONE, S_BACKLIGHT, S_GPS};
			const bool punct_values[] = {false, false, !GFLAGS.is_muted, (bool)DevNVRAM.GSETTING.LCD_BACKLIGHT, GFLAGS.is_tracking_enabled};
			const bool skip_flags[] = {true, true, false, false, false};
			draw_checkbox_menu_page(current_page_puncts, punct_values, skip_flags, 5);
		}break;
		//Меню выбора режима
		case 3:{
			const char* current_page_puncts[] = {S_SETTINGS, S_DOSE, S_FLASH, S_ALL};
			draw_simple_menu_page(current_page_puncts, 4);
		}break;
		//Меню настройки режима активности
		case 4:{
			const char* current_page_puncts[] = {S_TIME, S_MEANS, S_BEGIN};
			const uint16_t current_page_values[] = {GWORK.time_min, GMODE.means_times, GWORK.time_min};
			const char current_page_postfixes[] = {'m', 't', 'm'};
			draw_editable_menu_page(current_page_puncts, current_page_values, current_page_postfixes, 3);
	        }break;
	        //Меню сна
	        case 5:{
	        	const char* current_page_puncts[2] = {S_SLEEP, S_POFF};
	        	draw_simple_menu_page(current_page_puncts, 2);
	        }break;
	        //Кастомные настройки счётчика
	        case 6:{
	        	const char* current_page_puncts[] = {S_GTIME, S_ERROR, S_VOLTAGE, S_SENSOR_AREA, S_GEIGER_MODE};
	        	const int16_t current_page_values[] = {DevNVRAM.GSETTING.GEIGER_TIME, DevNVRAM.GSETTING.GEIGER_ERROR, DevNVRAM.GSETTING.GEIGER_VOLTAGE, DevNVRAM.GSETTING.sensor_area, DevNVRAM.GSETTING.ACTIVE_COUNTERS};
	        	const char current_page_postfixes[] = {'s', '%', 'V', 'c', ' '};
	        	draw_editable_menu_page(current_page_puncts, current_page_values, current_page_postfixes, 5);
	        }break;
	        //Advanced settings
	        case 7:{
	        	const char* current_page_puncts[] = {S_CONTRAST, S_DOSE_SAVE, S_ALARM, S_TRACKING_PERIOD, S_SLEEPT, S_TTSLEEP};
	        	const uint16_t current_page_values[] = {DevNVRAM.GSETTING.LCD_CONTRAST, DevNVRAM.GSETTING.SAVE_DOSE_INTERVAL, DevNVRAM.GSETTING.ALARM_THRESHOLD, DevNVRAM.GSETTING.log_save_period, DevNVRAM.GSETTING.sleep_time, DevNVRAM.GSETTING.time_to_sleep};
	        	const char current_page_postfixes[] = {' ', ' ', ' ', 's', 'm', 'm'};
	        	draw_editable_menu_page(current_page_puncts, current_page_values, current_page_postfixes, 6);
	        }break;
	        case 8:{
	        	uint8_t ext_mem = map((w25qxx.CapacityInKiloByte*1024) - DevNVRAM.GSETTING.w25qxx_address, 0, (w25qxx.CapacityInKiloByte*1024), 0, 100);
	        	LCD_print("Ext mem free:");
	        	LCD_SetCursor(LCD_X_SIZE - (getNumOfDigits(ext_mem)+1)*5, 9);
	        	LCD_write(ext_mem, false);
	        	LCD_JustDrawChar('%');
	        	LCD_SetCursor(0, 18);
	        	LCD_print("Device ram free:");
	        	ext_mem = map(GetRamFree(), 0, 20*1024, 0, 100);
	        	LCD_SetCursor(LCD_X_SIZE - (getNumOfDigits(ext_mem)+1)*5, 18);
	        	LCD_write(ext_mem, false);
	        	LCD_JustDrawChar('%');
	        	LCD_SetCursor(0, 27);
	        	LCD_print("Device rom free:");
	        	ext_mem = map(GetRomFree(), 0, 64*1024, 0, 100);
	        	LCD_SetCursor(LCD_X_SIZE - (getNumOfDigits(ext_mem)+1)*5, 27);
	        	LCD_write(ext_mem, false);
	        	LCD_JustDrawChar('%');
	        	LCD_SetCursor(0, 36);
	        	LCD_print("Build date: ");
	        	LCD_SetCursor(0, 45);
	        	LCD_print(__DATE__);
	        	LCD_SetCursor(0, 54);
	        	LCD_print("Session:");
	        	LCD_write(DevNVRAM.GSETTING.session_number, false);
	        }break;
	    }
	    LCD_Update();
#endif
}

/*****************************************************************************************************************/
void draw_bat_low(){

}
