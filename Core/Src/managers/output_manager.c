/*
 * output_manager.c
 *
 *  Created on: 15 авг. 2020 г.
 *      Author: VeAnIlAn
 */

#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "managers/data_manager.h"
#include "libs/LCD_1202.h"

#include "language.h"
#include "util.h"

extern geiger_ui GUI;
extern geiger_mode GMODE;
extern geiger_meaning GMEANING;
extern geiger_work GWORK;
extern geiger_settings GSETTING;
extern geiger_flags GFLAGS;

#define BAT_ADC_MIN 50
#define BAT_ADC_MAX 200

#define settings_puncts 4
#define asettings_puncts 3

int getNumOfDigits(uint32_t number){
	int digits=1; uint32_t num = number;
	while ((num/=10) > 0) digits++;
	return digits;
}

void draw_update(){
	if(GUI.update_required){
		switch (GUI.page){
			case 0: draw_logo(); GUI.page = 1; break;
			case 1: draw_main(); break;
			case 2: draw_menu(); break;
			case 3: draw_bat_low(); break;
		}
	GUI.update_required  = false;
	}
}

void update_request(){
	GUI.update_required = true;
}

void draw_logo(){

}

void draw_main(){
	LCD_Clear();
	int coeff = mapfloat(GMEANING.current_battery_voltage, BAT_ADC_MIN, BAT_ADC_MAX, 0, 12);             //Значение сдвига пикселей для визуализации заряда аккумулятора
	bool show_battery = false;

	//отрисовка этой части занимает 1.87 кб

	/*if(!datamgr->is_charging && !show_battery){
		display.drawBitmap(69, 0, battery_Bitmap, 15, 7, BLACK);
		display.fillRect(83-coeff, 1, 12, 5, BLACK);
	}
	if(datamgr->is_charging) display.drawBitmap(60, 0, charge_Bitmap, 7, 7, BLACK);

	if(!datamgr->mute && !datamgr->is_charging) display.drawBitmap(0, 0, speaker_Bitmap, 7, 7, BLACK);
	if(!datamgr->no_alarm && !datamgr->is_charging) display.drawBitmap(8, 0, alarm_Bitmap, 7, 7, BLACK);
	if(datamgr->mean_mode && !datamgr->is_charging) display.drawBitmap(17, 0, mean_Bitmap, 7, 7, BLACK);*/

	if(show_battery){
		//display.drawBitmap(17, 12, big_battery_Bitmap, 50, 24, BLACK);
		//LCD_SetCursor(20, 21);
		//LCD_write(mapfloat(datamgr->battery_voltage, BAT_ADC_MIN, BAT_ADC_MAX, 3.6, 4.2));
		//LCD_write("/");
		//LCD_write(map(datamgr->battery_voltage, BAT_ADC_MIN, BAT_ADC_MAX, 0, 100));
	}else if(false){
		//uint8_t progress = map(datamgr->battery_voltage, BAT_ADC_MIN, BAT_ADC_MAX, 0, 42);
		//display.drawBitmap(17, 12, big_battery_Bitmap, 50, 24, BLACK);
		//display.fillRect(19, 14, progress, 20, BLACK);
		//LCD_SetCursor(0, 0);
		//LCD_write("V:");
		//LCD_write(mapfloat(datamgr->battery_voltage, BAT_ADC_MIN, BAT_ADC_MAX, 3.6, 4.2));
		//LCD_SetCursor(0, 38);
		//if(datamgr->is_charged) LCD_write(SUCCESS);
	}else{
		if(GMODE.counter_mode == 0){
			LCD_SetTextColor(COLOR_BLACK, COLOR_WHITE);
			uint16_t deviation = map(100-(GMEANING.mean/(GMEANING.mean+GMEANING.std))*100, 0, 100, GSETTING.GEIGER_ERROR, 100);
			if(deviation > 100) deviation = 100;

			LCD_SetCharSize(2);
			LCD_SetCursor(0, 8);
			if(GWORK.rad_back > 1000) LCD_write((float)GWORK.rad_back/1000, true);
			else if(GWORK.rad_back > 1000000) LCD_write((float)GWORK.rad_back/1000000, true);
			else LCD_write(GWORK.rad_back, false);

			LCD_SetCharSize(0);
			LCD_AddToCursor(0, 3);
			LCD_JustDrawChar(0x60);
			LCD_write(deviation, false);
			LCD_print("%");

			LCD_SetCursor(0, 23);
			if(GWORK.rad_back > 1000) LCD_print(T_MRH);
			else if(GWORK.rad_back > 1000000) LCD_print(T_RH);
			else LCD_print(T_URH);

			LCD_SetCharSize(0);
			LCD_SetCursor(0, 33);
			LCD_write(GWORK.rad_max, false);
			LCD_SetCursor(0, 40);
			LCD_print(T_MAX);

			LCD_DrawFastHLine(0,31,20,COLOR_BLACK);
			LCD_DrawFastHLine(0,48,20,COLOR_BLACK);

			LCD_SetCursor(0, 50);
			if(GWORK.rad_dose > 1000) LCD_write((float)GWORK.rad_dose/1000, true);
			else if(GWORK.rad_dose > 1000000) LCD_write((float)GWORK.rad_dose/1000000, true);
			else LCD_write(GWORK.rad_dose, false);
			if(GWORK.rad_dose > 1000) LCD_print(T_MR);
			else if(GWORK.rad_dose > 1000000) LCD_print(T_R);
			else LCD_print(T_UR);
			LCD_SetCursor(0, 58);
			LCD_print(S_DOSE);

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
			LCD_SetCursor(LCD_X/2 - (getNumOfDigits(GWORK.time_min)+getNumOfDigits(GWORK.time_sec)+1)*3, 23);
			LCD_write(GWORK.time_min, false);
			LCD_print(":");
			LCD_write(GWORK.time_sec, false);
			LCD_DrawFastHLine(0,48, LCD_X, COLOR_BLACK);
			LCD_FillRect(0, 50, map(GWORK.timer_remain, GWORK.timer_time, 0, 0, LCD_X), 16, COLOR_BLACK);
			//LCD_DrawFastHLine(0,LCD_Y-1,LCD_X,COLOR_BLACK);
			LCD_SetTextColor(COLOR_WHITE, COLOR_BLACK);
			LCD_SetCursor(LCD_X - strlen(S_PRESSSET), 54);
			if(GFLAGS.stop_timer && !GFLAGS.next_step) LCD_print(S_PRESSSET);
			LCD_SetCursor(LCD_X - strlen(S_SUCCESS), 54);
			if(GFLAGS.stop_timer && GFLAGS.next_step) LCD_print(S_SUCCESS);
		}else if(GMODE.counter_mode == 2){
			LCD_SetTextColor(COLOR_BLACK, COLOR_WHITE);
			LCD_SetCursor(0, 0);
			LCD_print(S_MODE_SEC);
			LCD_SetTextColor(COLOR_BLACK, COLOR_WHITE);
			LCD_SetCharSize(2);
			LCD_SetCursor(0, 9);
			LCD_write(GWORK.rad_buff[0], false);

			LCD_SetCharSize(0);
			LCD_SetCursor(0, 23);
			LCD_print(T_CPS);

			LCD_SetCursor(0, 33);
			LCD_write(GWORK.sum_old, false);
			LCD_SetCursor(0, 41);
			LCD_print(T_OLD);

			LCD_SetCursor(0, 51);
			LCD_write(GWORK.rad_max, false);
			LCD_SetCursor(0, 59);
			LCD_print(T_MAX);

			LCD_DrawFastHLine(0,31,20,COLOR_BLACK);
			LCD_DrawFastHLine(0,49,20,COLOR_BLACK);

			draw_graph();
		}
	}
	LCD_Update();
}

void draw_simple_menu_page(const char** punct_names, size_t punct_count){
	for(size_t i = 0; i < punct_count; i++){
		LCD_SetCursor(0, 10 + 9*i);
		if (GUI.cursor==i) LCD_print(T_CURSOR);
		LCD_print(punct_names[i]);
	}
}

void draw_editable_menu_page(const char** punct_names, uint8_t* punct_values, char* postfixes, size_t punct_count){
	for(size_t i = 0; i < punct_count; i++){
		LCD_SetCursor(0, 10+9*i);
		if (GUI.cursor==i) LCD_print(T_CURSOR);
		LCD_print(punct_names[i]);
		if(GUI.cursor==i && GFLAGS.is_editing_mode){
			LCD_SetCursor(LCD_X - (getNumOfDigits(GUI.editable)+1)*6, 10+9*i);
			LCD_SetTextColor(COLOR_WHITE, COLOR_BLACK);
			LCD_write(GUI.editable, false);
		}else{
			LCD_SetCursor(LCD_X - (getNumOfDigits(punct_values[i])+1)*6, 10+9*i);
			LCD_write(punct_values[i], false);
		}
		LCD_JustDrawChar(postfixes[i]);
		LCD_SetTextColor(COLOR_BLACK, COLOR_WHITE);
	}
}

void draw_checkbox_menu_page(const char** punct_names, bool* punct_values, bool* skip_flags, size_t punct_count){
	for(size_t i = 0; i < punct_count; i++){
		LCD_SetCursor(0, 10 + 9*i);
		LCD_SetTextColor(COLOR_BLACK, COLOR_WHITE);
		if (GUI.cursor==i) LCD_print(T_CURSOR);
		LCD_print(punct_names[i]);
		if(!skip_flags[i]){
			const char* curflag = punct_values[i] ? S_YES : S_NO;
			LCD_SetCursor(LCD_X - strlen(curflag)*6, 10 + 9*i);
			LCD_print(curflag);
		}
		LCD_SetTextColor(COLOR_BLACK, COLOR_WHITE);
	}
}

void draw_graph(){
	for(uint8_t i=0;i < LCD_X-1;i++){//сделать интерполяцию графика
		uint8_t x_coord = map(i, 0, LCD_X-1, 21, LCD_X-1);
		uint8_t interpolated = (uint8_t)lerp(LCD_Y-1-GUI.mass[i], LCD_Y-1-GUI.mass[i+1], 0.5);
		LCD_DrawLine(x_coord, LCD_Y - 1, x_coord, interpolated, COLOR_BLACK);
	}
}

void draw_menu(){
	const char* current_page_name[PAGES] = {S_MENU, S_MODE, S_SETTINGS, S_RESET, S_ACTIVITY, S_SURE, S_GCOUNTER, S_ADVANCED, S_ABOUT};
	LCD_SetCharSize(0);
	const char *page_name = current_page_name[GUI.menu_page];
	#if defined(RU)
	    size_t header_size = strlen(page_name)/1.5;
	#else
	    size_t header_size = strlen(page_name);
	#endif
	LCD_Clear();
	LCD_SetTextColor(COLOR_WHITE, COLOR_BLACK);
	LCD_FillRect(0, 0, LCD_X, 8, COLOR_BLACK);
	LCD_SetCursor((17 - header_size)*6/2, 0);
	LCD_print(page_name);
	LCD_DrawFastHLine(0,8,LCD_X,COLOR_BLACK);
	LCD_SetCursor(0, 10);
	LCD_SetTextColor(COLOR_BLACK, COLOR_WHITE);

	//отрисовка меню занимает 1.56 кб, без оптимизации.

	switch (GUI.menu_page){
		case 0:{
			const char* current_page_puncts[5] = {S_MODE, S_SETTINGS, S_RESET, S_POFF, S_ABOUT};
			draw_simple_menu_page(current_page_puncts, 5);
		}break;

		case 1:{
			const char* current_page_puncts[3] = {S_BACKGROUND, S_ACTIVITY, S_MODE_SEC};
			draw_simple_menu_page(current_page_puncts, 3);
		}break;
		//Меню настроек
		case 2:{
			const char* current_page_puncts[4] = {S_GCOUNTER, S_ADVANCED, S_TONE, S_BACKLIGHT};
			const bool punct_values[4] = {false, false, GFLAGS.is_muted, GSETTING.LCD_BACKLIGHT};
			const bool skip_flags[4] = {true, true, false, false};
			draw_checkbox_menu_page(current_page_puncts, punct_values, skip_flags, 4);
		}break;
		//Меню выбора режима
		case 3:{
			const char* current_page_puncts[3] = {S_SETTINGS, S_DOSE, S_ALL};
			draw_simple_menu_page(current_page_puncts, 3);
		}break;
		//Меню настройки режима активности
		case 4:{
			const char* current_page_puncts[3] = {S_TIME, S_MEANS, S_BEGIN};
			const uint16_t current_page_values[3] = {GWORK.time_min, GMODE.means_times, GWORK.time_min};
			const char current_page_postfixes[3] = {'m', 't', 'm'};
			draw_editable_menu_page(current_page_puncts, current_page_values, current_page_postfixes, 3);
	        }break;
	        //Меню сна
	        case 5:{
	        	const char* current_page_puncts[2] = {S_YES, S_NO};
	        	draw_simple_menu_page(current_page_puncts, 2);
	        }break;
	        //Кастомные настройки счётчика
	        case 6:{
	        	const char* current_page_puncts[3] = {S_GTIME, S_ERROR, S_VOLTAGE};
	        	const uint8_t current_page_values[3] = {GSETTING.GEIGER_TIME, GSETTING.GEIGER_ERROR, GSETTING.GEIGER_VOLTAGE};
	        	const char current_page_postfixes[3] = {'s', '%', 'V'};
	        	draw_editable_menu_page(current_page_puncts, current_page_values, current_page_postfixes, 3);
	        }break;
	        //Advanced settings
	        case 7:{
	        	const char* current_page_puncts[asettings_puncts] = {S_CONTRAST, S_DOSE_SAVE, S_ALARM};
	        	const uint8_t current_page_values[asettings_puncts] = {GSETTING.LCD_CONTRAST, GSETTING.SAVE_DOSE_INTERVAL, GSETTING.ALARM_THRESHOLD};
	        	const char current_page_postfixes[3] = {' ', ' ', ' '};
	        	draw_editable_menu_page(current_page_puncts, current_page_values, current_page_postfixes, 3);
	        }break;
	        case 8:{
	        	LCD_print("Firmwave: v1.0");
	        	LCD_SetCursor(0, 20);
	        	LCD_print("Device name: SQUICK");
	        	LCD_SetCursor(0, 30);
	        	LCD_print("Device model: 2");
	        	LCD_SetCursor(0, 40);
	        	LCD_print("MPU: STM32F103");
	        	LCD_SetCursor(0, 50);
	        	LCD_print("GUI API: v0.1b");
	        	LCD_SetCursor(0, 60);
	        	LCD_print("Author: AdamFull");
	        	LCD_SetCursor(0, 70);
	        }break;
	    }
	    LCD_Update();
}

void draw_bat_low(){

}
