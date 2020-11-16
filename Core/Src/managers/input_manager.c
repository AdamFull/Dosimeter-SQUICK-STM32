#include "libs/GyverButton_stm32.h"
#include "managers/data_manager.h"
#include "configuration.h"


GyverButton btn_set;
GyverButton btn_reset;

extern geiger_flags GFLAGS;
extern geiger_meaning GMEANING;
extern geiger_work GWORK;
extern geiger_mode GMODE;
extern NVRAM DevNVRAM;
extern geiger_ui GUI;

extern bool screen_saver_state;

uint8_t submode_cursor = 0;

/*****************************************************************************************************************/
void init_inputs(){
	gbuttonInit(&btn_set, BUTTON_SET_PORT, BUTTON_SET_PIN, HIGH_PULL, NORM_OPEN);
	gbuttonInit(&btn_reset, BUTTON_RESET_PORT, BUTTON_RESET_PIN, HIGH_PULL, NORM_OPEN);
	setClickTimeout(&btn_reset, 100);
	setClickTimeout(&btn_set, 100);
	setTimeout(&btn_reset, 500);
	setTimeout(&btn_set, 500);
}

/*****************************************************************************************************************/
void move_cursor(bool direction, bool editable, bool menu_mode){
#ifndef DEBUG
	//Cursor in editing
	if(editable){
		if(direction){
			if(GUI.menu_page == 4){
				switch (GUI.cursor){
					case 0:{ if(GUI.editable < 30) GUI.editable++; }break;
					case 1:{ if(GUI.editable < 1) GUI.editable++; }break;
				}
			}else if(GUI.menu_page == 6){
				switch (GUI.cursor){
					case 0:{ if(GUI.editable < 450) GUI.editable++; } break;
					case 1:{ if(GUI.editable < 40) GUI.editable+=5; } break;
					case 2:{ if(GUI.editable < 500) GUI.editable+=5; } break;
					case 3:{ if(GUI.editable < 50) GUI.editable++; } break;
					case 4:{ if(GUI.editable < 3) GUI.editable++; } break;
				}
			}else if(GUI.menu_page == 7){
				switch (GUI.cursor){
					case 0:{ if(GUI.editable < 31) GUI.editable++; } break;
					case 1:{ if(GUI.editable < 500) GUI.editable+=5; } break;
					case 2:{ if(GUI.editable < 500) GUI.editable+=10; } break;
					case 3:{ if(GUI.editable < 300) GUI.editable+=5; } break;
				}
			}
		}else{
			if(GUI.menu_page == 4){
				switch (GUI.cursor){
					case 0:{ if(GUI.editable > 1) GUI.editable--; }break;
					case 1:{ if(GUI.editable > 0) GUI.editable--; }break;
				}
			}else if(GUI.menu_page == 6){
				switch (GUI.cursor){
					case 0:{ if(GUI.editable > 5) GUI.editable--; } break;
					case 1:{ if(GUI.editable > 5) GUI.editable-=5; } break;
					case 2:{ if(GUI.editable > 100) GUI.editable-=5; } break;
					case 3:{ if(GUI.editable > 1) GUI.editable--; } break;
					case 4:{ if(GUI.editable > 0) GUI.editable--; } break;
				}
			}else if(GUI.menu_page == 7){
				switch (GUI.cursor){
					case 0:{ if(GUI.editable > 5) GUI.editable--; } break;
					case 1:{ if(GUI.editable > 5) GUI.editable-=5; } break;
					case 2:{ if(GUI.editable > 30) GUI.editable-=10; } break;
					case 3:{ if(GUI.editable > 5) GUI.editable-=5; } break;

				}
			}
		}
		//Cursor in menu
	}else{
		if(direction){
			if(menu_mode && GUI.menu_page != 8){
				switch (GUI.menu_page){
					case 0:{ if(GUI.cursor < 4) GUI.cursor++; } break;
					case 1:{ if(GUI.cursor < 1) GUI.cursor++; } break;
					case 2:{ if(GUI.cursor < 4) GUI.cursor++; } break;
					case 3:{ if(GUI.cursor < 3) GUI.cursor++; } break;
					case 4:{ if(GUI.cursor < 2) GUI.cursor++; } break;
					case 5:{ if(GUI.cursor < 1) GUI.cursor++; } break;
					case 6:{ if(GUI.cursor < 4) GUI.cursor++; } break;
					case 7:{ if(GUI.cursor < 3) GUI.cursor++; } break;
				}
			}

		}else{
			if(menu_mode && GUI.cursor > 0 && GUI.menu_page != 8) { GUI.cursor--; }
		}
	}
	GFLAGS.is_detected = true;
#endif
}

/*****************************************************************************************************************/
void cursor_select(bool direction, bool editable, bool menu_mode){
#ifndef DEBUG
	if(direction){
		if(editable){
			GFLAGS.is_detected = true;
			if(GUI.menu_page == 2){
				switch (GUI.cursor){
				case 1:{ Set_setting(&DevNVRAM.GSETTING.BUZZER_TONE, (uint32_t)GUI.editable); }break;
				case 2:{ Set_setting(&DevNVRAM.GSETTING.LCD_BACKLIGHT, (uint32_t)GUI.editable); }break;
				}
			}else if(GUI.menu_page == 4){
				switch (GUI.cursor){
					case 0:{ GWORK.time_min = GUI.editable; }break;
					case 1:{ GMODE.means_times = GUI.editable; }break;
				}
			}else if(GUI.menu_page == 6){
				switch (GUI.cursor){
					case 0:{ Set_setting(&DevNVRAM.GSETTING.GEIGER_TIME, (uint32_t)GUI.editable); }break;
					case 1:{ Set_setting(&DevNVRAM.GSETTING.GEIGER_ERROR, (uint32_t)GUI.editable); }break;
					case 2:{ Set_setting(&DevNVRAM.GSETTING.GEIGER_VOLTAGE, (uint32_t)GUI.editable); }break;
					case 3:{ Set_setting(&DevNVRAM.GSETTING.sensor_area, (uint32_t)GUI.editable); }break;
					case 4:{ Set_setting(&DevNVRAM.GSETTING.ACTIVE_COUNTERS, (uint32_t)GUI.editable); update_selected_counter(); }break;
				}
			}else if(GUI.menu_page == 7){
				switch (GUI.cursor){
					case 0:{ Set_setting(&DevNVRAM.GSETTING.LCD_CONTRAST, (uint32_t)GUI.editable); }break;
					case 1:{ Set_setting(&DevNVRAM.GSETTING.SAVE_DOSE_INTERVAL, (uint32_t)GUI.editable); }break;
					case 2:{ Set_setting(&DevNVRAM.GSETTING.ALARM_THRESHOLD, (uint32_t)GUI.editable); }break;
					case 3:{ Set_setting(&DevNVRAM.GSETTING.log_save_period, (uint32_t)GUI.editable); }break;
				}
			}
			GFLAGS.is_editing_mode = false;
			return;
		}else{
			GFLAGS.is_detected = true;
			switch (GUI.menu_page){
				case 0:{
					switch (GUI.cursor){
						case 0:{ GUI.menu_page = 1; }break;
						case 1:{ GUI.menu_page = 2; }break;
						case 2:{ GUI.menu_page = 3; }break;
						case 3:{ GUI.menu_page = 5; }break;
						case 4:{ GUI.menu_page = 8; }break;
					}
					GUI.cursor = 0;
				}break;
				case 1:{
					switch (GUI.cursor){
						case 0:{ GMODE.counter_mode = 0; GUI.page = 1; }break;
						case 1:{ GUI.menu_page = 4; }break;
					}
					GUI.cursor = 0;
				}break;
				case 2:{
					switch (GUI.cursor){
						case 0:{ GUI.menu_page = 6; }break;
						case 1:{ GUI.menu_page = 7; }break;
						case 2:{ GFLAGS.is_muted = !GFLAGS.is_muted; }break;
						case 3:{ DevNVRAM.GSETTING.LCD_BACKLIGHT = !DevNVRAM.GSETTING.LCD_BACKLIGHT; }break;
						case 4:{ GFLAGS.is_tracking_enabled = !GFLAGS.is_tracking_enabled; }break;
					}
					return;
				}break;
				case 3:{
					switch (GUI.cursor){								//Стереть данные
						case 0:{ Reset_settings(); GUI.menu_page = 0; }break;
						case 1:{ Reset_dose(); GUI.menu_page = 0; }break;
						case 2:{ Clear_memory(); GUI.menu_page = 0; }break;
						case 3:{ Erase_memory(); GUI.menu_page = 0; }break;
					}
					GUI.cursor = 0;
				}break;
				case 4:{
					switch (GUI.cursor){
						case 0:{ GUI.editable = GWORK.time_min; }break;
						case 1:{ GUI.editable = GMODE.means_times; }break;
						case 2:{
							Reset_activity_test();
							GWORK.time_min_old = GWORK.time_min;
							GWORK.timer_time = GWORK.time_min * 60;
							GWORK.timer_remain = GWORK.timer_time;
						}break;
					}
					if(GUI.cursor != 2) GFLAGS.is_editing_mode = true;

				}break;
				case 5:{
					switch (GUI.cursor){								//Вообще это диалог выбора, но пока что это не он
						case 0:{
							sleep();
						}break;
						case 1:{ GUI.menu_page = 0; }break;
					}
					GUI.cursor = 0;
					}break;
					case 6:{
						switch (GUI.cursor){
							case 0:{ GUI.editable = DevNVRAM.GSETTING.GEIGER_TIME; }break;
							case 1:{ GUI.editable = DevNVRAM.GSETTING.GEIGER_ERROR; }break;
							case 2:{ GUI.editable = DevNVRAM.GSETTING.GEIGER_VOLTAGE; }break;
							case 3:{ GUI.editable = DevNVRAM.GSETTING.sensor_area; }break;
							case 4:{ GUI.editable = DevNVRAM.GSETTING.ACTIVE_COUNTERS; }break;
						}
						GFLAGS.is_editing_mode = true;
					}break;
					case 7:{
						switch(GUI.cursor){
							case 0:{ GUI.editable = DevNVRAM.GSETTING.LCD_CONTRAST; }break;
							case 1:{ GUI.editable = DevNVRAM.GSETTING.SAVE_DOSE_INTERVAL; }break;
							case 2:{ GUI.editable = DevNVRAM.GSETTING.ALARM_THRESHOLD; }break;
							case 3:{ GUI.editable = DevNVRAM.GSETTING.log_save_period; }break;
						}
						GFLAGS.is_editing_mode = true;
					}
				}
			}
		}
#endif
}

/*****************************************************************************************************************/
void button_action(){
#ifndef DEBUG
	tick(&btn_reset);
	tick(&btn_set);


	bool btn_reset_isHolded = isHolded(&btn_reset);
	bool btn_set_isHolded = isHolded(&btn_set);

	bool menu_mode = GUI.page == 2;

	if(isHold(&btn_reset) && isHold(&btn_set)){
		battery_safe_update();
		if(!menu_mode){
			GUI.page = 2;
			GUI.menu_page = 0;
			GFLAGS.is_editing_mode = false;
		}else{
			GFLAGS.is_editing_mode = false;
			GUI.page = 1;
		}
		update_request();
		resetStates(&btn_reset);
		resetStates(&btn_set);
	}else if(isHold(&btn_set) && !menu_mode){
		//if(!menu_mode && !isPress(btn_reset)) battery_request(true);
	}else if(btn_reset_isHolded){											//Удержание кнопки ресет
		battery_safe_update();
		if(!menu_mode && !isPress(&btn_set)) GFLAGS.no_alarm = !GFLAGS.no_alarm;
		if(menu_mode && !GFLAGS.is_editing_mode){										//Если находимся в меню
			GFLAGS.is_detected = true;
			if(GUI.menu_page == 0) {GUI.page = 1; GFLAGS.do_alarm = false;}
			else if(GUI.menu_page == 2) { Accept_settings(); GUI.menu_page = 0; }
			else if(GUI.menu_page == 6) GUI.menu_page = 2;
			else if(GUI.menu_page == 7) GUI.menu_page = 2;
			else GUI.menu_page = 0;
			GUI.cursor = 0;
		}
		if(GFLAGS.is_editing_mode){
			GFLAGS.is_detected = true;
			GFLAGS.is_editing_mode = false;
		}
		if(!menu_mode && GMODE.counter_mode == 1){
			Reset_activity_test();
			GWORK.timer_remain = GWORK.timer_time;
			GWORK.time_min = GWORK.time_min_old;
		}
		update_request();
	}else if(isClick(&btn_reset) && !btn_reset_isHolded){					//Клик кнопки ресет
		if(!screen_saver_state){
			if(!menu_mode && !GFLAGS.do_alarm) GFLAGS.is_muted = !GFLAGS.is_muted;
			if(!menu_mode && GFLAGS.do_alarm) GFLAGS.no_alarm = !GFLAGS.no_alarm;
			move_cursor(false, GFLAGS.is_editing_mode, menu_mode);
		}
		battery_safe_update();
		update_request();
	}else if(btn_set_isHolded){												//Удержание кнопки сет
		battery_safe_update();
		cursor_select(true, GFLAGS.is_editing_mode, menu_mode);
		update_request();
	}else if(isClick(&btn_set) && !btn_set_isHolded){					//Клик кнопки сет
		if(!screen_saver_state){
			if(!menu_mode && !GFLAGS.do_alarm){
				if(++submode_cursor > 3) submode_cursor = 0;
				GWORK.rad_max = 0;
				for(uint32_t i = 0; i < MAXIMUM_RAD_BUFEER_LEN; i++) GWORK.rad_buff[i] = 0;
			}
			if(!menu_mode && GMODE.counter_mode == 1 && !GFLAGS.next_step && GFLAGS.stop_timer){
				GWORK.rad_max = GWORK.rad_back;
				GWORK.rad_back = 0;
				GFLAGS.next_step = true;
				GFLAGS.stop_timer = false;
				GFLAGS.do_alarm = false;
				GWORK.time_min = GWORK.time_min_old;
				GWORK.timer_remain = GWORK.timer_time;
				GWORK.time_sec = 0;
			}
			if(!menu_mode && GMODE.counter_mode == 1 && GFLAGS.do_alarm && GFLAGS.next_step && GFLAGS.stop_timer){
				GFLAGS.do_alarm = false;
			}

			if(!menu_mode && GMODE.counter_mode == 0){
				GFLAGS.is_mean_mode = !GFLAGS.is_mean_mode;
			}

			move_cursor(true, GFLAGS.is_editing_mode, menu_mode);
		}
		battery_safe_update();
		update_request();
	}
#endif
}
