/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "configuration.h"
#include "stdio.h"
#include "util.h"
#include <managers/meaning_manager.h>
#include "managers/output_manager.h"
#include "managers/data_manager.h"
#include "libs/GyverButton_stm32.h"
#include "libs/LCD_1202.h"
#include "libs/GPS.h"
#include "libs/w25qxx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//typedef void (*pFunction)(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char rx_buffer[32];
unsigned long current_millis;
unsigned long gps_millis;

GyverButton btn_set;
GyverButton btn_reset;

extern DINITSTATUS device_status;

extern geiger_flags GFLAGS;
extern geiger_meaning GMEANING;
extern geiger_work GWORK;
extern geiger_mode GMODE;
extern NVRAM DevNVRAM;
extern geiger_ui GUI;

//pFunction JumpToApplication;
//uint32_t JumpAddress;

LCD_CONFIG lcd_config_g;

uint8_t strbuffer[64] = {0};

extern uint8_t current_hour;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void SendMessage(char *message);
void move_cursor(bool direction, bool editable, bool menu_mode);
void cursor_select(bool direction, bool editable, bool menu_mode);
void button_action();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void SendMessage(char *message){
	CDC_Transmit_FS(message, strlen(message));
}

void bootToDFU(){
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_7);
	LL_mDelay(10);
	NVIC_SystemReset();
}

//Cursor movement
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
					case 0:{ if(GUI.editable < 250) GUI.editable++; } break;
					case 1:{ if(GUI.editable < 40) GUI.editable+=5; } break;
					case 2:{ if(GUI.editable < 500) GUI.editable+=10; } break;
					case 3:{ if(GUI.editable < 3) GUI.editable++; } break;
				}
			}else if(GUI.menu_page == 7){
				switch (GUI.cursor){
					case 0:{ if(GUI.editable < 31) GUI.editable++; } break;
					case 1:{ if(GUI.editable < 500) GUI.editable+=5; } break;
					case 2:{ if(GUI.editable < 500) GUI.editable+=10; } break;
					case 3:{ if(GUI.editable < 12) GUI.editable+=1; } break;
					case 4:{ if(GUI.editable < 300) GUI.editable+=30; } break;
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
					case 2:{ if(GUI.editable > 100) GUI.editable-=10; } break;
					case 3:{ if(GUI.editable > 0) GUI.editable--; } break;
				}
			}else if(GUI.menu_page == 7){
				switch (GUI.cursor){
					case 0:{ if(GUI.editable > 5) GUI.editable--; } break;
					case 1:{ if(GUI.editable > 5) GUI.editable-=5; } break;
					case 2:{ if(GUI.editable > 30) GUI.editable-=10; } break;
					case 3:{ if(GUI.editable > -12) GUI.editable-=1; } break;
					case 4:{ if(GUI.editable > 30) GUI.editable-=30; } break;
				}
			}
		}
		//Cursor in menu
	}else{
		if(direction){
			if(menu_mode && GUI.menu_page != 8){
				switch (GUI.menu_page){
					case 0:{ if(GUI.cursor < 5) GUI.cursor++; } break;
					case 1:{ if(GUI.cursor < 2) GUI.cursor++; } break;
					case 2:{ if(GUI.cursor < 4) GUI.cursor++; } break;
					case 3:{ if(GUI.cursor < 2) GUI.cursor++; } break;
					case 4:{ if(GUI.cursor < 2) GUI.cursor++; } break;
					case 5:{ if(GUI.cursor < 1) GUI.cursor++; } break;
					case 6:{ if(GUI.cursor < 3) GUI.cursor++; } break;
					case 7:{ if(GUI.cursor < 4) GUI.cursor++; } break;
				}
			}

		}else{
			if(menu_mode && GUI.cursor > 0 && GUI.menu_page != 8) { GUI.cursor--; }
		}
	}
	GFLAGS.is_detected = true;
#endif
}


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
					case 3:{ Set_setting(&DevNVRAM.GSETTING.ACTIVE_COUNTERS, (uint32_t)GUI.editable); }break;
				}
			}else if(GUI.menu_page == 7){
				switch (GUI.cursor){
					case 0:{ Set_setting(&DevNVRAM.GSETTING.LCD_CONTRAST, (uint32_t)GUI.editable); }break;
					case 1:{ Set_setting(&DevNVRAM.GSETTING.SAVE_DOSE_INTERVAL, (uint32_t)GUI.editable); }break;
					case 2:{ Set_setting(&DevNVRAM.GSETTING.ALARM_THRESHOLD, (uint32_t)GUI.editable); }break;
					case 3:{ Set_setting(&DevNVRAM.GSETTING.UTC, (uint32_t)GUI.editable); }break;
					case 4:{ Set_setting(&DevNVRAM.GSETTING.log_save_period, (uint32_t)GUI.editable); }break;
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
						case 4:{ bootToDFU(); }break;
						case 5:{ GUI.menu_page = 8; }break;
					}
					GUI.cursor = 0;
				}break;
				case 1:{
					switch (GUI.cursor){
						case 0:{ GMODE.counter_mode = 0; GUI.page = 1; }break;
						case 1:{ GUI.menu_page = 4; }break;
						case 2:{ GMODE.counter_mode = 2; GUI.page = 1; GWORK.rad_max = 0;
						for(int i = 0; i < LCD_X; i++) GUI.mass[i] = 0;
						}break;
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
						case 0:{ GUI.menu_page = 0; }break;
						case 1:{ GUI.menu_page = 0; }break;
						case 2:{ GUI.menu_page = 0; }break;
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
										//sleep();
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
							case 3:{ GUI.editable = DevNVRAM.GSETTING.ACTIVE_COUNTERS; }break;
						}
						GFLAGS.is_editing_mode = true;
					}break;
					case 7:{
						switch(GUI.cursor){
							case 0:{ GUI.editable = DevNVRAM.GSETTING.LCD_CONTRAST; }break;
							case 1:{ GUI.editable = DevNVRAM.GSETTING.SAVE_DOSE_INTERVAL; }break;
							case 2:{ GUI.editable = DevNVRAM.GSETTING.ALARM_THRESHOLD; }break;
							case 3:{ GUI.editable = DevNVRAM.GSETTING.UTC; }break;
							case 4:{ GUI.editable = DevNVRAM.GSETTING.log_save_period; }break;
						}
						GFLAGS.is_editing_mode = true;
					}
				}
			}
		}
#endif
}

void button_action(){
#ifndef DEBUG
	tick(&btn_reset);
	tick(&btn_set);


	bool btn_reset_isHolded = isHolded(&btn_reset);
	bool btn_set_isHolded = isHolded(&btn_set);

	bool menu_mode = GUI.page == 2;

	if(isHold(&btn_reset) && isHold(&btn_set)){
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
		if(!menu_mode && !GFLAGS.do_alarm) GFLAGS.is_muted = !GFLAGS.is_muted;
		if(!menu_mode && GFLAGS.do_alarm) GFLAGS.no_alarm = !GFLAGS.no_alarm;
		move_cursor(false, GFLAGS.is_editing_mode, menu_mode);
		update_request();
	}else if(btn_set_isHolded){												//Удержание кнопки сет
		cursor_select(true, GFLAGS.is_editing_mode, menu_mode);
		update_request();
	}else if(isClick(&btn_set) && !btn_set_isHolded){					//Клик кнопки сет
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
		update_request();
	}
#endif
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  LL_SYSTICK_EnableIT();
  LL_SPI_Enable(SPI2);

   Initialize_data();

   adc_init();

   GPS_Init();

  lcd_config_g.MOSIPORT = GPIOA;
  lcd_config_g.MOSIPIN = LL_GPIO_PIN_7;
  lcd_config_g.SCKPORT = GPIOA;
  lcd_config_g.SCKPIN = LL_GPIO_PIN_5;
  lcd_config_g.CSPORT = GPIOA;
  lcd_config_g.CSPIN = LL_GPIO_PIN_4;
  lcd_config_g.RESPORT = GPIOB;
  lcd_config_g.RESPIN = LL_GPIO_PIN_0;

  LCD_Init(lcd_config_g);
  LCD_SetContrast(10);
  LCD_Flip();

  gbuttonInit(&btn_set, GPIOB, GPIO_IDR_IDR5, HIGH_PULL, NORM_OPEN);
  gbuttonInit(&btn_reset, GPIOB, GPIO_IDR_IDR4, HIGH_PULL, NORM_OPEN);

    //реализовать режимы энергосбережения в зависимости от уровня заряда и установленного в настройках
    //От этого будет зависеть частота процессора и яркость подсветки если она включена

    setClickTimeout(&btn_reset, 100);
    setClickTimeout(&btn_set, 100);
    setTimeout(&btn_reset, 500);
    setTimeout(&btn_set, 500);

  GMODE.counter_mode = 0;

  //Enable timers
     LL_TIM_EnableIT_UPDATE(TIM1);
     LL_TIM_EnableCounter(TIM1);

     LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3);
     LL_TIM_EnableCounter(TIM2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#ifndef DEBUG
	current_hour = (GPS.GPGGA.UTC_Hour + DevNVRAM.GSETTING.UTC)%24;

	//Timer for idle screen disable

	if((GMEANING.current_battery_voltage < BAT_ADC_MIN) && !GFLAGS.is_low_voltage) GFLAGS.is_low_voltage = true;		//Check, is battery low
	GFLAGS.is_charging = false;//(GPIOB->IDR & LL_GPIO_PIN_11);
	//if(is_low_voltage) low_battery_kill();

	if(!GFLAGS.is_charging){
		  if(strlen(rx_buffer) > 0) {																//If cbc received string, check what is it
			  if(strcmp(rx_buffer, "rdlog\n") == 0 && !GFLAGS.log_transfer){						//If it was rdlog, start transmit log from flash
				  GFLAGS.log_transfer = true;
				  GFLAGS.is_monitor_enabled = false;
				  uint32_t l_Address = 0x00001000;
				  uint32_t l_Index = 0;
				  char to_append = '\0';
				  sprintf(strbuffer, "%d\r\n", DevNVRAM.GSETTING.w25qxx_address - l_Address);
				  LL_mDelay(100);
				  SendMessage(strbuffer);
				  while(l_Address < DevNVRAM.GSETTING.w25qxx_address){
					  l_Index = 0;
					  memset(strbuffer, '\0', sizeof(strbuffer));
					  do{
						  W25qxx_ReadByte(&strbuffer[l_Index], l_Address);
						  l_Address++; l_Index++;
					  }while(strbuffer[abs(l_Index)] != '\n' && isdigit(strbuffer[abs(l_Index - 1)]));
					  strncat(strbuffer, &to_append, sizeof(char));
					  if(strbuffer != NULL) SendMessage(strbuffer);
					  //LL_mDelay(10);
					  delayUs(150);
				  }
				  SendMessage("done\r\n");
			  }else if(strcmp(rx_buffer, "clmem\n") == 0){											//If received clmem, start cleaning flash
				  SendMessage("Erasing chip.\r\n");
				  W25qxx_EraseChip();
				  SendMessage("done\0");
				  LL_mDelay(1);
				  SendMessage("Please reconnect device to computer.\r\n");
				  NVIC_SystemReset();
			  }else if(strcmp(rx_buffer, "monitor\n") == 0){										//If received monitor, let's enable work log transfer
				  GFLAGS.is_monitor_enabled = !GFLAGS.is_monitor_enabled;
			  }else if(strcmp(rx_buffer, "update\n") == 0){											//Here we starting backup and go to firmware update mode
				  SendMessage("Rebooting to dfu.\n");
				  bootToDFU();
			  }else if(strcmp(rx_buffer, "rdcfg\n") == 0){

			  }
			  memset(rx_buffer, 0, sizeof(rx_buffer));
		  }
		  GFLAGS.log_transfer = false;
		button_action();

		GFLAGS.is_charging = !(bool)LL_GPIO_ReadInputPort(GPIOB)&GPIO_IDR_IDR11;

		//Timer for update voltage values
		voltage_required();
		if(GetTick() - current_millis > 100){
			current_millis = GetTick();
			GMEANING.current_battery_voltage = get_battery_voltage();
			GMEANING.current_high_voltage = get_high_voltage();

			//High voltage regulation
			if(DevNVRAM.GSETTING.ACTIVE_COUNTERS != 0){
				if(GMEANING.current_high_voltage < GWORK.voltage_req) { if(GWORK.transformer_pwm < 200)GWORK.transformer_pwm++; }
				else { if(GWORK.transformer_pwm > 0) GWORK.transformer_pwm--; }
				TIM2->CCR1 = GWORK.transformer_pwm;
			}else{
				TIM2->CCR1 = 0;
			}

		}

		//Timer for logger
		if(GetTick() - gps_millis > DevNVRAM.GSETTING.log_save_period * 1000){
			gps_millis = GetTick();

			if(GFLAGS.is_satellites_found && GPS.end_convertation == 1 && GFLAGS.is_tracking_enabled){
				uint8_t buffer[64];
				memset(buffer, 0, sizeof(buffer));
				sprintf(buffer, "%u,%u,%u,%u,%u,%lf,%lf\n", GWORK.rad_back, GWORK.rad_dose, current_hour, GPS.GPGGA.UTC_Min, GPS.GPGGA.UTC_Sec, GPS.GPGGA.LatitudeDecimal, GPS.GPGGA.LongitudeDecimal);
				//if(GFLAGS.is_monitor_enabled) CDC_Transmit_FS(buffer, sizeof(buffer));
				GPS.end_convertation = 0;
				if(!Write_string_w25qxx(buffer)){
					//Not enought memory, can't write
					//Do something
				}
			}
		}

		GPS_Process();

		//Check is current radiation more then alarm threshold
		if((GWORK.rad_back > DevNVRAM.GSETTING.ALARM_THRESHOLD) && !GFLAGS.no_alarm && (GMODE.counter_mode == 0)) { GFLAGS.do_alarm = true; }
		else { if(GMODE.counter_mode == 0) GFLAGS.do_alarm = false; }

		if(!GFLAGS.do_alarm){ if((bool)DevNVRAM.GSETTING.LCD_BACKLIGHT){ pwm_backlight(255); } else {pwm_backlight(0);} }//Disable or enable backlight after alarm

		//Alarm algorithm
		if(!GFLAGS.no_alarm) {
			if(GFLAGS.do_alarm){
				if(GetTick()-GWORK.alarm_timer > 300){
					GWORK.alarm_timer = GetTick();
					pwm_tone(GFLAGS.is_alarm ? 100 : 200);
					pwm_backlight(GFLAGS.is_alarm ? 255 : 0);
					GFLAGS.is_alarm = !GFLAGS.is_alarm;
				}
			}
		}

		GFLAGS.is_satellites_found = (GPS.GPGGA.Latitude > 0) && (GPS.GPGGA.Longitude > 0);								//Flag for check sat status

		draw_update();
		beep();

		//Save dose part
		if(GMODE.counter_mode==0){
			if(GWORK.rad_dose - GWORK.rad_dose_old > DevNVRAM.GSETTING.SAVE_DOSE_INTERVAL){
				GWORK.rad_dose_old = GWORK.rad_dose;

				GWORK.rad_max = GWORK.rad_back;
			}
		}
	}else{
		//pwm_transformer(0);
		pwm_tone(0);
		pwm_backlight(0);
	}
#endif
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(36000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSRC_PCLK2_DIV_4);
  LL_RCC_SetUSBClockSource(LL_RCC_USB_CLKSOURCE_PLL_DIV_1_5);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**ADC1 GPIO Configuration
  PA0-WKUP   ------> ADC1_IN0
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* ADC1 interrupt Init */
  NVIC_SetPriority(ADC1_2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),4, 0));
  NVIC_EnableIRQ(ADC1_2_IRQn);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_0, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC2);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**ADC2 GPIO Configuration
  PB1   ------> ADC2_IN9
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* ADC2 interrupt Init */
  NVIC_SetPriority(ADC1_2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),4, 0));
  NVIC_EnableIRQ(ADC1_2_IRQn);

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
  LL_ADC_Init(ADC2, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  LL_ADC_REG_Init(ADC2, &ADC_REG_InitStruct);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_9);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_9, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**SPI2 GPIO Configuration
  PB13   ------> SPI2_SCK
  PB14   ------> SPI2_MISO
  PB15   ------> SPI2_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13|LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* SPI2 interrupt Init */
  NVIC_SetPriority(SPI2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(SPI2_IRQn);

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI2, &SPI_InitStruct);
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* TIM1 interrupt Init */
  NVIC_SetPriority(TIM1_UP_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(TIM1_UP_IRQn);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 3600-LL_TIM_IC_FILTER_FDIV1_N2;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 9999;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* TIM2 interrupt Init */
  NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),3, 0));
  NVIC_EnableIRQ(TIM2_IRQn);

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  TIM_InitStruct.Prescaler = 35;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 255;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM2);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH2);
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH3);
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH3);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**TIM2 GPIO Configuration
  PB10   ------> TIM2_CH3
  PA15   ------> TIM2_CH1
  PB3   ------> TIM2_CH2
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10|LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  LL_GPIO_AF_EnableRemap_TIM2();

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 9600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, SPI1_CS_Pin|SPI1_SCK_Pin|SPI1_MOSI_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, SPI1_RST_Pin|LL_GPIO_PIN_12|LL_GPIO_PIN_7);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = SPI1_CS_Pin|SPI1_SCK_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = SPI1_RST_Pin|LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11|BSET_Pin|BRSET_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTA, LL_GPIO_AF_EXTI_LINE1);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTA, LL_GPIO_AF_EXTI_LINE2);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTA, LL_GPIO_AF_EXTI_LINE3);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_1;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_2;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_3;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinPull(GINT1_GPIO_Port, GINT1_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(GINT2_GPIO_Port, GINT2_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(GINT3_GPIO_Port, GINT3_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(GINT1_GPIO_Port, GINT1_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(GINT2_GPIO_Port, GINT2_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(GINT3_GPIO_Port, GINT3_Pin, LL_GPIO_MODE_INPUT);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(EXTI1_IRQn);
  NVIC_SetPriority(EXTI2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(EXTI2_IRQn);
  NVIC_SetPriority(EXTI3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	while(1){
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
