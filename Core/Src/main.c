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
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "configuration.h"
#include "stdio.h"
#include "util.h"
#include <managers/meaning_manager.h>
#include "managers/data_manager.h"
#include "libs/GyverButton_stm32.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
unsigned long current_millis;
extern bool is_memory_initialized;
extern DMGRESULT error_detector;

extern uint16_t current_battery_voltage, current_high_voltage;
extern bool is_charging, is_low_voltage, do_alarm, no_alarm, is_alarm;
extern uint32_t rad_dose_old;
extern volatile uint32_t rad_back, rad_max, rad_dose;
extern uint16_t Transformer_pwm, LCD_backlight;
extern uint8_t Save_dose_interval, counter_mode, Alarm_threshold;

extern unsigned long alarm_timer;
GyverButton btn_set;
GyverButton btn_reset;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void button_action(){
	tick(&btn_reset);
	tick(&btn_set);

	if(isClick(&btn_reset)){
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}

	/*bool btn_reset_isHolded = btn_reset.isHolded();
	bool btn_set_isHolded = btn_set.isHolded();

	bool menu_mode = datamgr.page == 2;
	bool editing_mode = datamgr.editing_mode;

	if(btn_reset.isHold() && btn_set.isHold()){
		if(!menu_mode){
			datamgr.page = 2;
			datamgr.menu_page = 0;
			datamgr.editing_mode = false;
		}else{
			datamgr.editing_mode = false;
			datamgr.page = 1;
		}
		outmgr.update_request();
		btn_reset.resetStates();
		btn_set.resetStates();
	}else if(btn_set.isHold() && !menu_mode){
		if(!menu_mode && !btn_reset.isPress()) outmgr.battery_request(true);
	}else if(btn_reset_isHolded){											//Удержание кнопки ресет
		if(!menu_mode && !btn_set.isPress()) datamgr.no_alarm = !datamgr.no_alarm;
		if(menu_mode && !editing_mode){										//Если находимся в меню
			datamgr.is_detected = true;
			if(datamgr.menu_page == 0) {datamgr.page = 1; datamgr.do_alarm = false;}
			else if(datamgr.menu_page == 6) datamgr.menu_page = 2;
			else if(datamgr.menu_page == 7) datamgr.menu_page = 6;
			else datamgr.menu_page = 0;
			datamgr.cursor = 0;
		}
		if(editing_mode){
			datamgr.is_detected = true;
			datamgr.editing_mode = false;
		}
		if(!menu_mode && datamgr.counter_mode == 1){
			datamgr.reset_activity_test();
			datamgr.timer_remain = datamgr.timer_time;
			datamgr.time_min = datamgr.time_min_old;
		}
		outmgr.update_request();
	}else if(btn_reset.isClick() && !btn_reset_isHolded){					//Клик кнопки ресет
		if(!menu_mode && !datamgr.do_alarm) datamgr.mute = !datamgr.mute;
		if(!menu_mode && datamgr.do_alarm) datamgr.no_alarm = !datamgr.no_alarm;
		if(menu_mode && !editing_mode && datamgr.cursor > 0) { datamgr.is_detected = true; datamgr.cursor--; }
		if(editing_mode){
			if(datamgr.menu_page == 2){
				#if defined(UNIVERSAL_COUNTER)
				if(datamgr.cursor == 1 && datamgr.editable > 0) datamgr.editable-=5;
				if(datamgr.cursor == 2 && datamgr.editable > 0) datamgr.editable-=51;
				if(datamgr.cursor == 3 && datamgr.editable > 0) datamgr.editable-=5;
				if(datamgr.cursor == 4 && datamgr.editable > 0) datamgr.editable--;
				if(datamgr.cursor == 5 && datamgr.editable > 30) datamgr.editable-=5;
				#else
				if(datamgr.cursor == 0 && datamgr.editable > 0) datamgr.editable-=5;
				if(datamgr.cursor == 1 && datamgr.editable > 0) datamgr.editable-=51;
				if(datamgr.cursor == 2 && datamgr.editable > 0) datamgr.editable-=5;
				if(datamgr.cursor == 3 && datamgr.editable > 0) datamgr.editable--;
				if(datamgr.cursor == 4 && datamgr.editable > 0) datamgr.editable-=5;
				#endif
			}else if(datamgr.menu_page == 4){
				switch (datamgr.cursor){
					case 0:{ if(datamgr.editable > 1) datamgr.editable--; }break;
					case 1:{ if(datamgr.editable > 0) datamgr.editable--; }break;
				}
			}
			#if defined(UNIVERSAL_COUNTER)
			else if(datamgr.menu_page == 7){
				switch (datamgr.cursor){
					case 0:{ if(datamgr.editable > 1) datamgr.editable--; } break;
					case 1:{ if(datamgr.editable > 1) datamgr.editable--; } break;
				}
			}
			#endif
		}
		outmgr.update_request();
	}else if(btn_set_isHolded){												//Удержание кнопки сет
		if(datamgr.is_sleeping) sleep();
		if(menu_mode && !editing_mode) {
			datamgr.is_detected = true;
			switch (datamgr.menu_page){
				case 0:{
					switch (datamgr.cursor){
						case 0:{ datamgr.menu_page = 1; }break;
						case 1:{ datamgr.menu_page = 2; }break;
						case 2:{ datamgr.menu_page = 3; }break;
						#if defined(CAN_SLEEP)
						case 3:{ datamgr.menu_page = 5; }break;
						#endif
					}
					datamgr.cursor = 0;
				}break;
				case 1:{
					switch (datamgr.cursor){
						case 0:{ datamgr.counter_mode = 0; datamgr.page = 1; }break;
						case 1:{ datamgr.menu_page = 4; }break;
						case 2:{ datamgr.counter_mode = 2; datamgr.page = 1; datamgr.rad_max = 0;
						#if defined(DRAW_GRAPH)
							for(int i = 0; i < 83; i++) datamgr.mass[i] = 0;
						#endif
						}break;
					}
					datamgr.cursor = 0;
				}break;
				case 2:{
					switch (datamgr.cursor){
						#if defined(UNIVERSAL_COUNTER)
						case 0:{ datamgr.menu_page = 6; }break;
						case 1:{ datamgr.editable = datamgr.ton_BUZZ; }break;
						case 2:{ datamgr.editable = datamgr.backlight; }break;
						case 3:{ datamgr.editable = datamgr.contrast; }break;
						case 4:{ datamgr.editable = datamgr.save_dose_interval; }break;
						case 5:{ datamgr.editable = datamgr.alarm_threshold; }break;
						#else
						case 0:{ datamgr.editable = datamgr.ton_BUZZ; }break;
						case 1:{ datamgr.editable = datamgr.backlight; }break;
						case 2:{ datamgr.editable = datamgr.contrast; }break;
						case 3:{ datamgr.editable = datamgr.save_dose_interval; }break;
						case 4:{ datamgr.editable = datamgr.alarm_threshold; }break;
						#endif
					}
					#if defined(UNIVERSAL_COUNTER)
					if(datamgr.cursor != 0) datamgr.editing_mode = true;
					#else
					datamgr.editing_mode = true;
					#endif
				}break;
				case 3:{
					switch (datamgr.cursor){								//Стереть данные
						case 0:{ datamgr.reset_settings(); datamgr.menu_page = 0; }break;
						case 1:{ datamgr.reset_dose(); datamgr.menu_page = 0; }break;
						case 2:{ datamgr.reset_settings(); datamgr.reset_dose(); datamgr.menu_page = 0; }break;
					}
					datamgr.cursor = 0;
				}break;
				case 4:{
					switch (datamgr.cursor){
						case 0:{ datamgr.editable = datamgr.time_min; }break;
						case 1:{ datamgr.editable = datamgr.means_times; }break;
						case 2:{
							datamgr.reset_activity_test();
							datamgr.time_min_old = datamgr.time_min;
							datamgr.timer_time = datamgr.time_min * 60;
							datamgr.timer_remain = datamgr.timer_time;
						}break;
					}
					if(datamgr.cursor != 2) datamgr.editing_mode = true;

				}break;
				case 5:{
					switch (datamgr.cursor){								//Вообще это диалог выбора, но пока что это не он
						case 0:{
						#if defined(CAN_SLEEP)
						sleep();
						#endif
						}break;
						case 1:{ datamgr.menu_page = 0; }break;
					}
					datamgr.cursor = 0;
				}break;
				#if defined(UNIVERSAL_COUNTER)
				case 6:{
					switch (datamgr.cursor){
						case 0:{ datamgr.menu_page = 2; datamgr.setup_sbm20(); }break;
						case 1:{ datamgr.menu_page = 2; datamgr.setup_sbm19(); }break;
						case 2:{ datamgr.menu_page = 2; datamgr.setup_beta(); }break;
						case 3:{ datamgr.menu_page = 7; }break;
					}
					datamgr.cursor = 0;
				}break;
				case 7:{
					switch (datamgr.cursor){
						case 0:{ datamgr.editable = datamgr.GEIGER_TIME; }break;
						case 1:{ datamgr.editable = datamgr.geiger_error; }break;
					}
					datamgr.editing_mode = true;
				}break;
				#endif
			}
		}
		if(menu_mode && editing_mode){
			datamgr.is_detected = true;
			if(datamgr.menu_page == 4){
				switch (datamgr.cursor){
					case 0:{ datamgr.time_min = datamgr.editable; }break;
					case 1:{ datamgr.means_times = datamgr.editable; }break;
				}
			}
			#if defined(UNIVERSAL_COUNTER)
			else if(datamgr.menu_page == 7){
				switch (datamgr.cursor){
					case 0:{ datamgr.save_time(); }break;
					case 1:{ datamgr.save_error(); }break;
				}
			}
			#endif
			else{
				switch (datamgr.cursor){
					#if defined(UNIVERSAL_COUNTER)
					case 1:{ datamgr.save_tone(); }break;
					case 2:{ datamgr.save_bl(); }break;
					case 3:{ datamgr.save_contrast(); }break;
					case 4:{ datamgr.save_interval(); }break;
					case 5:{ datamgr.save_alarm(); }break;
					#else
					case 0:{ datamgr.save_tone(); }break;
					case 1:{ datamgr.save_bl(); }break;
					case 2:{ datamgr.save_contrast(); }break;
					case 3:{ datamgr.save_interval(); }break;
					case 4:{ datamgr.save_alarm(); }break;
					#endif
				}
			}
			datamgr.editing_mode = false;
		}
		outmgr.update_request();
	}else if(btn_set.isClick() && !btn_set_isHolded){					//Клик кнопки сет
		if(!menu_mode && datamgr.counter_mode == 1 && !datamgr.next_step && datamgr.stop_timer){
			datamgr.rad_max = datamgr.rad_back;
			datamgr.rad_back = 0;
			datamgr.next_step = true;
			datamgr.stop_timer = false;
			datamgr.do_alarm = false;
			datamgr.time_min = datamgr.time_min_old;
			datamgr.timer_remain = datamgr.timer_time;
			datamgr.time_sec = 0;
		}
		if(!menu_mode && datamgr.counter_mode == 1 && datamgr.do_alarm && datamgr.next_step && datamgr.stop_timer){
			datamgr.do_alarm = false;
		}

		if(!menu_mode && datamgr.counter_mode == 0){
			datamgr.mean_mode = !datamgr.mean_mode;
		}

		if(menu_mode && !editing_mode){						//Сдвинуть курсор, если можно
			datamgr.is_detected = true;
			switch (datamgr.menu_page){
				#if defined(CAN_SLEEP)
				case 0:{ if(datamgr.cursor < 3) datamgr.cursor++; } break;
				#else
				case 0:{ if(datamgr.cursor < 2) datamgr.cursor++; } break;
				#endif
				case 1:{ if(datamgr.cursor < 2) datamgr.cursor++; } break;
				#if defined(UNIVERSAL_COUNTER)
				case 2:{ if(datamgr.cursor < 5) datamgr.cursor++; } break;
				#else
				case 2:{ if(datamgr.cursor < 2) datamgr.cursor++; } break;
				#endif
				case 3:{ if(datamgr.cursor < 2) datamgr.cursor++; } break;
				case 4:{ if(datamgr.cursor < 2) datamgr.cursor++; } break;
				#if defined(CAN_SLEEP)
				case 5:{ if(datamgr.cursor < 1) datamgr.cursor++; } break;
				#endif
				#if defined(UNIVERSAL_COUNTER)
				case 6:{ if(datamgr.cursor < 3) datamgr.cursor++; } break;
				case 7:{ if(datamgr.cursor < 1) datamgr.cursor++; } break;
				#endif
			}
		}
		if(editing_mode){
			if(datamgr.menu_page == 2){
				#if defined(UNIVERSAL_COUNTER)
				if(datamgr.cursor == 1 && datamgr.editable < 255) datamgr.editable+=5;
				if(datamgr.cursor == 2 && datamgr.editable < 255) datamgr.editable+=51;
				if(datamgr.cursor == 3 && datamgr.editable < 255) datamgr.editable+=5;
				if(datamgr.cursor == 4 && datamgr.editable < 255) datamgr.editable++;
				if(datamgr.cursor == 5 && datamgr.editable < 255) datamgr.editable+=5;
				#else
				if(datamgr.cursor == 0 && datamgr.editable < 255) datamgr.editable+=5;
				if(datamgr.cursor == 1 && datamgr.editable < 255) datamgr.editable+=51;
				if(datamgr.cursor == 2 && datamgr.editable < 255) datamgr.editable+=5;
				if(datamgr.cursor == 3 && datamgr.editable < 255) datamgr.editable++;
				if(datamgr.cursor == 4 && datamgr.editable < 255) datamgr.editable+5;
				#endif
			}else if(datamgr.menu_page == 4){
				switch (datamgr.cursor){
					case 0:{ datamgr.editable++; }break;
					case 1:{ if(datamgr.editable < 1) datamgr.editable++; }break;
				}
			}
			#if defined(UNIVERSAL_COUNTER)
			else if(datamgr.menu_page == 7){
				switch (datamgr.cursor){
					case 0:{ if(datamgr.editable < 100) datamgr.editable++; } break;
					case 1:{ if(datamgr.editable < 40) datamgr.editable++; } break;
				}
			}
			#endif
		}
		outmgr.update_request();
	}*/
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  Initialize_data();

  //if(error_detector == NO_ERROR){

	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

	  HAL_ADCEx_Calibration_Start(&hadc1);
	  HAL_ADC_Start_IT(&hadc1);
	  HAL_ADCEx_Calibration_Start(&hadc2);
	  HAL_ADC_Start_IT(&hadc2);

	  //Enable timers
	  HAL_TIM_Base_Start_IT(&htim1);
	  //HAL_TIM_Base_Start_IT(&htim2);
	  HAL_TIM_Base_Start_IT(&htim3);

  //}else{
	  //Error_Handler();
  //}

  gbuttonInit(&btn_set, GPIOB, GPIO_PIN_4, HIGH_PULL, NORM_OPEN);
  gbuttonInit(&btn_reset, GPIOB, GPIO_PIN_5, HIGH_PULL, NORM_OPEN);

  setClickTimeout(&btn_reset, 100);
  setClickTimeout(&btn_set, 100);
  setTimeout(&btn_reset, 1000);
  setTimeout(&btn_set, 1000);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	is_charging = !(bool)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);

	if(millis() - current_millis > 5000){
		current_millis = millis();
		adc_enable_reading();
		current_battery_voltage = get_battery_voltage();
		current_high_voltage = get_high_voltage();
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}

	if((current_battery_voltage < BAT_ADC_MIN) && !is_low_voltage) is_low_voltage = true;
	//if(is_low_voltage) low_battery_kill();

	if(!is_charging){
		if(get_high_voltage() < HV_ADC_REQ) { Transformer_pwm++; }
		else { Transformer_pwm--; }
		pwm_transformer(Transformer_pwm);
		if((rad_back > Alarm_threshold) && !no_alarm && (counter_mode == 0)) { do_alarm = true; }
		else { if(counter_mode == 0) do_alarm = false; }
		if(!do_alarm) pwm_backlight(LCD_backlight);
		if(!no_alarm) {
			if(do_alarm){
				if(millis()-alarm_timer > 300){
					alarm_timer = millis();
					pwm_tone(is_alarm ? 100 : 200);
					pwm_backlight(is_alarm ? 204 : 0);
					is_alarm = !is_alarm;
				}
			}
		}

		//if(get_battery_requet()) update_request();
		//battery_request(false);
		button_action();

		if(counter_mode==0){
			if(rad_dose - rad_dose_old > Save_dose_interval){
				rad_dose_old = rad_dose;
				Save_dose();
				rad_max = rad_back;
			}
		}
	}else{
		//ADCManager::pwm_PD3(0);
		//ADCManager::pwm_PB3(0);
	}
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 3600-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 18000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : GINT1_Pin GINT2_Pin */
  GPIO_InitStruct.Pin = GINT1_Pin|GINT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GINT3_Pin */
  GPIO_InitStruct.Pin = GINT3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GINT3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_RST_Pin PB11 BSET_Pin BRSET_Pin */
  GPIO_InitStruct.Pin = SPI1_RST_Pin|GPIO_PIN_11|BSET_Pin|BRSET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

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
		switch(error_detector){
			case FLASH_MEMORY_ERROR:{
				for(int i = 0; i < 10; i++){
					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
					HAL_Delay(100);
				}
			}break;

			case HEAP_INITIALIZATION_ERROR:{
				for(int i = 0; i < 20; i++){
					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
					HAL_Delay(100);
				}
			}break;
		}
		HAL_Delay(1000);
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
