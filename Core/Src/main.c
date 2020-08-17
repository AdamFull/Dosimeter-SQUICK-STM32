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

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
unsigned long current_millis;

GyverButton btn_set;
GyverButton btn_reset;

extern DMGRESULT error_detector;

extern geiger_flags GFLAGS;
extern geiger_meaning GMEANING;
extern geiger_work GWORK;
extern geiger_mode GMODE;
extern geiger_settings GSETTING;
extern geiger_ui GUI;
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
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len){
	int i = 0;
	for(i = 0; i<len; i++){
		ITM_SendChar((*ptr++));
	}
	return len;
}

void button_action(){
	tick(&btn_reset);
	tick(&btn_set);


	bool btn_reset_isHolded = isHolded(&btn_reset);
	bool btn_set_isHolded = isHolded(&btn_set);

	bool menu_mode = GUI.page == 2;
	bool editing_mode = GFLAGS.is_editing_mode;

	/*if(isHold(&btn_reset) && isHold(&btn_set)){
		if(!menu_mode){
			page = 2;
			menu_page = 0;
			editing_mode = false;
		}else{
			editing_mode = false;
			page = 1;
		}
		//update_request();
		resetStates(&btn_reset);
		resetStates(&btn_set);
	}else if(isHold(&btn_set) && !menu_mode){
		if(!menu_mode && !isPress(btn_reset)) battery_request(true);
	}else if(btn_reset_isHolded){											//Удержание кнопки ресет
		if(!menu_mode && !isPress(&btn_set)) no_alarm = !no_alarm;
		if(menu_mode && !editing_mode){										//Если находимся в меню
			is_detected = true;
			if(menu_page == 0) {page = 1; do_alarm = false;}
			else if(menu_page == 6) menu_page = 2;
			else if(menu_page == 7) menu_page = 6;
			else menu_page = 0;
			cursor = 0;
		}
		if(editing_mode){
			is_detected = true;
			editing_mode = false;
		}
		if(!menu_mode && counter_mode == 1){
			Reset_activity_test();
			timer_remain = timer_time;
			time_min = time_min_old;
		}
		update_request();
	}else if(isClick(&btn_reset) && !btn_reset_isHolded){					//Клик кнопки ресет
		if(!menu_mode && !do_alarm) mute = !mute;
		if(!menu_mode && do_alarm) no_alarm = !no_alarm;
		if(menu_mode && !editing_mode && cursor > 0) { is_detected = true; cursor--; }
		if(editing_mode){
			if(menu_page == 2){
				if(cursor == 1 && editable > 0) editable-=5;
				if(cursor == 2 && editable > 0) editable-=51;
				if(cursor == 3 && editable > 0) editable-=5;
				if(cursor == 4 && editable > 0) editable--;
				if(cursor == 5 && editable > 30) editable-=5;
			}else if(menu_page == 4){
				switch (cursor){
					case 0:{ if(editable > 1) editable--; }break;
					case 1:{ if(editable > 0) editable--; }break;
				}
			}
			else if(menu_page == 7){
				switch (cursor){
					case 0:{ if(editable > 1) editable--; } break;
					case 1:{ if(editable > 1) editable--; } break;
				}
			}
		}
		update_request();
	}else if(btn_set_isHolded){												//Удержание кнопки сет
		if(is_sleeping) sleep();
		if(menu_mode && !editing_mode) {
			is_detected = true;
			switch (menu_page){
				case 0:{
					switch (cursor){
						case 0:{ menu_page = 1; }break;
						case 1:{ menu_page = 2; }break;
						case 2:{ menu_page = 3; }break;
						case 3:{ menu_page = 5; }break;
					}
					cursor = 0;
				}break;
				case 1:{
					switch (cursor){
						case 0:{ counter_mode = 0; page = 1; }break;
						case 1:{ menu_page = 4; }break;
						case 2:{ counter_mode = 2; page = 1; rad_max = 0;
						for(int i = 0; i < 83; i++) datamgr.mass[i] = 0;
						}break;
					}
					cursor = 0;
				}break;
				case 2:{
					switch (cursor){
						case 0:{ menu_page = 6; }break;
						case 1:{ editable = ton_BUZZ; }break;
						case 2:{ editable = backlight; }break;
						case 3:{ editable = contrast; }break;
						case 4:{ editable = save_dose_interval; }break;
						case 5:{ editable = alarm_threshold; }break;
					}
					if(cursor != 0) editing_mode = true;
				}break;
				case 3:{
					switch (cursor){								//Стереть данные
						case 0:{ reset_settings(); menu_page = 0; }break;
						case 1:{ reset_dose(); menu_page = 0; }break;
						case 2:{ reset_settings(); reset_dose(); menu_page = 0; }break;
					}
					cursor = 0;
				}break;
				case 4:{
					switch (cursor){
						case 0:{ editable = time_min; }break;
						case 1:{ editable = means_times; }break;
						case 2:{
							Reset_activity_test();
							time_min_old = time_min;
							timer_time = time_min * 60;
							timer_remain = timer_time;
						}break;
					}
					if(cursor != 2) editing_mode = true;

				}break;
				case 5:{
					switch (cursor){								//Вообще это диалог выбора, но пока что это не он
						case 0:{
						sleep();
						}break;
						case 1:{ menu_page = 0; }break;
					}
					cursor = 0;
				}break;
				case 6:{
					switch (cursor){
						case 0:{ menu_page = 7; }break;
					}
					cursor = 0;
				}break;
				case 7:{
					switch (cursor){
						case 0:{ editable = GEIGER_TIME; }break;
						case 1:{ editable = geiger_error; }break;
					}
					editing_mode = true;
				}break;
			}
		}
		if(menu_mode && editing_mode){
			is_detected = true;
			if(menu_page == 4){
				switch (cursor){
					case 0:{ time_min = editable; }break;
					case 1:{ means_times = editable; }break;
				}
			}
			else if(menu_page == 7){
				switch (cursor){
					case 0:{ Save_time(); }break;
					case 1:{ Save_error(); }break;
				}
			}
			else{
				switch (cursor){
					case 1:{ Save_tone(); }break;
					case 2:{ Save_bl(); }break;
					case 3:{ Save_contrast(); }break;
					case 4:{ Save_interval(); }break;
					case 5:{ Save_alarm(); }break;
				}
			}
			editing_mode = false;
		}
		update_request();
	}else if(btn_set.isClick() && !btn_set_isHolded){					//Клик кнопки сет
		if(!menu_mode && counter_mode == 1 && !next_step && stop_timer){
			rad_max = rad_back;
			rad_back = 0;
			next_step = true;
			stop_timer = false;
			do_alarm = false;
			time_min = datamgr.time_min_old;
			timer_remain = datamgr.timer_time;
			time_sec = 0;
		}
		if(!menu_mode && datamgr.counter_mode == 1 && datamgr.do_alarm && datamgr.next_step && datamgr.stop_timer){
			do_alarm = false;
		}

		if(!menu_mode && counter_mode == 0){
			mean_mode = !mean_mode;
		}

		if(menu_mode && !editing_mode){						//Сдвинуть курсор, если можно
			is_detected = true;
			switch (menu_page){
				case 0:{ if(datamgr.cursor < 3) datamgr.cursor++; } break;
				case 1:{ if(datamgr.cursor < 2) datamgr.cursor++; } break;
				case 2:{ if(datamgr.cursor < 5) datamgr.cursor++; } break;
				case 3:{ if(datamgr.cursor < 2) datamgr.cursor++; } break;
				case 4:{ if(datamgr.cursor < 2) datamgr.cursor++; } break;
				case 5:{ if(datamgr.cursor < 1) datamgr.cursor++; } break;
				case 6:{ if(datamgr.cursor < 3) datamgr.cursor++; } break;
				case 7:{ if(datamgr.cursor < 1) datamgr.cursor++; } break;
			}
		}
		if(editing_mode){
			if(menu_page == 2){
				if(cursor == 1 && editable < 255) editable+=5;
				if(cursor == 2 && editable < 255) editable+=51;
				if(cursor == 3 && editable < 255) editable+=5;
				if(cursor == 4 && editable < 255) editable++;
				if(cursor == 5 && editable < 255) editable+=5;
			}else if(menu_page == 4){
				switch (cursor){
					case 0:{ editable++; }break;
					case 1:{ if(editable < 1) editable++; }break;
				}
			}
			else if(menu_page == 7){
				switch (cursor){
					case 0:{ if(editable < 100) editable++; } break;
					case 1:{ if(editable < 40) editable++; } break;
				}
			}
		}
		update_request();
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */
  Initialize_data();

  //if(error_detector == NO_ERROR){

	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	  //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
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
	GFLAGS.is_charging = !(bool)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);

	if(millis() - current_millis > 5000){
		current_millis = millis();
		adc_enable_reading();
		GMEANING.current_battery_voltage = get_battery_voltage();
		GMEANING.current_high_voltage = get_high_voltage();
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}

	if((GMEANING.current_battery_voltage < BAT_ADC_MIN) && !GFLAGS.is_low_voltage) GFLAGS.is_low_voltage = true;
	//if(is_low_voltage) low_battery_kill();

	if(!GFLAGS.is_charging){
		if(get_high_voltage() < HV_ADC_REQ) { GWORK.transformer_pwm++; }
		else { GWORK.transformer_pwm--; }
		pwm_transformer(GWORK.transformer_pwm);
		if((GWORK.rad_back > GSETTING.ALARM_THRESHOLD) && !GFLAGS.no_alarm && (GMODE.counter_mode == 0)) { GFLAGS.do_alarm = true; }
		else { if(GMODE.counter_mode == 0) GFLAGS.do_alarm = false; }
		if(!GFLAGS.do_alarm) pwm_backlight(GSETTING.LCD_BACKLIGHT);
		if(!GFLAGS.no_alarm) {
			if(GFLAGS.do_alarm){
				if(millis()-GWORK.alarm_timer > 300){
					GWORK.alarm_timer = millis();
					pwm_tone(GFLAGS.is_alarm ? 100 : 200);
					pwm_backlight(GFLAGS.is_alarm ? 204 : 0);
					GFLAGS.is_alarm = !GFLAGS.is_alarm;
				}
			}
		}

		//if(get_battery_requet()) update_request();
		//battery_request(false);
		button_action();

		if(GMODE.counter_mode==0){
			if(GWORK.rad_dose - GWORK.rad_dose_old > GSETTING.SAVE_DOSE_INTERVAL){
				GWORK.rad_dose_old = GWORK.rad_dose;
				Save_dose();
				GWORK.rad_max = GWORK.rad_back;
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
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.low_power_enable = ENABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
