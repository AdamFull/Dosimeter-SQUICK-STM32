/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "managers/data_manager.h"
/* USER CODE END Includes */

/* External functions --------------------------------------------------------*/
void SystemClock_Config(void);

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
volatile unsigned long millis_counter;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern volatile unsigned long millis_timer;

extern volatile uint8_t mass[];
extern uint8_t page, active_counters, counter_mode, Real_geigertime;
extern volatile uint8_t time_min, time_sec, x_p, stat_time;
extern volatile uint16_t timer_remain;
extern uint16_t *rad_buff;
extern volatile uint64_t rad_sum, rad_back, rad_max, rad_dose, sum_old;
extern uint32_t *stat_buff;
extern volatile bool is_detected;
extern bool stop_timer, do_alarm, is_mean_mode;
extern float mean;
extern uint16_t battery_adc_value, high_voltage_adc_value;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1) //check if the interrupt comes from ACD1
    {
    	battery_adc_value = HAL_ADC_GetValue(hadc);
    }else if(hadc->Instance == ADC2){
    	high_voltage_adc_value = HAL_ADC_GetValue(hadc);
    }
    //HAL_ADC_Start_IT(hadc);
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern PCD_HandleTypeDef hpcd_USB_FS;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
	if(active_counters == 1 || active_counters == 3){
		if(counter_mode==0){    //Режим поиска
			if(rad_buff[0]!=65535) rad_buff[0]++;
			if(++rad_sum>999999UL*3600/Real_geigertime) rad_sum=999999UL*3600/Real_geigertime; //общая сумма импульсов
			if(page == 1 && !do_alarm){ is_detected = true; }
		}else if(counter_mode==1){							//Режим измерения активности
			if(!stop_timer) if(++rad_back>999999UL*3600/Real_geigertime) rad_back=999999UL*3600/Real_geigertime; //Сумма импульсов для режима измерения
		}else if(counter_mode==2){							//Режим измерения активности
			if(rad_buff[0]!=65535) rad_buff[0]++;
			//outmgr.update_request();
		}
		//ADCManager::pwm_PD3(datamgr.pwm_converter + 10);
	}
  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
	if(active_counters == 2 || active_counters == 3){
		if(counter_mode==0){    //Режим поиска
			if(rad_buff[0]!=65535) rad_buff[0]++;
			if(++rad_sum>999999UL*3600/Real_geigertime) rad_sum=999999UL*3600/Real_geigertime; //общая сумма импульсов
			if(page == 1 && !do_alarm){ is_detected = true; }
		}else if(counter_mode==1){							//Режим измерения активности
			if(!stop_timer) if(++rad_back>999999UL*3600/Real_geigertime) rad_back=999999UL*3600/Real_geigertime; //Сумма импульсов для режима измерения
		}else if(counter_mode==2){							//Режим измерения активности
			if(rad_buff[0]!=65535) rad_buff[0]++;
				//outmgr.update_request();
		}
		//ADCManager::pwm_PD3(datamgr.pwm_converter + 10);
	}
  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */
	if(active_counters == 0){
		if(counter_mode==0){    //Режим поиска
			if(rad_buff[0]!=65535) rad_buff[0]++;
			if(++rad_sum>999999UL*3600/Real_geigertime) rad_sum=999999UL*3600/Real_geigertime; //общая сумма импульсов
			if(page == 1 && !do_alarm){ is_detected = true; }
		}else if(counter_mode==1){							//Режим измерения активности
			if(!stop_timer) if(++rad_back>999999UL*3600/Real_geigertime) rad_back=999999UL*3600/Real_geigertime; //Сумма импульсов для режима измерения
		}else if(counter_mode==2){							//Режим измерения активности
			if(rad_buff[0]!=65535) rad_buff[0]++;
			//outmgr.update_request();
		}
		//ADCManager::pwm_PD3(datamgr.pwm_converter + 10);
	}
  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupts.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */
	//update_request();

	uint32_t tmp_buff=0;

	if(counter_mode == 0){
		for(uint8_t i=0; i<Real_geigertime; i++) tmp_buff+=rad_buff[i]; //расчет фона мкР/ч
		if(tmp_buff>999999) tmp_buff=999999; //переполнение
		rad_back=tmp_buff;
		stat_buff[stat_time] = rad_back; //Записываю текущее значение мкр/ч для расчёта погрешности

		Calculate_std(); //- crashing

		if(rad_back>rad_max) rad_max=rad_back; //фиксируем максимум фона
		for(uint8_t k=Real_geigertime-1; k>0; k--) rad_buff[k]=rad_buff[k-1]; //перезапись массива

		rad_buff[0]=0; //сбрасываем счетчик импульсов

		if(is_mean_mode){
			rad_back = (uint32_t)mean;
		}

		if(stat_time > Real_geigertime) stat_time = 0; //Счётчик для расчёта статистической погрешности
		else stat_time++;

		rad_dose=(rad_sum*Real_geigertime/3600); //расчитаем дозу

		//mass[x_p]=map(rad_back, 0, rad_max < 40 ? 40 : rad_max, 0, 15);
		if(x_p<83)x_p++;
		if(x_p==83){
			for(uint8_t i=0;i<83;i++)mass[i]=mass[i+1];
		}
		//if(rad_max > 1) rad_max--;		//Потихоньку сбрасываем максиму

	}else if(counter_mode == 1){
		//ТАймер для второго режима. Обратный отсчёт
		//bool stop_timer = stop_timer;
		if(!stop_timer){
			if(time_min != 0 && time_sec == 0){
				--time_min;
				time_sec=60;
			}
			if(time_sec != 0){ --time_sec; }
			timer_remain--;
			if(timer_remain == 0){
				stop_timer = true;
				do_alarm = true;
			}
		}
	}else if(counter_mode == 2){
		//Секундный замер, сбрасываем счётчик каждую секунду
		if(rad_max < rad_buff[0]) rad_max = rad_buff[0];
		sum_old=rad_buff[0];

		//mass[x_p]=map(rad_buff[0], 0, rad_max < 2 ? 2 : rad_max, 0, 15);
	    if(x_p<83)x_p++;
	    if(x_p==83){
	        for(uint8_t i=0;i<83;i++)mass[i]=mass[i+1];
	    }

		if(rad_max > 1) rad_max--;		//Потихоньку сбрасываем максимум
			rad_buff[0]=0; //сбрасываем счетчик импульсов
		}
  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
  //millis_counter++;
  //if(millis_counter > 1000){
	//  millis_counter = 0;
	  millis_timer++;
  //}
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USB wake-up interrupt through EXTI line 18.
  */
void USBWakeUp_IRQHandler(void)
{
  /* USER CODE BEGIN USBWakeUp_IRQn 0 */

  /* USER CODE END USBWakeUp_IRQn 0 */
  if ((&hpcd_USB_FS)->Init.low_power_enable) {
    /* Reset SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
    SystemClock_Config();
  }
  /* Clear EXTI pending bit */
  __HAL_USB_WAKEUP_EXTI_CLEAR_FLAG();
  /* USER CODE BEGIN USBWakeUp_IRQn 1 */

  /* USER CODE END USBWakeUp_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
