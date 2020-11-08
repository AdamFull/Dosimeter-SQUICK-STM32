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
#include "usbd_cdc_if.h"
#include "stdbool.h"
#include "managers/data_manager.h"
#include "util.h"
#include "configuration.h"
/* USER CODE END Includes */

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


volatile unsigned long ticks;
extern uint16_t battery_adc_value, high_voltage_adc_value;
extern bool screen_saver_state;

extern geiger_flags GFLAGS;
extern geiger_meaning GMEANING;
extern geiger_work GWORK;
extern geiger_mode GMODE;
extern NVRAM DevNVRAM;
extern geiger_ui GUI;

extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
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
	IncTick();
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
  /* USER CODE END EXTI1_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_1) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
    /* USER CODE BEGIN LL_EXTI_LINE_1 */
    if(DevNVRAM.GSETTING.ACTIVE_COUNTERS == 1 || DevNVRAM.GSETTING.ACTIVE_COUNTERS == 3){
    	if(GMODE.counter_mode==0){    //Режим поиска
    		if(GWORK.rad_buff[0]!=65535) GWORK.rad_buff[0]++;
    		if(GFLAGS.calculate_dose)if(++DevNVRAM.GSETTING.rad_sum>MAX_PARTICLES*3600/GWORK.real_geigertime) DevNVRAM.GSETTING.rad_sum=MAX_PARTICLES*3600/GWORK.real_geigertime; //общая сумма импульсов
    		if(GUI.page == 1 && !GFLAGS.do_alarm){ GFLAGS.is_detected = true; }
    	}else if(GMODE.counter_mode==1){							//Режим измерения активности
    		if(!GFLAGS.stop_timer) if(++GWORK.rad_back>MAX_PARTICLES*3600/GWORK.real_geigertime) GWORK.rad_back=MAX_PARTICLES*3600/GWORK.real_geigertime; //Сумма импульсов для режима измерения
    	}else if(GMODE.counter_mode==2){							//Режим измерения активности
    		if(GWORK.rad_buff[0]!=65535) GWORK.rad_buff[0]++;
    	    	GUI.update_required = true;
    	}
    }
    /* USER CODE END LL_EXTI_LINE_1 */
  }
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
  /* USER CODE END EXTI2_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_2) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);
    /* USER CODE BEGIN LL_EXTI_LINE_2 */
    if(DevNVRAM.GSETTING.ACTIVE_COUNTERS == 2 || DevNVRAM.GSETTING.ACTIVE_COUNTERS == 3){
    	if(GMODE.counter_mode==0){    //Режим поиска
    		if(GWORK.rad_buff[0]!=65535) GWORK.rad_buff[0]++;
    		if(GFLAGS.calculate_dose)if(++DevNVRAM.GSETTING.rad_sum>MAX_PARTICLES*3600/GWORK.real_geigertime) DevNVRAM.GSETTING.rad_sum=MAX_PARTICLES*3600/GWORK.real_geigertime; //общая сумма импульсов
    		if(GUI.page == 1 && !GFLAGS.do_alarm){ GFLAGS.is_detected = true; }
    	}else if(GMODE.counter_mode==1){							//Режим измерения активности
    		if(!GFLAGS.stop_timer) if(++GWORK.rad_back>MAX_PARTICLES*3600/GWORK.real_geigertime) GWORK.rad_back=MAX_PARTICLES*3600/GWORK.real_geigertime; //Сумма импульсов для режима измерения
    	}else if(GMODE.counter_mode==2){							//Режим измерения активности
    		if(GWORK.rad_buff[0]!=65535) GWORK.rad_buff[0]++;
    		GUI.update_required = true;
    	}
    }
    /* USER CODE END LL_EXTI_LINE_2 */
  }
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */
	if(DevNVRAM.GSETTING.ACTIVE_COUNTERS == 0){
	    if(GMODE.counter_mode==0){    //Режим поиска
	    	if(GWORK.rad_buff[0]!=65535) GWORK.rad_buff[0]++;
	    	if(GFLAGS.calculate_dose)if(++DevNVRAM.GSETTING.rad_sum>MAX_PARTICLES*3600/GWORK.real_geigertime) DevNVRAM.GSETTING.rad_sum=MAX_PARTICLES*3600/GWORK.real_geigertime; //общая сумма импульсов
	    	if(GUI.page == 1 && !GFLAGS.do_alarm){ GFLAGS.is_detected = true; }
	    }else if(GMODE.counter_mode==1){							//Режим измерения активности
	    	if(!GFLAGS.stop_timer) if(++GWORK.rad_back>MAX_PARTICLES*3600/GWORK.real_geigertime) GWORK.rad_back=MAX_PARTICLES*3600/GWORK.real_geigertime; //Сумма импульсов для режима измерения
	    }else if(GMODE.counter_mode==2){							//Режим измерения активности
	    	if(GWORK.rad_buff[0]!=65535) GWORK.rad_buff[0]++;
	    	GUI.update_required = true;
	    }
	}
  /* USER CODE END EXTI3_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_3) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_3);
    /* USER CODE BEGIN LL_EXTI_LINE_3 */

    /* USER CODE END LL_EXTI_LINE_3 */
  }
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupts.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */
	if(LL_ADC_IsActiveFlag_JEOS(ADC1) != 0) {
	    LL_ADC_ClearFlag_JEOS(ADC1);
	    battery_adc_value = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
	 }else if(LL_ADC_IsActiveFlag_JEOS(ADC2) != 0){
		 LL_ADC_ClearFlag_JEOS(ADC2);
		 high_voltage_adc_value = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);
	 }
  /* USER CODE END ADC1_2_IRQn 0 */

  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */
  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */
	//update_request();
	  if(LL_TIM_IsActiveFlag_UPDATE(TIM1)){
		  LL_TIM_ClearFlag_UPDATE(TIM1);
	  }

	uint64_t tmp_buff=0;

	if(GMODE.counter_mode == 0){
		for(uint8_t i=0; i<GWORK.real_geigertime; i++) tmp_buff+=GWORK.rad_buff[i]; //расчет фона мкР/ч
		if(tmp_buff>MAX_PARTICLES) tmp_buff=MAX_PARTICLES; //переполнение

		if(GFLAGS.particle_mode) GWORK.rad_back=tmp_buff/DevNVRAM.GSETTING.sensor_area;
		else GWORK.rad_back=tmp_buff;

		if(!GFLAGS.particle_mode) GWORK.stat_buff[GWORK.stat_time] = GWORK.rad_back; //Записываю текущее значение мкр/ч для расчёта погрешности

		Calculate_std(); //- crashing

		if(GWORK.rad_back>GWORK.rad_max) GWORK.rad_max=GWORK.rad_back;

		for(uint8_t k=GWORK.real_geigertime-1; k>0; k--) GWORK.rad_buff[k]=GWORK.rad_buff[k-1]; //перезапись массива

		GWORK.rad_buff[0]=0; //сбрасываем счетчик импульсов

		if(GWORK.stat_time > GWORK.real_geigertime) GWORK.stat_time = 0; //Счётчик для расчёта статистической погрешности
		else GWORK.stat_time++;

		GWORK.rad_dose=(DevNVRAM.GSETTING.rad_sum*GWORK.real_geigertime/3600); //расчитаем дозу

		GUI.mass[GUI.x_p]=map(GWORK.rad_back, 0, GWORK.rad_max < 40 ? 40 : GWORK.rad_max, 0, 19);
		//for(uint8_t i=0;i<96;i++) GUI.mass[i]=GUI.mass[i] * sqrt(GWORK.rad_back/GWORK.rad_max);
		if(GUI.x_p<95)GUI.x_p++;
		if(GUI.x_p==95){
			for(uint8_t i=0;i<LCD_X_SIZE-1;i++)GUI.mass[i]=GUI.mass[i+1];
		}
		if(GWORK.rad_max > 1) GWORK.rad_max--;		//Потихоньку сбрасываем максиму

	}else if(GMODE.counter_mode == 1){
		//ТАймер для второго режима. Обратный отсчёт
		//bool stop_timer = stop_timer;
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
	}else if(GMODE.counter_mode == 2){
		//Секундный замер, сбрасываем счётчик каждую секунду
		if(GWORK.rad_max < GWORK.rad_buff[0]) GWORK.rad_max = GWORK.rad_buff[0];
		GWORK.sum_old=GWORK.rad_buff[0];

		GUI.mass[GUI.x_p]=map(GWORK.rad_buff[0], 0, GWORK.rad_max < 40 ? 40 : GWORK.rad_max, 0, 19);
	    if(GUI.x_p<95)GUI.x_p++;
	    if(GUI.x_p==95){
	        for(uint8_t i=0;i<LCD_X_SIZE-1;i++)GUI.mass[i]=GUI.mass[i+1];
	    }

		if(GWORK.rad_max > 1) GWORK.rad_max--;		//Потихоньку сбрасываем максимум
			GWORK.rad_buff[0]=0; //сбрасываем счетчик импульсов
		}
  /* USER CODE END TIM1_UP_IRQn 0 */
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */
  if(!screen_saver_state) GUI.update_required = true;
  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles SPI2 global interrupt.
  */
void SPI2_IRQHandler(void)
{
  /* USER CODE BEGIN SPI2_IRQn 0 */

  /* USER CODE END SPI2_IRQn 0 */
  /* USER CODE BEGIN SPI2_IRQn 1 */

  /* USER CODE END SPI2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
