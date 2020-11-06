/*
 * configuration.h
 *
 *  Created on: Aug 9, 2020
 *      Author: logot
 */

#ifndef INC_CONFIGURATION_H_
#define INC_CONFIGURATION_H_

#define VERSION "b1.0.0.0"

/************************SETTINGS***********************/

//Time before screen will turn off
#define SCREEN_SAVER_TIME 60000

/***********************CONSTANTS***********************/
//High voltage convertation coefficient
#define HV_COEF 6.5f

//Battery minimum and maximum values (min = 3.6v, max = 4.2v)
#define BAT_ADC_MIN 2234
#define BAT_ADC_MAX 2606

//#define DEBUG

#endif /* INC_CONFIGURATION_H_ */
