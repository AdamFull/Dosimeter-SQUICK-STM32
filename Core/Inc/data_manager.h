/*
 * data_manager.h
 *
 *  Created on: Aug 9, 2020
 *      Author: logot
 */

#ifndef INC_DATA_MANAGER_H_
#define INC_DATA_MANAGER_H_

#include "stdbool.h"

typedef const char* string;

typedef enum {
	NO_ERROR,
	FLASH_MEMORY_ERROR,

} DMGRESULT;

void Initialize_variables();
void Initialize_data();
void Update_rad_buffer();

void Save_dose();
void Save_tone();
void Save_backlight();
void Save_contrast();
void Save_geiger_time();
void Save_geiger_error();
void Save_dose_save_interval();
void Save_alarm_threshold();
void Reset_to_defaults();

bool Init_memory();
bool Setup_memory();
string Read_memory(string file_name);
bool Write_memory(string file_name, string file_data);
bool is_memory_valid();

void Reset_dose();

void Reset_activity_test();

volatile void Calculate_std();

#endif /* INC_DATA_MANAGER_H_ */
