/*
 * output_manager.c
 *
 *  Created on: 15 авг. 2020 г.
 *      Author: VeAnIlAn
 */

#include "stdint.h"
#include "stdbool.h"

uint8_t page = 0;
uint8_t counter = 0;
uint8_t menu_page = 0;

uint16_t editable = 0;

bool editing_mode = false;
