/*
 * output_manager.h
 *
 *  Created on: 12 авг. 2020 г.
 *      Author: VeAnIlAn
 */

#ifndef INC_OUTPUT_MANAGER_H_
#define INC_OUTPUT_MANAGER_H_

#include "stdbool.h"

void draw_update();
void update_request();
void draw_logo();
void draw_main();
void draw_simple_menu_page(const char** punct_names, size_t punct_count);
void draw_editable_menu_page(const char** punct_names, uint8_t* punct_values, char* postfixes, size_t punct_count);
void draw_checkbox_menu_page(const char** punct_names, bool* punct_values, bool* skip_flags, size_t punct_count);
void draw_graph();
void draw_menu();
void draw_bat_low();
void battery_request(bool value);

#endif /* INC_OUTPUT_MANAGER_H_ */
