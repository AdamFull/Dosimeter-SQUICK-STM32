/*
 * output_manager.h
 *
 *  Created on: 12 авг. 2020 г.
 *      Author: VeAnIlAn
 */

#ifndef INC_OUTPUT_MANAGER_H_
#define INC_OUTPUT_MANAGER_H_

//#include "stdbool.h"
#include "libs/LCD_1202.h"

void init_outputs();
void send_report();
void clear_screen();
void beep();
void draw_update();
void update_request();
void draw_logo();
void draw_main();
void draw_statusbar(const char**, bool*, uint32_t);
void draw_graph();
void draw_menu();
void draw_bat_low();
void battery_request(bool value);

#endif /* INC_OUTPUT_MANAGER_H_ */
