#ifndef __POWER_MANAGER__
#define __POWER_MANAGER__

void screen_saver_update();
void screen_saver_ticker();
void sleep();
void sleep_ticker();
void update_selected_counter();
void enter_to_sleep_mode();
void exit_sleep_mode();

void enter_to_stop_mode();
void exit_stop_mode();
void stop_mode();
void stop_ticker();

#endif
