/*
 * GyverButton_stm32.c
 *
 *  Created on: 13 авг. 2020 г.
 *      Author: VeAnIlAn
 */

#include "libs/GyverButton_stm32.h"
#include "util.h"

// ==================== CONSTRUCTOR ====================
void gbuttonInit(GyverButton* btn, GPIO_TypeDef* BTNPORT, uint16_t BTNPIN, bool type, bool dir){
	btn->BTNPORT = BTNPORT;
	btn->BTNPIN = BTNPIN;

	btn->btn_state = btn->btn_flag = false;
	btn->btn_counter = btn->last_counter = btn->last_hold_counter = 0;
	btn->btn_timer = 0;

	btn->_debounce = 50;
	btn->_timeout = 500;
	btn->_click_timeout = 500;

	btn->flags.noPin = false;
	btn->flags.type = type;
	btn->flags.mode = false;
	btn->flags.tickMode = false;
	btn->flags.inv_state = dir;
}

// ==================== SET ====================
void setDebounce(GyverButton* btn, uint16_t debounce) {
	btn->_debounce = debounce;
}
void setTimeout(GyverButton* btn, uint16_t new_timeout) {
	btn->_timeout = new_timeout;
}
void setClickTimeout(GyverButton* btn, uint16_t new_timeout) {
	btn->_click_timeout = new_timeout;
}

void setDirection(GyverButton* btn, bool dir) {
	btn->flags.inv_state = dir;
}
void setTickMode(GyverButton* btn, bool tickMode) {
	btn->flags.tickMode = tickMode;
}

// ==================== IS ====================
bool isPress(GyverButton* btn) {
	if (btn->flags.tickMode) tick(btn);
	if (btn->flags.isPress_f) {
		btn->flags.isPress_f = false;
		return true;
	} else return false;
}
bool isRelease(GyverButton* btn) {
	if (btn->flags.tickMode) tick(btn);
	if (btn->flags.isRelease_f) {
		btn->flags.isRelease_f = false;
		return true;
	} else return false;
}
bool isClick(GyverButton* btn) {
	if (btn->flags.tickMode) tick(btn);
	if (btn->flags.isOne_f) {
		btn->flags.isOne_f = false;
		return true;
	} else return false;
}
bool isHolded(GyverButton* btn) {
	if (btn->flags.tickMode) tick(btn);
	if (btn->flags.isHolded_f) {
		btn->flags.isHolded_f = false;
		return true;
	} else return false;
}
bool isHold(GyverButton* btn) {
	if (btn->flags.tickMode) tick(btn);
	if (btn->flags.step_flag) return true;
	else return false;
}

void resetStates(GyverButton* btn) {
	btn->flags.isPress_f = false;
	btn->flags.isRelease_f = false;
	btn->flags.isOne_f = false;
	btn->flags.isHolded_f = false;
	btn->flags.step_flag = false;
	btn->last_hold_counter = 0;
	btn->last_counter = 0;
}

// ==================== TICK ====================
void ttick(GyverButton* btn, bool state) {
	btn->flags.mode = true;
	btn->btn_state = state ^ btn->flags.inv_state;
	tick(btn);
	btn->flags.mode = false;
}

void tick(GyverButton* btn) {
	// читаем пин
	bool readed_state = (bool)(btn->BTNPORT->IDR & btn->BTNPIN);
	if (!btn->flags.mode && !btn->flags.noPin) btn->btn_state = !readed_state ^ (btn->flags.inv_state ^ btn->flags.type);

	uint32_t thisMls = millis();

	// нажатие
	if (btn->btn_state && !btn->btn_flag) {
		if (!btn->flags.btn_deb) {
			btn->flags.btn_deb = true;
			btn->btn_timer = thisMls;
		} else {
			if (thisMls - btn->btn_timer >= btn->_debounce) {
				btn->btn_flag = true;
				btn->flags.isPress_f = true;
				btn->flags.oneClick_f = true;
			}
		}
	} else {
		btn->flags.btn_deb = false;
	}

	// отпускание
	if (!btn->btn_state && btn->btn_flag) {
		btn->btn_flag = false;
		if (!btn->flags.hold_flag) btn->btn_counter++;
		btn->flags.hold_flag = false;
		btn->flags.isRelease_f = true;
		btn->btn_timer = thisMls;
		if (btn->flags.step_flag) {
			btn->last_counter = 0;
			btn->btn_counter = 0;
			btn->flags.step_flag = false;
		}
		if (btn->flags.oneClick_f) {
			btn->flags.oneClick_f = false;
			btn->flags.isOne_f = true;
		}
	}

	// кнопка удерживается
	if (btn->btn_flag && btn->btn_state && (thisMls - btn->btn_timer >= btn->_timeout) && !btn->flags.hold_flag) {
		btn->flags.hold_flag = true;
		btn->last_hold_counter = btn->btn_counter;
		//btn_counter = 0;
		//last_counter = 0;
		btn->flags.isHolded_f = true;
		btn->flags.step_flag = true;
		btn->flags.oneClick_f = false;
		btn->btn_timer = thisMls;
	}
}
