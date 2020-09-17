/*
 * GyverButton_stm32.h
 *
 *  Created on: 13 авг. 2020 г.
 *      Author: VeAnIlAn
 */

#ifndef INC_LIBS_GYVERBUTTON_STM32_H_
#define INC_LIBS_GYVERBUTTON_STM32_H_

#include "stdbool.h"
#include "stdint.h"
#include "stm32f1xx_ll_gpio.h"

/*
	GyverButton - библиотека для многофункциональной отработки нажатия кнопки. Возможности:
	- Работа с нормально замкнутыми и нормально разомкнутыми кнопками
	- Работа с подключением PULL_UP и PULL_DOWN
	- Опрос кнопки с программным антидребезгом контактов (настраиваемое время)
	- Отработка нажатия, удерживания, отпускания, клика по кнопке (+ настройка таймаутов)
	- Отработка одиночного, двойного и тройного нажатия (вынесено отдельно)
	- Отработка любого количества нажатий кнопки (функция возвращает количество нажатий)
	- Функция изменения значения переменной с заданным шагом и заданным интервалом по времени
	- Возможность работы с "виртуальными" кнопками (все возможности библиотеки используются для матричных и резистивных клавиатур)

	Документацию читай здесь: https://alexgyver.ru/gyverbutton/
	Для максимально быстрого опроса кнопки рекомендуется использовать ядро GyverCore https://alexgyver.ru/gyvercore/

	Версия 2.15: Добавлена возможность объявить кнопку без привязки к пину
	Версия 3.0: Ускорен и оптимизирован код, переделана инициализация, дополнены примеры
	Версия 3.1: isStep может принимать количество кликов, сделанных перед ним (см. пример clicks_step)
	Версия 3.2: Добавлен метод getHoldClicks() - вернуть количество кликов, предшествующее удерживанию
	Версия 3.3: Мелкие исправления
	Версия 3.4: Добавлен метод resetStates(), сбрасывающий состояния и счётчики
*/

#pragma pack(push,1)
typedef struct {
	bool btn_deb: 1;
	bool hold_flag: 1;
	bool isHolded_f: 1;
	bool isRelease_f: 1;
	bool isPress_f: 1;
	bool step_flag: 1;
	bool oneClick_f: 1;
	bool isOne_f: 1;
	bool inv_state: 1;
	bool mode: 1;
	bool type: 1;
	bool tickMode: 1;
	bool noPin: 1;
} GyverButtonFlags;
#pragma pack(pop)

typedef struct {
	GPIO_TypeDef* BTNPORT;
	uint16_t BTNPIN;

	bool btn_state;
	bool btn_flag;

	uint8_t btn_counter, last_counter, last_hold_counter;
	uint32_t btn_timer;

	GyverButtonFlags flags;
	uint16_t _debounce;
	uint16_t _timeout;
	uint16_t _click_timeout;
} GyverButton;

#define BTN_NO_PIN -1
#define HIGH_PULL 0
#define LOW_PULL 1
#define NORM_OPEN 0
#define NORM_CLOSE 1
#define MANUAL 0
#define AUTO 1

// Варианты инициализации:
// GButton btn;							// без привязки к пину (виртуальная кнопка) и без указания типа (по умолч. HIGH_PULL и NORM_OPEN)
// GButton btn(пин);					// с привязкой к пину и без указания типа (по умолч. HIGH_PULL и NORM_OPEN)
// GButton btn(пин, тип подключ.);		// с привязкой к пину и указанием типа подключения (HIGH_PULL / LOW_PULL) и без указания типа кнопки (по умолч. NORM_OPEN)
// GButton btn(пин, тип подключ., тип кнопки);			// с привязкой к пину и указанием типа подключения (HIGH_PULL / LOW_PULL) и типа кнопки (NORM_OPEN / NORM_CLOSE)
// GButton btn(BTN_NO_PIN, тип подключ., тип кнопки);	// без привязки к пину и указанием типа подключения (HIGH_PULL / LOW_PULL) и типа кнопки (NORM_OPEN / NORM_CLOSE)

void gbuttonInit(GyverButton* btn, GPIO_TypeDef* BTNPORT, uint16_t BTNPIN, bool type, bool dir);

void setDebounce(GyverButton* btn, uint16_t debounce);				// установка времени антидребезга (по умолчанию 80 мс)
void setTimeout(GyverButton* btn, uint16_t new_timeout);				// установка таймаута удержания (по умолчанию 300 мс)
void setClickTimeout(GyverButton* btn, uint16_t new_timeout);			// установка таймаута между кликами (по умолчанию 500 мс)
void setStepTimeout(GyverButton* btn, uint16_t step_timeout);			// установка таймаута между инкрементами (по умолчанию 400 мс)
//void setType(GyverButton* btn, bool type);							// установка типа кнопки (HIGH_PULL - подтянута к питанию, LOW_PULL - к gnd)
void setDirection(GyverButton* btn, bool dir);						// установка направления (разомкнута/замкнута по умолчанию - NORM_OPEN, NORM_CLOSE)

void setTickMode(GyverButton* btn, bool tickMode);					// (MANUAL / AUTO) ручной или автоматический опрос кнопки функцией tick()
														// MANUAL - нужно вызывать функцию tick() вручную
														// AUTO - tick() входит во все остальные функции и опрашивается сама

void tick(GyverButton* btn);										// опрос кнопки
void ttick(GyverButton* btn, bool state);							// опрос внешнего значения (0 нажато, 1 не нажато) (для матричных, резистивных клавиатур и джойстиков)

//void resetFlags();

bool isPress(GyverButton* btn);		// возвращает true при нажатии на кнопку. Сбрасывается после вызова
bool isRelease(GyverButton* btn);	// возвращает true при отпускании кнопки. Сбрасывается после вызова
bool isClick(GyverButton* btn);		// возвращает true при клике. Сбрасывается после вызова
bool isHolded(GyverButton* btn);		// возвращает true при удержании дольше timeout. Сбрасывается после вызова
bool isHold(GyverButton* btn);		// возвращает true при нажатой кнопке, не сбрасывается
//bool state(GyverButton* btn);		// возвращает состояние кнопки

void resetStates(GyverButton* btn);		// сбрасывает все is-флаги и счётчики

#endif /* INC_LIBS_GYVERBUTTON_STM32_H_ */
