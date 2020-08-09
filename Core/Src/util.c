/*
 * util.c
 *
 *  Created on: Aug 9, 2020
 *      Author: logot
 */

#include "util.h"

volatile unsigned long long micros_timer;
volatile unsigned long millis_timer;

unsigned long millis(){ return millis_timer; }
unsigned long long micros() { return micros_timer; }
