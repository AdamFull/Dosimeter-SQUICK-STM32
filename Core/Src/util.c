/*
 * util.c
 *
 *  Created on: Aug 9, 2020
 *      Author: logot
 */

#include "util.h"

volatile unsigned long millis_timer;

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

long map(long x, long in_min, long in_max, long out_min, long out_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

unsigned long millis(){ return millis_timer; }
