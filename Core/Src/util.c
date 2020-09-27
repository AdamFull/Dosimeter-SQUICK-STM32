/*
 * util.c
 *
 *  Created on: Aug 9, 2020
 *      Author: logot
 */

#include "util.h"
#include "math.h"
#include "stm32f103xb.h"

extern volatile unsigned long ticks;


uint32_t getUs(){
	uint32_t usTicks = 48000000 / 1000000;
	register uint32_t ms, cycle_cnt;

	do {
		ms = GetTick();
		cycle_cnt = SysTick->VAL;
	} while (ms != GetTick());


	return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}
void delayUs(uint16_t micros) {
	uint32_t start = getUs();
	while (getUs()-start < (uint32_t) micros) { asm("nop"); }
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

long map(long x, long in_min, long in_max, long out_min, long out_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void IncTick(){
	ticks += 1U;
}

unsigned long GetTick(){
	return ticks;
}

float lerp( float v0,  float v1, float  t){
  return  (1 - t) * v0 + t * v1;
}

void reverse(char* str, int len){
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

int intToStr(int x, char str[], int d){
    int i = 0;
    while (x) {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }
    while (i < d)
        str[i++] = '0';
    reverse(str, i);
    str[i] = '\0';
    return i;
}

void ftoa(float n, char* res, int afterpoint){
    int ipart = (int)n;
    float fpart = n - (float)ipart;
    int i = intToStr(ipart, res, 0);
    if (afterpoint != 0) {
        res[i] = '.'; // add dot
        fpart = fpart * pow(10, afterpoint);
        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}
