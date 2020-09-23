/*
 * util.h
 *
 *  Created on: Aug 9, 2020
 *      Author: logot
 */

#ifndef INC_UTIL_H_
#define INC_UTIL_H_
void IncTick();
unsigned long GetTick();
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
long map(long x, long in_min, long in_max, long out_min, long out_max);
float  lerp ( float  v0 ,  float  v1 ,  float  t );
void ftoa(float n, char* res, int afterpoint);

#endif /* INC_UTIL_H_ */
