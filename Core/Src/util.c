/*
 * util.c
 *
 *  Created on: Aug 9, 2020
 *      Author: logot
 */

#include "util.h"
#include "stm32f1xx.h"

volatile unsigned long millis_timer;

#define STACK_CANARY_WORD (0xCACACACAUL)

/*void fill_stack(){
	volatile unsigned *top, *start;
	__asm__ volatile ("mov %[top], sp" : [top] "=r" (top) : : );
	start = &_ebss;
	while (start < top) {
	    *(start++) = STACK_CANARY_WORD;
	}
}

unsigned check_stack_size(void) {

    unsigned *addr = &_ebss;


    while ((addr < &_stack) && (*addr == STACK_CANARY_WORD)) {
        addr++;
    }

    return ((unsigned)&_stack - (unsigned)addr);
}*/

unsigned long millis(){ return millis_timer; }
