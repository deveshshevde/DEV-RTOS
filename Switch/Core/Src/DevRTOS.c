/*
 * DevRTOS.c
 *
 *  Created on: Jun 11, 2024
 *      Author: deveshshevde
 */

#include "DevRTOS.h"
#include <stdint.h>

void thread_Start(thread_t *me ,
				  threadHandler handler,
				   void *stackstorage ,
				   uint32_t stacksize)
{
	//stack pointer is going upside down in arm and also ensuring 8 byte allingment
	uint32_t *_sp = (uint32_t*) ((((uint32_t)stackstorage + stacksize)/8) *8 );

	//
	uint32_t *_stk_limit;
	*(--_sp) = (1U << 24) ;// enabling thumb instruction bit
	*(--_sp) = (uint32_t)&handler; // location of the func loaded

	me->stackPointer = _sp;


	_stk_limit = (uint32_t*) (((((uint32_t)stackstorage - 1)/8) +1 ) *8 );

	for(_sp = _sp - 1 ; _sp >= _stk_limit ; --_sp){
		*_sp = 0xDEADBEEFU;
	}
}
