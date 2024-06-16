/*
 * DevRTOS.h
 *
 *  Created on: Jun 11, 2024
 *      Author: deveshshevde
 */

#include "main.h"
#ifndef INC_DEVRTOS_H_
#define INC_DEVRTOS_H_




typedef void (*threadHandler)();



/* TCB -> Thread Control Block */

typedef struct Threads
{

	void *stackPointer;

}thread_t;






void DEVOS_Init(void);

void DEVOS_Scheduler(void);

void PendSV_Handler(void) __attribute__((naked));

#endif /* INC_DEVRTOS_H_ */
