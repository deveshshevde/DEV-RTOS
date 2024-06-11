/*
 * DevRTOS.h
 *
 *  Created on: Jun 11, 2024
 *      Author: deveshshevde
 */

#ifndef INC_DEVRTOS_H_
#define INC_DEVRTOS_H_



/* TCB -> Thread Control Block */

typedef struct Threads
{

	void *stackPointer;

}thread_t;



typedef void (*threadHandler)();




#endif /* INC_DEVRTOS_H_ */
