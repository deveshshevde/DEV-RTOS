/*
 * DevRTOS.c
 *
 *  Created on: Jun 11, 2024
 *      Author: deveshshevde
 */

#include "DevRTOS.h"
#include <stdint.h>
#include <stdio.h>
#include <main.h>

#include "stm32f4xx_it.h"

#define NUMBER_OF_THREADS 32


int x;
thread_t * __IO currentThread;
thread_t * __IO nextThread;


extern void systick(void);

extern void LedOff(void);
extern void LedOn(void);


thread_t *AllThread[NUMBER_OF_THREADS];

uint32_t ThreadNumber;

uint16_t ThreadIndex;


void thread_Start(thread_t *me, threadHandler handler, void *stackstorage, uint32_t stacksize) {
    // Ensure stack pointer is 8-byte aligned and points to the top of the stack

    uint32_t *_sp = (uint32_t *)((((uint32_t)stackstorage + stacksize) / 8) * 8);

    // Initialize the stack frame for the new thread
    *(--_sp) = (1U << 24);  /* xPSR */
    *(--_sp) = (uint32_t)handler; /* PC */
    *(--_sp) = 0x0000000EU; /* LR  */
    *(--_sp) = 0x0000000CU; /* R12 */
    *(--_sp) = 0x00000003U; /* R3  */
    *(--_sp) = 0x00000002U; /* R2  */
    *(--_sp) = 0x00000001U; /* R1  */
    *(--_sp) = 0x00000000U; /* R0  */
    /* additionally, fake registers R4-R11 */
    *(--_sp) = 0x0000000BU; /* R11 */
    *(--_sp) = 0x0000000AU; /* R10 */
    *(--_sp) = 0x00000009U; /* R9 */
    *(--_sp) = 0x00000008U; /* R8 */
    *(--_sp) = 0x00000007U; /* R7 */
    *(--_sp) = 0x00000006U; /* R6 */
    *(--_sp) = 0x00000005U; /* R5 */
    *(--_sp) = 0x00000004U; /* R4 */
    // Set the stack pointer in the thread control block
    me->stackPointer = _sp;

    // Initialize the rest of the stack with a known pattern (optional)
    uint32_t * _stk_limit;
    _stk_limit = (uint32_t *)(((((uint32_t)stackstorage - 1U) / 8) + 1U) * 8);
    for (_sp = _sp - 1; _sp >= _stk_limit; --_sp) {
        *_sp = 0XEFBEADDE;  // Little-endian representation: 0xEFBEADDE
    }

    if(ThreadNumber < NUMBER_OF_THREADS){
    	AllThread[ThreadNumber] = me;
    	ThreadNumber++;
    }

    else{
    printf("Its all over");

    }

}




void DEVOS_Init()
{

	*(uint32_t volatile *)0xE000ED20 |= (0xFFU << 16);

}

void DEVOS_Scheduler()
{
	// Here we trigger the PSV interrupt to switch context if we get
	// next thread != current thread
	// here we have to prevent race condition because here we are managing multiple thread
	// which may use single resource

//	*(uint32_t volatile *)0xE000ED20 |= (0xFFU << 28);

	(ThreadIndex++);
	if(ThreadIndex = ((uint16_t)NUMBER_OF_THREADS)){
		ThreadIndex=0U;
	}
	nextThread = AllThread[ThreadIndex];


//	TODO: should uncomment sys();
//	sys();
	if(nextThread != currentThread)
	{

		SCB ->ICSR |= SCB_ICSR_PENDSVSET_Msk;
	}
}


__attribute__ ((naked))
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */
	x++;

	systick();
  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

//			asm volatile (
//			    // Load the address of current_tcb into R0 (R0 = &current_tcb)
//			    "LDR     R0, =currentThread\n"
//
//			    // Load contents of the memory address pointed to by R0 into R2
//			    // (R2 = current_tcb.stack_pointer)
//			    "LDR     R2, [R0]\n"
//
//			    // Load contents of the memory address pointed to by R2 into the SP
//			    // (SP = current_tcb->stack_pointer)
//			    "LDR     SP, [R2]\n"
//
//			    // Restore context from the thread's stack frame in reverse order
//			    // Restore R4-R11
//			    "POP     {R4-R11}\n"
//
//			    // Restore R0-R3
//			    "POP     {R0-R3}\n"
//
//			    // Restore R12
//			    "POP     {R12}\n"
//
//			    // Skip over the saved stack pointer's spot in the stack
//			    "ADD     SP, SP, #4\n"
//
//			    // Load LR with the return address (this should point to the thread's resume address)
//			    "POP     {LR}\n"
//
//			    // Skip over the saved stack pointer's spot in the stack again
//			    "ADD     SP, SP, #4\n"
//
//			    // Enable interrupts to allow SysTick
//			    "CPSIE   I\n"
//
//			    // Branch to the address in LR (return to the thread's execution)
//			    "BX      LR\n"
//			);


//
//	__asm volatile ("CPSID         I"); //ok
//	__asm volatile ("LDR           r1,=currentThread");//ok
//	__asm volatile ("LDR           r1,[r1,#0x00]");//ok
//	__asm volatile (" CMP           r1,#0 ");//ok
// 	__asm volatile ("BEQ           PendSV_restore"); //ok
//
//	__asm volatile ("PUSH          {r4-r11}    "); //ok
//
//
//	__asm volatile ("LDR           r1,=currentThread");
//	__asm volatile ("LDR           r1,[r1,#0x00]");
//	__asm volatile ("MOV           r0,sp");
//	__asm volatile ("STR           r0,[r1,#0x00]");
//
//
//
//	__asm volatile (  "PendSV_restore:                        ");
//	__asm   volatile (  "  LDR           r1,=nextThread         ");
//	__asm volatile("LDR           r1,[r1,#0x00]         ");
//	__asm volatile(	    "  LDR           r0,[r1,#0x00]          ");
//	__asm volatile(	    "   MOV           sp,r0                      ");
//
//	__asm volatile (  "  LDR           r1,=nextThread     ");
//	__asm volatile("    LDR           r1,[r1,#0x00]     ");
//	__asm volatile("   LDR           r2,=currentThread          ");
//	__asm volatile("   STR           r1,[r2,#0x00]                 ");
//
//
//	__asm volatile("    POP           {r4-r11}");
////	__asm volatile("     CPSIE         I                   ");
//
//	__asm volatile("BX            lr");
//
////
//
//	    "  LDR           r1,=currentThread       \n"
//	    "  LDR           r1,[r1,#0x00]     \n"
//	    "  MOV           r0,sp             \n"
//	    "  STR           r0,[r1,#0x00]     \n"
//	    /* } */
//
//	    "PendSV_restore:                   "
//	    "  LDR           r1,=nextThread       \n"
//	    "  LDR           r1,[r1,#0x00]     \n"
//	    "  LDR           r0,[r1,#0x00]     \n"
//	    "  MOV           sp,r0             \n"
//
//
//	    "  LDR           r1,=nextThread       \n"
//	    "  LDR           r1,[r1,#0x00]     \n"
//	    "  LDR           r2,=currentThread       \n"
//	    "  STR           r1,[r2,#0x00]     \n"
//
//
//                             // ARMv7-M or higher
//	    "  POP           {r4-r11}          \n"
//                      // ARMv7-M or highe
//	    /* __enable_irq(); */
//	    "  CPSIE         I                 \n"
//
//	    /* return to the next thread */
//	    "  BX            lr                "
//	    );
//
//
//             ARMv7-M or higher
//
//	    /* currentThread->stackPointer = sp; */
//	    "  LDR           r1,=currentThread       \n"
//	    "  LDR           r1,[r1,#0x00]          \n"
//	    "  MOV           r0,sp                  \n"
//	    "  STR           r0,[r1,#0x00]          \n"
//
//	    "PendSV_restore:                        \n"
//	    "  LDR           r1,=nextThread         \n"
//	    "  LDR           r1,[r1,#0x00]          \n"
//	    "  LDR           r0,[r1,#0x00]          \n"
//	    "  MOV           sp,r0                  \n"

//	    /* currentThread = nextThread; */
//	    "  LDR           r1,=nextThread         \n"
//	    "  LDR           r1,[r1,#0x00]          \n"
//	    "  LDR           r2,=currentThread      \n"
//	    "  STR           r1,[r2,#0x00]          \n"
//
//	    /* Pop registers r4-r11 */
//	#if (__ARM_ARCH == 6)               // if ARMv6-M...
//	    "  MOV           r0,sp                  \n" // r0 := top of stack
//	    "  MOV           r2,r0                  \n"
//	    "  ADDS          r2,r2,#(4*4)           \n" // point r2 to the 4 high registers r7-r11
//	    "  LDMIA         r2!,{r4-r7}            \n" // pop the 4 high registers into low registers
//	    "  MOV           r8,r4                  \n" // move low registers into high registers
//	    "  MOV           r9,r5                  \n"
//	    "  MOV           r10,r6                 \n"
//	    "  MOV           r11,r7                 \n"
//	    "  LDMIA         r0!,{r4-r7}            \n" // pop the low registers
//	    "  ADD           sp,sp,#(8*4)           \n" // remove 8 registers from the stack
//	#else                               // ARMv7-M or higher
//	    "  POP           {r4-r11}               \n"
//	#endif                              // ARMv7-M or higher
//
//	    /* __enable_irq(); */
//	    "  CPSIE         i                      \n"
//
//	    /* Ensure context restore synchronization */
//	    "  DSB                                  \n"
//	    "  ISB                                  \n"
//
//	    /* Return to the next thread */
//	    "  BX            lr                     \n"
//	);




  /* USER CODE END PendSV_IRQn 1 */
 }

