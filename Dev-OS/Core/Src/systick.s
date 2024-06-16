

.extern currentThread
.extern nextThread

.type   systick, %function
.global systick



.type   sys, %function
.global sys

.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb



systick:


	CPSID         I
	LDR           r1,=currentThread
 	LDR           r1,[r1,#0x00]
 	CBZ           r1,PendSV_restore

 	PUSH          {r4-r11}

	 LDR           r1,=currentThread
	 LDR           r1,[r1,#0x00]
	 STR           sp,[r1,#0x00]

	PendSV_restore:
		LDR           r1,=nextThread
	   	LDR           r1,[r1,#0x00]
	    LDR            sp,[r1,#0x00]


	LDR           r1,=nextThread
	LDR           r1,[r1,#0x00]
	LDR           r2,=currentThread
	STR           r1,[r2,#0x00]



	POP     {r4-r11}
	POP     {R0-R3}              // order they are saved: stack frame
    POP     {R12}
                      // ARMv7-M or highe
	    /* __enable_irq(); */
	CPSIE         I

	    /* return to the next thread */
	BX            lr




sys:
  CPSID   I                   // Disable interrupts as this is a
                                        // critical code section

            PUSH    {R4-R11}            // Save R4-11 by pushing onto stack
                                        // as they are not in exception stack
                                        // frame

            LDR     R0, =currentThread    // Load address of *current_tcb
                                        // (R0 = &current_tcb)

            LDR     R1, [R0]            // Load contents of address pointed to
                                        // by R0 into R1 which will be the
                                        // address of SP
                                        // (R1 = &current_tcb.stack_pointer)

            STR     SP, [R1]            // Store SP register in address pointed
                                        // to by R1. Address pointed to by R1
                                        // is the current_tcb.stack_pointer
                                        // (the SP has changed since
                                        // the thread started executing)
                                        // (current_tcb->stack_pointer = SP)

            LDR     R1, [R1, #4]        // Load contents of address pointed to
                                        // by R1 + 4 bytes into R1. This loads
                                        // current_tcb->next_tcb into memory
                                        // (int32_t = 4 bytes= .next_tcb size)
                                        // (R1 = current_tcb->next_tcb)

            STR     R1, [R0]            // Store contents of R1 into the
                                        // address pointed to by R0. R0
                                        // holds &current_tcb meaning
                                        // we are scheduling the next thread
                                        // to execute by updating *current_tcb.
                                        // (current_tcb = current_tcb->next_tcb)

            LDR     SP, [R1]            // Load contents of address pointed to
                                        // in R1 into SP. R1 currently points
                                        // to current_tcb.next_tcb which is
                                        // the address of the next TCB. The
                                        // first element is stack_pointer
                                        // (SP = current_tcb->next_tcb)
                                        // (SP = next_tcb.stack_pointer)

            POP     {R4-R11}            // Restore R4 - R11 registers manaully
                                        // from next task's stack as they
                                        // are not in the exception stack frame

            CPSIE   I                   // Enable interrupts as critical
                                        // code is done and SysTick is needed

            BX      LR                   // Branch to next task

                                         // At this point, exception stack
                                         // frame is popped from the stack
