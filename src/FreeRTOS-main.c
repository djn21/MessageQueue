/*
 FreeRTOS V8.2.0 - Copyright (C) 2015 Real Time Engineers Ltd.
 All rights reserved

 VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

 This file is part of the FreeRTOS distribution.

 FreeRTOS is free software; you can redistribute it and/or modify it under
 the terms of the GNU General Public License (version 2) as published by the
 Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

 ***************************************************************************
 >>!   NOTE: The modification to the GPL is included to allow you to     !<<
 >>!   distribute a combined work that includes FreeRTOS without being   !<<
 >>!   obliged to provide the source code for proprietary components     !<<
 >>!   outside of the FreeRTOS kernel.                                   !<<
 ***************************************************************************

 FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
 WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 FOR A PARTICULAR PURPOSE.  Full license text is available on the following
 link: http://www.freertos.org/a00114.html

 ***************************************************************************
 *                                                                       *
 *    FreeRTOS provides completely free yet professionally developed,    *
 *    robust, strictly quality controlled, supported, and cross          *
 *    platform software that is more than just the market leader, it     *
 *    is the industry's de facto standard.                               *
 *                                                                       *
 *    Help yourself get started quickly while simultaneously helping     *
 *    to support the FreeRTOS project by purchasing a FreeRTOS           *
 *    tutorial book, reference manual, or both:                          *
 *    http://www.FreeRTOS.org/Documentation                              *
 *                                                                       *
 ***************************************************************************

 http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
 the FAQ page "My application does not run, what could be wrong?".  Have you
 defined configASSERT()?

 http://www.FreeRTOS.org/support - In return for receiving this top quality
 embedded software for free we request you assist our global community by
 participating in the support forum.

 http://www.FreeRTOS.org/training - Investing in training allows your team to
 be as productive as possible as early as possible.  Now you can receive
 FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
 Ltd, and the world's leading authority on the world's leading RTOS.

 http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
 including FreeRTOS+Trace - an indispensable productivity tool, a DOS
 compatible FAT file system, and our tiny thread aware UDP/IP stack.

 http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
 Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

 http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
 Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
 licenses offer ticketed support, indemnification and commercial middleware.

 http://www.SafeRTOS.com - High Integrity Systems also provide a safety
 engineered and independently SIL3 certified version for use in safety and
 mission critical applications that require provable dependability.

 1 tab == 4 spaces!
 */

/*
 * FreeRTOS-main.c (this file) defines a very simple demo that creates two tasks,
 * one queue, and one timer.
 *
 * The main() Function:
 * main() creates one software timer, one queue, and two tasks.  It then starts
 * the scheduler.
 *
 * The Software Timer:
 * The software timer is configured to be an "auto reset" timer.  Its callback
 * function simply increments the ulCallback variable each time it executes.
 */

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* BSP includes. */
#include "xtmrctr.h"

/* The following constants describe the timer instance used in this application.
 They are defined here such that a user can easily change all the needed parameters
 in one place. */
#define TIMER_DEVICE_ID						XPAR_TMRCTR_0_DEVICE_ID
#define TIMER_FREQ_HZ						XPAR_TMRCTR_0_CLOCK_FREQ_HZ
#define TIMER_INTR_ID						XPAR_INTC_0_TMRCTR_0_VEC_ID
/*-----------------------------------------------------------*/

/*
 * The tasks as described in the comments at the top of this file.
 */
static void task1Method(void *pvParameters);
static void task2Method(void *pvParameters);
static void task3Method(void *pvParameters);

/*
 * Reverse bits order.
 */
char reverseBits(char);

/* Structures that hold the state of the various peripherals used by this demo.
 These are used by the Xilinx peripheral driver API functions. */
static XTmrCtr xTimer0Instance;

static QueueHandle_t xQueue;

static volatile char* SWITCHes = (char*) XPAR_DIP_SWITCHES_BASEADDR;
static volatile char* LEDs = (char*) XPAR_LEDS_BASEADDR;

struct AMessage {
	char taskID;
	char value;
};

int main(void) {
	/***************************************************************************
	 See http://www.FreeRTOS.org for full information on FreeRTOS, including
	 an API reference, pdf API reference manuals, and FreeRTOS tutorial books.

	 See http://www.freertos.org/Free-RTOS-for-Xilinx-MicroBlaze-on-Spartan-6-FPGA.html
	 for comprehensive standalone FreeRTOS for MicroBlaze demos.
	 ***************************************************************************/

	/* Create a queue capable of containing of containing 10 AMessage pointer values. */
	xQueue = xQueueCreate(10, sizeof(struct AMessage *));

	/* Initialize LEDs */
	*LEDs = 0x00;

	/* Start the three tasks as described in the comments at the top of this
	 file. */
	xTaskCreate(task1Method, "task1", configMINIMAL_STACK_SIZE, NULL,
			tskIDLE_PRIORITY, NULL);
	xTaskCreate(task2Method, "task2", configMINIMAL_STACK_SIZE, NULL,
			tskIDLE_PRIORITY, NULL);
	xTaskCreate(task3Method, "task3", configMINIMAL_STACK_SIZE, NULL,
			tskIDLE_PRIORITY, NULL);

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	/* Delete queue */
	vQueueDelete(xQueue);

	/* If all is well, the scheduler will now be running, and the following line
	 will never be reached.  If the following line does execute, then there was
	 insufficient FreeRTOS heap memory available for the idle and/or timer tasks
	 to be created.  See the memory management section on the FreeRTOS web site
	 for more details. */
	for (;;)
		;
}
/*-----------------------------------------------------------*/

static void task1Method(void *pvParameters) {
	struct AMessage xSendMessage;
	struct AMessage *pxSendMessage = &xSendMessage;
	char oldValue = 0x00;
	while (1) {
		/* If SWITCHes changed */
		if (oldValue != *SWITCHes) {
			oldValue = *SWITCHes;
			/* Set message for second task */
			pxSendMessage->taskID = 2;
			pxSendMessage->value = oldValue;
			/* Send message to second task  */
			xQueueSend(xQueue, (void* )&pxSendMessage, (TickType_t ) 0);
		}
	}
}
/*-----------------------------------------------------------*/

static void task2Method(void *pvParameters) {
	struct AMessage xSendMessage;
	struct AMessage *pxSendMessage = &xSendMessage;
	struct AMessage *pxReceiveMessage;
	while (1) {
		/* Get message from queue */
		if (xQueuePeek(xQueue, &(pxReceiveMessage), (TickType_t) 0)) {
			/* Check if message sent to second task */
			if (pxReceiveMessage->taskID == 2) {
				/* Set message for third task */
				pxSendMessage->taskID = 3;
				pxSendMessage->value = reverseBits(pxReceiveMessage->value);
				/* Remove message from queue */
				xQueueReceive(xQueue, NULL, (TickType_t ) 0);
				/* Send message to third task */
				xQueueSend(xQueue, (void* )&pxSendMessage, (TickType_t ) 0);
			}
		}
	}
}
/*----------------------------------------------------------*/

static void task3Method(void *pvParameters) {
	struct AMessage *pxReceiveMessage;
	while (1) {
		/* Get message from queue */
		if (xQueuePeek(xQueue, &(pxReceiveMessage), (TickType_t ) 0)) {
			/* Check if message sent to third task */
			if (pxReceiveMessage->taskID == 3) {
				/* Set LEDs */
				*LEDs = pxReceiveMessage->value;
				/* Remove message from queue */
				xQueueReceive(xQueue, NULL, (TickType_t ) 0);
			}
		}
	}
}
/*-----------------------------------------------------------*/

char reverseBits(char param) {
	char ret = 0;
	char i;
	for (i = 0; i < sizeof(param) * 8; i++) {
		if ((param & (1 << i)))
			ret |= 1 << ((sizeof(param) * 8 - 1) - i);
	}
	return ret;
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void) {
	/* vApplicationMallocFailedHook() will only be called if
	 configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	 function that will get called if a call to pvPortMalloc() fails.
	 pvPortMalloc() is called internally by the kernel whenever a task, queue or
	 semaphore is created.  It is also called by various parts of the demo
	 application.  If heap_1.c or heap_2.c are used, then the size of the heap
	 available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	 FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	 to query the size of free heap space that remains (although it does not
	 provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for (;;)
		;
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(TaskHandle_t *pxTask,
		signed char *pcTaskName) {
	(void) pcTaskName;
	(void) pxTask;

	/* vApplicationStackOverflowHook() will only be called if
	 configCHECK_FOR_STACK_OVERFLOW is set to either 1 or 2.  The handle and name
	 of the offending task will be passed into the hook function via its
	 parameters.  However, when a stack has overflowed, it is possible that the
	 parameters will have been corrupted, in which case the pxCurrentTCB variable
	 can be inspected directly. */taskDISABLE_INTERRUPTS();
	for (;;)
		;
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook(void) {
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	 to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	 task.  It is essential that code added to this hook function never attempts
	 to block in any way (for example, call xQueueReceive() with a block time
	 specified, or call vTaskDelay()).  If the application makes use of the
	 vTaskDelete() API function (as this demo application does) then it is also
	 important that vApplicationIdleHook() is permitted to return to its calling
	 function, because it is the responsibility of the idle task to clean up
	 memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationTickHook(void) {
	/* vApplicationTickHook() will only be called if configUSE_TICK_HOOK is set
	 to 1 in FreeRTOSConfig.h.  It executes from an interrupt context so must
	 not use any FreeRTOS API functions that do not end in ...FromISR().

	 This simple blinky demo does not use the tick hook, but a tick hook is
	 required to be defined as the blinky and full demos share a
	 FreeRTOSConfig.h header file. */
}
/*-----------------------------------------------------------*/

/* This is an application defined callback function used to install the tick
 interrupt handler.  It is provided as an application callback because the kernel
 will run on lots of different MicroBlaze and FPGA configurations - there could
 be multiple timer instances in the hardware platform and the users can chose to
 use any one of them. This example uses Timer 0. If that is available in  your
 hardware platform then this example callback implementation should not require
 modification. The definitions for the timer instance used are at the top of this
 file so that users can change them at one place based on the timer instance they
 use. The name of the interrupt handler that should be installed is vPortTickISR(),
 which the function below declares as an extern. */
void vApplicationSetupTimerInterrupt(void) {
	portBASE_TYPE xStatus;
	const unsigned char ucTimerCounterNumber = (unsigned char) 0U;
	const unsigned long ulCounterValue = ((TIMER_FREQ_HZ / configTICK_RATE_HZ)
			- 1UL);
	extern void vPortTickISR(void *pvUnused);

	/* Initialise the timer/counter. */
	xStatus = XTmrCtr_Initialize(&xTimer0Instance, TIMER_DEVICE_ID);

	if (xStatus == XST_SUCCESS) {
		/* Install the tick interrupt handler as the timer ISR.
		 *NOTE* The xPortInstallInterruptHandler() API function must be used for
		 this purpose. */
		xStatus = xPortInstallInterruptHandler(TIMER_INTR_ID, vPortTickISR,
				NULL );
	}

	if (xStatus == pdPASS ) {
		/* Enable the timer interrupt in the interrupt controller.
		 *NOTE* The vPortEnableInterrupt() API function must be used for this
		 purpose. */
		vPortEnableInterrupt(TIMER_INTR_ID);

		/* Configure the timer interrupt handler. */
		XTmrCtr_SetHandler(&xTimer0Instance, (void *) vPortTickISR, NULL );

		/* Set the correct period for the timer. */
		XTmrCtr_SetResetValue(&xTimer0Instance, ucTimerCounterNumber,
				ulCounterValue);

		/* Enable the interrupts.  Auto-reload mode is used to generate a
		 periodic tick.  Note that interrupts are disabled when this function is
		 called, so interrupts will not start to be processed until the first
		 task has started to run. */
		XTmrCtr_SetOptions(&xTimer0Instance, ucTimerCounterNumber,
				(XTC_INT_MODE_OPTION | XTC_AUTO_RELOAD_OPTION
						| XTC_DOWN_COUNT_OPTION));

		/* Start the timer. */
		XTmrCtr_Start(&xTimer0Instance, ucTimerCounterNumber);
	}

	/* Sanity check that the function executed as expected. */
	configASSERT( ( xStatus == pdPASS ) );
}
/*-----------------------------------------------------------*/

/* This is an application defined callback function used to clear whichever
 interrupt was installed by the the vApplicationSetupTimerInterrupt() callback
 function - in this case the interrupt generated by the AXI timer.  It is
 provided as an application callback because the kernel will run on lots of
 different MicroBlaze and FPGA configurations - not all of which will have the
 same timer peripherals defined or available.  This example uses the AXI Timer 0.
 If that is available on your hardware platform then this example callback
 implementation should not require modification provided the example definition
 of vApplicationSetupTimerInterrupt() is also not modified. */
void vApplicationClearTimerInterrupt(void) {
	unsigned long ulCSR;

	/* Clear the timer interrupt */
	ulCSR = XTmrCtr_GetControlStatusReg( XPAR_TMRCTR_0_BASEADDR, 0 );
	XTmrCtr_SetControlStatusReg(XPAR_TMRCTR_0_BASEADDR, 0, ulCSR);
}
/*-----------------------------------------------------------*/

