/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

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
FreeRTOS is a market leading RTOS from Real Time Engineers Ltd. that supports
31 architectures and receives 77500 downloads a year. It is professionally
developed, strictly quality controlled, robust, supported, and free to use in
commercial products without any requirement to expose your proprietary source
code.

This simple FreeRTOS demo does not make use of any IO ports, so will execute on
any Cortex-M3 of Cortex-M4 hardware.  Look for TODO markers in the code for
locations that may require tailoring to, for example, include a manufacturer
specific header file.

This is a starter project, so only a subset of the RTOS features are
demonstrated.  Ample source comments are provided, along with web links to
relevant pages on the http://www.FreeRTOS.org site.

Here is a description of the project's functionality:

The main() Function:
main() creates the tasks and software timers described in this section, before
starting the scheduler.

The Queue Send Task:
The queue send task is implemented by the prvQueueSendTask() function.
The task uses the FreeRTOS vTaskDelayUntil() and xQueueSend() API functions to
periodically send the number 100 on a queue.  The period is set to 200ms.  See
the comments in the function for more details.
http://www.freertos.org/vtaskdelayuntil.html
http://www.freertos.org/a00117.html

The Queue Receive Task:
The queue receive task is implemented by the prvQueueReceiveTask() function.
The task uses the FreeRTOS xQueueReceive() API function to receive values from
a queue.  The values received are those sent by the queue send task.  The queue
receive task increments the ulCountOfItemsReceivedOnQueue variable each time it
receives the value 100.  Therefore, as values are sent to the queue every 200ms,
the value of ulCountOfItemsReceivedOnQueue will increase by 5 every second.
http://www.freertos.org/a00118.html

An example software timer:
A software timer is created with an auto reloading period of 1000ms.  The
timer's callback function increments the ulCountOfTimerCallbackExecutions
variable each time it is called.  Therefore the value of
ulCountOfTimerCallbackExecutions will count seconds.
http://www.freertos.org/RTOS-software-timer.html

The FreeRTOS RTOS tick hook (or callback) function:
The tick hook function executes in the context of the FreeRTOS tick interrupt.
The function 'gives' a semaphore every 500th time it executes.  The semaphore
is used to synchronise with the event semaphore task, which is described next.

The event semaphore task:
The event semaphore task uses the FreeRTOS xSemaphoreTake() API function to
wait for the semaphore that is given by the RTOS tick hook function.  The task
increments the ulCountOfReceivedSemaphores variable each time the semaphore is
received.  As the semaphore is given every 500ms (assuming a tick frequency of
1KHz), the value of ulCountOfReceivedSemaphores will increase by 2 each second.

The idle hook (or callback) function:
The idle hook function queries the amount of free FreeRTOS heap space available.
See vApplicationIdleHook().

The malloc failed and stack overflow hook (or callback) functions:
These two hook functions are provided as examples, but do not contain any
functionality.
*/

/* Standard includes. */
#include <stdint.h>

/* Kernel includes. */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"

/* Priorities at which the tasks are created.  The event semaphore task is
given the maximum priority of ( configMAX_PRIORITIES - 1 ) to ensure it runs as
soon as the semaphore is given. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define	mainQUEUE_SEND_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define mainEVENT_SEMAPHORE_TASK_PRIORITY	( configMAX_PRIORITIES - 1 )

/* The rate at which data is sent to the queue, specified in milliseconds, and
converted to ticks using the portTICK_RATE_MS constant. */
#define mainQUEUE_SEND_PERIOD_MS			( 200 / portTICK_RATE_MS )

/* The period of the example software timer, specified in milliseconds, and
converted to ticks using the portTICK_RATE_MS constant. */
#define mainSOFTWARE_TIMER_PERIOD_MS		( 1000 / portTICK_RATE_MS )

/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define mainQUEUE_LENGTH					( 1 )

static xSemaphoreHandle xEventSemaphore = NULL;

/*-----------------------------------------------------------*/

/*
 * TODO: Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void prvSetupHardware( void );

typedef enum task_type {
	PERIODIC = 0,
	APERIODIC = 1
};

typedef enum scheduler_function {
	create = 0,
	delete = 1,
	return_overdue_list = 2,
	return_active_list = 3
} scheduler_function;

typedef struct task_param {
	TaskHandle_t t_handle;
	uint32_t release_time;
	uint32_t exec_time;
	uint32_t deadline;
	int priority;
	int task_type;
} task_param;

typedef struct dd_message {
	scheduler_function function_to_call;
	task_param task;
} dd_message;


typedef struct task_list {
	TaskHandle_t t_handle;
	uint32_t deadline;
	uint32_t task_type;
	uint32_t creation_time;
	struct task_list *next_cell;
	struct task_list *previous_cell;
} task_list;

typedef struct overdue_tasks {
	TaskHandle_t tid;
	uint32_t deadline;
	uint32_t task_type;
	uint32_t creation_time;
	struct overdue_tasks *next_cell; struct
	overdue_tasks *previous_cell;
} overdue_tasks;

/* Queues */
xQueueHandle DD_Scheduler_Queue = 0;

/* Tasks */
static void DD_Scheduler();
static void Task_Generator();
static void User_Tasks();
static void Monitor_Task();

static void dummy_task(uint32_t wait_time);


/* DD scheduler interface functions */
TaskHandle_t dd_tcreate(task_param task);
uint32_t dd_delete(TaskHandle_t t_handle);
uint32_t dd_return_active_list(struct task_list **list);
uint32_t dd_return_overdue_list(struct overdue_tasks **list);

/* aperiodic and periodic task generator helper functions */
task_param generate_periodic_task();
task_param generate_aperiodic_task();

/* linked list helper functions */
static void add_to_list(task_list *head_of_list, task_param task);

/*-----------------------------------------------------------*/

int main(void)
{
	prvSetupHardware();

	DD_Scheduler_Queue = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));

	xTaskCreate(DD_Scheduler, "DD_Scheduler", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(Task_Generator, "Task_Generator", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(User_Tasks, "User_Tasks", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(Monitor_Task, "Monitor_Task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	/* Start the tasks and timer running. */
	vTaskStartScheduler();
}

/* DD Scheduler interface functions */
TaskHandle_t dd_tcreate(task_param task) {
	xQueueHandle Task_Creator_Queue;
	Task_Creator_Queue = xQueueCreate(mainQUEUE_LENGTH, sizeof(dd_message));
	TaskHandle_t task_handle;

	xTaskCreate(dummy_task, "dummy_task", configMINIMAL_STACK_SIZE, NULL, 1, &task_handle);

	task.t_handle = task_handle;

	dd_message dd_scheduler_message;
	dd_scheduler_message.function_to_call = create;
	dd_scheduler_message.task = task;

	xQueueSend(Task_Creator_Queue, &dd_scheduler_message, 1000);
	xQueueSend(DD_Scheduler_Queue, &Task_Creator_Queue, 1000);


}

uint32_t dd_delete(TaskHandle_t t_handle) {

}

uint32_t dd_return_active_list(struct task_list **list) {

}

uint32_t dd_return_overdue_list(struct overdue_tasks **list) {

}


/* periodic and aperiodic task generators */

task_param generate_periodic_task() {
	struct task_param periodic_task;

	return periodic_task;
}

task_param generate_aperiodic_task() {
	task_param aperiodic_task;

	aperiodic_task.exec_time = 100;
	aperiodic_task.release_time = 0;
	aperiodic_task.deadline = 200;
	aperiodic_task.task_type = APERIODIC;

	return aperiodic_task;
}

static void add_to_list(task_list *head_of_list, task_param task) {
	task_list *task_to_add = (task_list*) malloc(sizeof(task_list));

	task_to_add->deadline = task.deadline;
	task_to_add->task_type = task.task_type;
	task_to_add->creation_time = task.release_time;
	task_to_add->next_cell = NULL;
	task_to_add->t_handle = task.t_handle;

	if (head_of_list == NULL) {
		head_of_list = task_to_add;
		head_of_list->previous_cell = NULL;
	}

	else {
		head_of_list->previous_cell = task_to_add;
		task_to_add->next_cell = head_of_list;
		head_of_list = task_to_add;
	}

}

static void dummy_task(uint32_t wait_time) {
	int j = 0;
	while(++j < wait_time);
}


/* FreeRTOS tasks */

static void DD_Scheduler() {
	xQueueHandle queue_handler;
	dd_message message;

	task_list *head_of_list = NULL;

	task_param task;
	scheduler_function func;

	while(1) {
		if (xQueueReceive(DD_Scheduler_Queue, &queue_handler, 500)) {
			if (xQueueReceive(queue_handler, &message, 500)) {

				switch(message.function_to_call) {
					case create:
						task = message.task;
						vTaskPrioritySet(task.t_handle, (UBaseType_t) 100/task.deadline);
					
						add_to_list(head_of_list, task);
						break;

					case delete:
						printf("delete");
						break;

					case return_overdue_list:
						printf("overdue");
						break;

					case return_active_list:
						printf("active");
						break;

				}
			}
		}
	}

}

static void Task_Generator() {
	while(1) {
		task_param aperiodic_task;

		aperiodic_task = generate_aperiodic_task();
		dd_tcreate(aperiodic_task);


		vTaskDelay(5000);
	}
}

static void User_Tasks() {

	while(1) {

	}
}

static void Monitor_Task() {

	while(1) {

	}

}

/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
static uint32_t ulCount = 0;

	/* The RTOS tick hook function is enabled by setting configUSE_TICK_HOOK to
	1 in FreeRTOSConfig.h.

	"Give" the semaphore on every 500th tick interrupt. */
	ulCount++;
	if( ulCount >= 500UL )
	{
		/* This function is called from an interrupt context (the RTOS tick
		interrupt),	so only ISR safe API functions can be used (those that end
		in "FromISR()".

		xHigherPriorityTaskWoken was initialised to pdFALSE, and will be set to
		pdTRUE by xSemaphoreGiveFromISR() if giving the semaphore unblocked a
		task that has equal or higher priority than the interrupted task.
		http://www.freertos.org/a00124.html */
		xSemaphoreGiveFromISR( xEventSemaphore, &xHigherPriorityTaskWoken );
		ulCount = 0UL;
	}

	/* If xHigherPriorityTaskWoken is pdTRUE then a context switch should
	normally be performed before leaving the interrupt (because during the
	execution of the interrupt a task of equal or higher priority than the
	running task was unblocked).  The syntax required to context switch from
	an interrupt is port dependent, so check the documentation of the port you
	are using.  http://www.freertos.org/a00090.html

	In this case, the function is running in the context of the tick interrupt,
	which will automatically check for the higher priority task to run anyway,
	so no further action is required. */
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* The malloc failed hook is enabled by setting
	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software 
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  pxCurrentTCB can be
	inspected in the debugger if the task name passed into this function is
	corrupt. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	FreeRTOSConfig.h.

	This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );

	/* TODO: Setup the clocks, etc. here, if they were not configured before
	main() was called. */
}

