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
#include <stdio.h>
#include <time.h>
#include <stm32f4_discovery.h>
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
#define mainQUEUE_LENGTH					( 1000 )
#define minQUEUE_LENGTH					( 10 )


static xSemaphoreHandle xEventSemaphore = NULL;

/* --------------------------------------------------------- */

static void prvSetupHardware( void );
static void set_up_input_button();

GPIO_InitTypeDef GPIO_InitDef; //Where GPIO_InitDef is variable to work with struct

#define amber_led	LED3 //medium priority
#define green_led	LED4 //lowest priority
#define red_led		LED5 //highest priority
#define blue_led	LED6 //idle priority

#define task1_period 500
#define task2_period 500
#define task3_period 500

#define task1_exec 100
#define task2_exec 200
#define task3_exec 200

#define task1_led amber_led; // amber and green have the high priorities++
#define task2_led red_led;
#define task3_led blue_led;

// response codes for dd_tcreate and dd_delete
typedef enum dd_scheduler_status_code {
	STATUS_OKAY = 1,
	STATUS_FAILURE = 0
};

// function codes for dd sched interface functions
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
	int led;
} task_param;

typedef struct task_list {
	TaskHandle_t t_handle;
	uint32_t deadline;
	uint32_t task_type;
	uint32_t creation_time;
	struct task_list *next_cell;
	int led;
} task_list;

typedef struct overdue_tasks {
	TaskHandle_t tid;
	uint32_t deadline;
	uint32_t task_type;
	uint32_t creation_time;
	struct overdue_tasks *next_cell;
} overdue_tasks;

typedef struct dd_message {
	scheduler_function function_to_call;
	task_param task;
	task_list *head_of_task_list; 
	overdue_tasks *head_of_overdue_list;
	int status_message;
} dd_message;


/* Queues */
xQueueHandle DD_Scheduler_Queue = 0;
xQueueHandle Overdue_Task_To_Add_Queue = 0;
xQueueHandle Head_Of_Overdue_Task_Queue = 0;
xQueueHandle notify_user_task_generator_queue = 0;

/* Tasks */
static void DD_Scheduler();
static void Task_Generator();
static void Monitor_Task(); // idle task
static void Button_Task();

static void User_Tasks(task_param *task);

/* DD scheduler interface functions */
TaskHandle_t dd_tcreate(task_param task);
int dd_delete(TaskHandle_t t_handle);
task_list* dd_return_active_list();
overdue_tasks* dd_return_overdue_list();

/* linked list helper functions */
task_list * add_to_sched_list(task_list *head_of_list, task_param task);
overdue_tasks* add_to_overdue(task_list *head_of_active_list, overdue_tasks *head_of_overdue_task_list, TaskHandle_t task_handle);
static void remove_from_list(task_list *head_of_list, task_param *task);
task_list * reassign_task_priorities(task_list *head_of_list);
static void swap(task_list **head_ref, uint32_t x, uint32_t y);
TaskHandle_t is_in_list(task_list *highest_priority_task, int task_type);

/* periodic task generator timer callbacks */ 
static void vPeriodicTaskTimerCallback1( xTimerHandle xTimer );
static void vPeriodicTaskTimerCallback2( xTimerHandle xTimer );
static void vPeriodicTaskTimerCallback3( xTimerHandle xTimer );

/* binary semaphore for deferring task from ISR */
SemaphoreHandle_t xButtonSemaphore = NULL;
/* mutex for task generators all using the same queue */ 
SemaphoreHandle_t createTaskSemaphore;

/* variables to track total and idle task utilization */
TickType_t total_time_count = 0;
TickType_t  monitor_time_count = 0;
/*-----------------------------------------------------------*/

static void set_up_input_button() {

	/* Set variables used */
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	/* Enable clock for GPIOD */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* Enable clock for SYSCFG */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Set pin as input */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* Tell system that you will use PD0 for EXTI_Line0 */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

	/* PD0 is connected to EXTI_Line0 */
	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	/* Enable interrupt */
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	/* Interrupt mode */
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	/* Triggers on rising and falling edge */
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	/* Add to EXTI */
	EXTI_Init(&EXTI_InitStruct);

	/* Add IRQ vector to NVIC */
	/* PD0 is connected to EXTI_Line0, which has EXTI0_IRQn vector */
	NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
	/* Set priority */
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0xff;
	/* Set sub priority */
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	/* Enable interrupt */
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;

	/* Add to NVIC */
	NVIC_Init(&NVIC_InitStruct);
	NVIC_SetPriority(EXTI0_IRQn, 6);
}

void EXTI0_IRQHandler(void) {
	BaseType_t xHigherPriorityTaskWoken;

	if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
		/* Button has been pressed because rising edge */
	    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) != 1) {
	    	xHigherPriorityTaskWoken = pdFALSE;
	    	// defer further processing to button task by setting binary semaphore
			xSemaphoreGiveFromISR(xButtonSemaphore, &xHigherPriorityTaskWoken);
	    }
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line0);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

int main(void)
{
	set_up_input_button();

	/* Initialize LEDs */
	STM_EVAL_LEDInit(amber_led);
	STM_EVAL_LEDInit(green_led);
	STM_EVAL_LEDInit(red_led);
	STM_EVAL_LEDInit(blue_led);

	prvSetupHardware();

	DD_Scheduler_Queue = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));
	Overdue_Task_To_Add_Queue = xQueueCreate(mainQUEUE_LENGTH, sizeof(TaskHandle_t));
	Head_Of_Overdue_Task_Queue = xQueueCreate(mainQUEUE_LENGTH, sizeof(overdue_tasks));
	notify_user_task_generator_queue = xQueueCreate(mainQUEUE_LENGTH, sizeof( uint32_t ));

	vQueueAddToRegistry( DD_Scheduler_Queue, "DD_Scheduler_Queue" );
	vQueueAddToRegistry( Overdue_Task_To_Add_Queue, "Overdue_Task_To_Add_Queue" );
	vQueueAddToRegistry( notify_user_task_generator_queue, "notify_user_task_generator_queue" );

	xTaskCreate(DD_Scheduler, "DD_Scheduler", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-3, NULL);
	xTaskCreate(Task_Generator, "Task_Generator", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(Monitor_Task, "Monitor_Task", configMINIMAL_STACK_SIZE, NULL, 0, NULL);
	xTaskCreate(Button_Task, "Monitor_Button", configMINIMAL_STACK_SIZE, NULL,  1, NULL);

	xTimerHandle vPeriodicTaskTimer1 = NULL;
	xTimerHandle vPeriodicTaskTimer2 = NULL;
	xTimerHandle vPeriodicTaskTimer3 = NULL;

    createTaskSemaphore = xSemaphoreCreateMutex();

	vPeriodicTaskTimer1 = xTimerCreate("Task1Timer", task1_period, pdTRUE,	( void * ) 0, vPeriodicTaskTimerCallback1);
	vPeriodicTaskTimer2 = xTimerCreate("Task2Timer", task2_period, pdTRUE, ( void * ) 0, vPeriodicTaskTimerCallback2);
	vPeriodicTaskTimer3 = xTimerCreate("Task3Timer", task3_period, pdTRUE, ( void * ) 0, vPeriodicTaskTimerCallback3);

	/* Start the tasks and timer running. */
	xTimerStart( vPeriodicTaskTimer1, 0 );
	xTimerStart( vPeriodicTaskTimer2, 0 );
	xTimerStart( vPeriodicTaskTimer3, 0 );
	vTaskStartScheduler();
}

/* ---------------- DD Scheduler interface functions ------------------*/
TaskHandle_t dd_tcreate(task_param task) {
	// open a queue to communicate with dd scheduler 
	xQueueHandle Task_Creator_Queue;
	Task_Creator_Queue = xQueueCreate(minQUEUE_LENGTH, sizeof(dd_message));

	if(Task_Creator_Queue == NULL){
		printf("fail to create queue\n");
	}
	
	vQueueAddToRegistry(Task_Creator_Queue, "Task_Creator_Queue");
	TaskHandle_t task_handle;

	// create FreeRTOS task
	xTaskCreate(User_Tasks, "dummy_task", configMINIMAL_STACK_SIZE, (&task), 1, &task_handle);

	task.t_handle = task_handle;

	// write all necessary info into message that will be read by dd scheduler
	dd_message dd_scheduler_message;
	dd_scheduler_message.function_to_call = create;
	dd_scheduler_message.task = task;
	// set the status message to -1 to indicate there hasn't been a response from dd sched
	dd_scheduler_message.status_message = -1;

	// send message into Task_Creator_Queue, and write that queue to dd sceduler global queue
	xQueueSend(Task_Creator_Queue, &dd_scheduler_message, 1000);
	xQueueSend(DD_Scheduler_Queue, &Task_Creator_Queue, 1000);

	// wait for confirmation from DD_scheduler
	vTaskDelay(500);
	if (xQueueReceive(Task_Creator_Queue, &dd_scheduler_message, 1000)) {
		// make sure that we didn't just read what we wrote to the Task_Creator_Queue above
		if(dd_scheduler_message.status_message == -1){
			xQueueSend(Task_Creator_Queue, &dd_scheduler_message, 1000);

		}else if (dd_scheduler_message.status_message == STATUS_OKAY && dd_scheduler_message.task.t_handle == task_handle) {
			vQueueDelete(Task_Creator_Queue);
			return task_handle;
		}
	}

	// if don't receive confirmation message, clean up and return with failure
	vTaskDelete(task_handle);
	vQueueDelete(Task_Creator_Queue);

	return STATUS_FAILURE;

}

int dd_delete(TaskHandle_t t_handle) {
	// open a queue to communicate with dd scheduler 
	xQueueHandle Task_Delete_Queue;
	Task_Delete_Queue = xQueueCreate(minQUEUE_LENGTH, sizeof(dd_message));

	if(Task_Delete_Queue == NULL){
		printf("fail to create queue\n");
	}

	vQueueAddToRegistry( Task_Delete_Queue, "Task_Delete_Queue" );
	task_param task;
	task.t_handle = t_handle;
	
	// write all necessary info into message that will be read by dd scheduler
	dd_message dd_scheduler_message;
	dd_scheduler_message.function_to_call = delete;
	dd_scheduler_message.task = task;
	// set the status message to -1 to indicate there hasn't been a response from dd sched
	dd_scheduler_message.status_message = -1;

	// error handling which aided debugging
	if(pdTRUE != xQueueSend(Task_Delete_Queue, &dd_scheduler_message, portMAX_DELAY)){
		printf("failed here %s\n", t_handle);
	};
	if(pdTRUE != xQueueSend(DD_Scheduler_Queue, &Task_Delete_Queue, portMAX_DELAY)){
		printf("failed sending %s\n", t_handle);
	};

	// wait for confirmation from DD_scheduler
	vTaskDelay(500);
	if (xQueueReceive(Task_Delete_Queue, &dd_scheduler_message, portMAX_DELAY)) {
		// make sure that we didn't just read what we wrote to the Task_Creator_Queue above
		if(dd_scheduler_message.status_message == -1){
			xQueueSend(Task_Delete_Queue, &dd_scheduler_message, portMAX_DELAY);

		}else if (dd_scheduler_message.status_message == STATUS_OKAY ) {
			vQueueDelete(Task_Delete_Queue);
			vTaskDelete(NULL);

			return 1;
		}
	}
	// cleanup on failure
	vQueueDelete(Task_Delete_Queue);
	return dd_scheduler_message.status_message;
}

task_list* dd_return_active_list() {
	// open a queue to communicate with dd scheduler 
	xQueueHandle Task_Return_Active_List;
	Task_Return_Active_List = xQueueCreate(mainQUEUE_LENGTH, sizeof(task_list));

	// write all necessary info into message that will be read by dd scheduler
	dd_message dd_scheduler_message;
	dd_scheduler_message.function_to_call = return_active_list;
	dd_scheduler_message.head_of_task_list = NULL;

	xQueueSend(Task_Return_Active_List, &dd_scheduler_message, 1000);
	xQueueSend(DD_Scheduler_Queue, &Task_Return_Active_List, 1000);

	vTaskDelay(500);
	dd_message dd_scheduler_return_message;
	dd_scheduler_return_message.head_of_task_list = NULL;
	
	// return null if we don't get a response from dd scheduler
	if(!xQueueReceive(Task_Return_Active_List, &dd_scheduler_return_message, portMAX_DELAY)) {
		vQueueDelete(Task_Return_Active_List);
		return NULL;
	}

	vQueueDelete(Task_Return_Active_List);
	return dd_scheduler_return_message.head_of_task_list;
}

overdue_tasks* dd_return_overdue_list() {
	// open a queue to communicate with dd scheduler 
	xQueueHandle Task_Return_Overdue_List;
	Task_Return_Overdue_List = xQueueCreate(mainQUEUE_LENGTH, sizeof(task_list));

	// write all necessary info into message that will be read by dd scheduler
	dd_message dd_scheduler_message;
	dd_scheduler_message.function_to_call = return_overdue_list;
	dd_scheduler_message.head_of_overdue_list = NULL;

	xQueueSend(Task_Return_Overdue_List, &dd_scheduler_message, 1000);
	xQueueSend(DD_Scheduler_Queue, &Task_Return_Overdue_List, 1000);
	
	dd_message dd_scheduler_return_message;
	dd_scheduler_return_message.head_of_overdue_list = NULL;
	
	// return null if we don't get a response from dd scheduler
	vTaskDelay(500);
	if(!xQueueReceive(Task_Return_Overdue_List, &dd_scheduler_return_message, portMAX_DELAY)) {
		vQueueDelete(Task_Return_Overdue_List);
		return NULL;
	}

	 vQueueDelete(Task_Return_Overdue_List);

	return dd_scheduler_return_message.head_of_overdue_list;
}


static void remove_from_list(task_list *head_of_list, task_param *task) {
	task_list *pointer1, *prev_pointer;

	pointer1 = head_of_list;
	prev_pointer = NULL;

	while (pointer1 != NULL) {
		// traverse through list until we find task to delete
		if (pointer1->t_handle == task->t_handle) {
			if (prev_pointer != NULL) {
				prev_pointer->next_cell = pointer1->next_cell;
			}
			else {
				head_of_list = pointer1->next_cell;
			}
			free(pointer1);
			return;
		}
		prev_pointer = pointer1;
		pointer1 = pointer1->next_cell;
	}

}

task_list * add_to_sched_list(task_list *head_of_list, task_param task) {
	// convert the task into task list node
	task_list *task_to_add = (task_list*) malloc(sizeof(task_list));

	task_to_add->deadline = task.deadline;
	task_to_add->task_type = task.task_type;
	task_to_add->creation_time = task.release_time;
	task_to_add->next_cell = NULL;
	task_to_add->t_handle = task.t_handle;
	task_to_add->led = task.led;

	if (head_of_list == NULL) {
		head_of_list = task_to_add;
	}

	else {
		task_to_add->next_cell = head_of_list;
		head_of_list = task_to_add;
	}

	return head_of_list;
}

task_list * reassign_task_priorities(task_list *head_of_list) {
	int swapped, count;
	task_list *pointer1;

	if (head_of_list == NULL)
		return head_of_list;

	count = 0;
	do {
		swapped = 0;
		pointer1 = head_of_list;

		while (pointer1->next_cell != NULL) {
			if (pointer1->deadline > pointer1->next_cell->deadline) {
				swap(&head_of_list, pointer1->deadline, pointer1->next_cell->deadline);
				swapped = 1;
			}

			pointer1 = pointer1->next_cell;
			count++;
		}
	}
	while(swapped);

	pointer1 = head_of_list;

	// reset the priorities
	while(pointer1 != NULL) {
		vTaskPrioritySet(pointer1->t_handle, count+1);
		pointer1 = pointer1->next_cell;
		count--;
	}

	return head_of_list;
}

static void swap(task_list **head_ref, uint32_t x, uint32_t y) {
	 // Nothing to do if x and y are same
	if (x == y) return;

	// Search for x (keep track of prevX and CurrX
	struct task_list *prevX = NULL, *currX = *head_ref;
	while (currX && currX->deadline != x)
	{
	   prevX = currX;
	   currX = currX->next_cell;
	}

	// Search for y (keep track of prevY and CurrY
	task_list *prevY = NULL, *currY = *head_ref;
	while (currY && currY->deadline != y)
	{
	   prevY = currY;
	   currY = currY->next_cell;
	}

	// If either x or y is not present, nothing to do
	if (currX == NULL || currY == NULL)
	   return;

	// If x is not head of linked list
	if (prevX != NULL)
	   prevX->next_cell = currY;
	else // Else make y as new head
	   *head_ref = currY;

	// If y is not head of linked list
	if (prevY != NULL)
	   prevY->next_cell = currX;
	else  // Else make x as new head
	   *head_ref = currX;

	// Swap next pointers
	task_list *temp = currY->next_cell;
	currY->next_cell = currX->next_cell;
	currX->next_cell  = temp;
}

TaskHandle_t is_in_list(task_list *highest_priority_task, int task_type) {
	// if the task has been deleted from the list, the deadline will be set to -1
	task_list * pointer;
	pointer = highest_priority_task;

	while (pointer != NULL) {
		if(pointer->led == task_type) {
			return pointer->t_handle;
		}
		pointer = pointer->next_cell;
	}
	return NULL;
}

overdue_tasks* add_to_overdue(task_list *head_of_active_list, overdue_tasks *head_of_overdue_task_list, TaskHandle_t task_handle) {
	task_list *pointer1;

	overdue_tasks *task_to_add = (overdue_tasks*) malloc(sizeof(overdue_tasks));

	pointer1 = head_of_active_list;

	while (pointer1 != NULL) {
		// traverse through active list until find matching overdue task
		if (pointer1->t_handle == task_handle) {
			// convert task list node to overdue task node
			task_to_add->deadline = pointer1->deadline;
			task_to_add->task_type = pointer1->task_type;
			task_to_add->creation_time = pointer1->creation_time;
			task_to_add->next_cell = NULL;
			task_to_add->tid = pointer1->t_handle;

			if (head_of_overdue_task_list == NULL) {
				head_of_overdue_task_list = task_to_add;
			}

			else {
				task_to_add->next_cell = head_of_overdue_task_list;
				head_of_overdue_task_list = task_to_add;
			}
		}
		pointer1 = pointer1->next_cell;
	}
	return head_of_overdue_task_list;
}

/* -------------------------FreeRTOS tasks ------------------------------*/
static void DD_Scheduler() {
	xQueueHandle queue_handler;

	dd_message message;

	task_list *head_of_active_list = NULL;
	overdue_tasks *head_of_overdue_list = NULL;

	task_param task;

	TaskHandle_t *overdue_task_to_add;
	while(1) {
		// will always be blocked unless one of the interface functions put somthing in dd sched queue
		if (xQueueReceive(DD_Scheduler_Queue, &queue_handler, portMAX_DELAY)) {
			// read message in queue inside global queue
			if (xQueueReceive(queue_handler, &message, portMAX_DELAY)) {
				// check what function was called
				switch(message.function_to_call) {
					case create:
						task = message.task;

						/* check if its in active list and add it to overdue list, since all tasks are periodic, 
						   should expect task of same type to not be in active list by next period */
						overdue_task_to_add = is_in_list(head_of_active_list, task.led);

						if (overdue_task_to_add != NULL){
							head_of_overdue_list = add_to_overdue(head_of_active_list, head_of_overdue_list, overdue_task_to_add);
							remove_from_list(head_of_active_list, &task);
						}

						head_of_active_list = add_to_sched_list(head_of_active_list, task);
						head_of_active_list = reassign_task_priorities(head_of_active_list);
						
						message.status_message = STATUS_OKAY;

						xQueueSend(queue_handler, &message, 1000);

						break;

					case delete:
						task = message.task;

						if(task.t_handle != NULL){
							remove_from_list(head_of_active_list, &task);
							message.status_message = STATUS_OKAY;
						}

						xQueueSend(queue_handler, &message, 1000);
						break;

					case return_overdue_list:
						if(head_of_overdue_list != NULL){
							message.head_of_overdue_list = head_of_overdue_list;
						}else{
							message.head_of_overdue_list = NULL;
						}
						xQueueSend(queue_handler, &message, 1000);
						break;

					case return_active_list:
						if(head_of_active_list != NULL){
							message.head_of_task_list = head_of_active_list;
						}else{
							message.head_of_task_list = NULL;
						}
						xQueueSend(queue_handler, &message, 1000);
						break;

					default:
						// send failure message as the message.function is not one of the options
						// shouldn't ever get here
						message.status_message = STATUS_FAILURE;
						xQueueSend(queue_handler, &message, 1000);
				}

			}
		}

	}

}

static void Task_Generator() {
	int task_to_run = 1;
	uint32_t period;
	uint32_t exec_time;
	int led;

	while(1) {
		// queue is filled by one of the task generator timers, which run periodically
		if(xQueueReceive(notify_user_task_generator_queue , &task_to_run, 1000)){
			// set the correct paramters for specifid task
			switch(task_to_run) {
				case 1:
					period = task1_period;
					exec_time = task1_exec;
					led = task1_led;

					break;
				case 2:
					period = task2_period;
					exec_time = task2_exec;
					led = task2_led;

					break;
				case 3:
					period = task3_period;
					exec_time = task3_exec;
					led = task3_led;

					break;
				}

			task_param task_to_create;

			task_to_create.exec_time = exec_time;
			task_to_create.release_time = 0;
			task_to_create.deadline = period;
			task_to_create.led = led;

			dd_tcreate(task_to_create);
		}
		vTaskDelay(1500);
	}
}


static void Monitor_Task(){
	TickType_t t;

	/* Monitor task simply acts as the idle task and keeps a record of running time */
	while(1) {
		t = xTaskGetTickCount();
		t = xTaskGetTickCount() - t;
		monitor_time_count +=  t;
	}

}

static void Button_Task() {
	xButtonSemaphore = xSemaphoreCreateBinary();

	task_list *head_of_task_list;
	task_list *current_task;
	
	overdue_tasks *head_of_overdue_tasks;
	overdue_tasks *current_overdue_task;
	
	TickType_t current;
	
	double result = 0.0;
	while(1) {
		// will only be triggered when button has been pressed and ISR sets semaphore
		if(xSemaphoreTake(xButtonSemaphore, 0xff)  == pdTRUE) {
			head_of_task_list = dd_return_active_list();
			current_task = head_of_task_list;
			
			if(current_task == NULL) {
				printf("Active list: EMPTY \n");
			}
			// if there is anything in active list, traverse through and print info
			while (current_task != NULL) {
				printf("Active list: \n");
				printf("\ttask: %d \n", current_task->led);
				printf("\tdeadline: %u\n", (unsigned int) current_task->deadline);
				printf("\ttask handle: %d\n", (unsigned int) current_task->t_handle);
				current_task = current_task->next_cell;
			}

			head_of_overdue_tasks = dd_return_overdue_list();
			current_overdue_task = head_of_overdue_tasks;
			
			if(current_overdue_task == NULL) {
				printf("Overdue list: EMPTY \n");
			}

			// if there is anything in overdue list, traverse through and print info
			while (current_overdue_task != NULL) {
				printf("Overdue list: \n");
				printf("\tdeadline: %u\n", (unsigned int) current_overdue_task->deadline);
				printf("\ttask handle: %u\n", (unsigned int) current_overdue_task->tid);
				current_overdue_task = current_overdue_task->next_cell;
			}

			current = xTaskGetTickCount();
			/* determine the processor utilization by dividing total time that user tasks have been running by total time 
			   since the scheduler has been running (since program was first started)
			*/
			result = ((double)total_time_count/(double)current)*100;
			printf("processor utilization is: %d percent\n", (int)result);

			/* determine the processor overhead by subtracting percentage of time monitor task has been running from percentage
			   user tasks have been running - i.e. this will be all other tasks besided monitor task and user tasks
			*/
			result = (1.0 -((double)total_time_count/(double)current) - ((double)monitor_time_count/(double)current))*100;
			printf("processor overhead is: %d percent\n", (int)result);
		}

		vTaskDelay(500);
	}

}


static void User_Tasks(task_param *task) {
	TickType_t t, end;
	int j = 0;

	// turn off led so it can be updated
	STM_EVAL_LEDOff(red);
	STM_EVAL_LEDOff(amber);
	STM_EVAL_LEDOff(green);
	STM_EVAL_LEDOff(blue);
	// allow the led for specified task to stay on until next task is run
	STM_EVAL_LEDOn(task->led);

	// run task for specified time, keep track of total time in this function
	t = xTaskGetTickCount();
	while(xTaskGetTickCount()-t < (task->exec_time)) {}
	end = xTaskGetTickCount();
	total_time_count += end - t;

	TaskHandle_t current = xTaskGetCurrentTaskHandle();
	// delete the current task because it will be regenerated next period
	dd_delete(current);
}


/* Periodic tasks that generate user tasks - callbacks are handled in main */

static void vPeriodicTaskTimerCallback1( xTimerHandle xTimer )
{
	int task = 1;
	if( xSemaphoreTake( createTaskSemaphore, ( TickType_t ) 10 ) == pdTRUE ) {
		xQueueSend(notify_user_task_generator_queue,&task,100);
		xSemaphoreGive( createTaskSemaphore );
	}

}
static void vPeriodicTaskTimerCallback2( xTimerHandle xTimer )
{
	int task = 2;

	if( xSemaphoreTake( createTaskSemaphore, ( TickType_t ) 10 ) == pdTRUE ) {
		xQueueSend(notify_user_task_generator_queue,&task,100);
		xSemaphoreGive( createTaskSemaphore );
	}
}

static void vPeriodicTaskTimerCallback3( xTimerHandle xTimer )
{
	int task = 3;

	if( xSemaphoreTake( createTaskSemaphore, ( TickType_t ) 10 ) == pdTRUE ) {
		xQueueSend(notify_user_task_generator_queue,&task,100);
		xSemaphoreGive( createTaskSemaphore );
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
//	xFreeStackSpace = xPortGetFreeHeapSize();

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
