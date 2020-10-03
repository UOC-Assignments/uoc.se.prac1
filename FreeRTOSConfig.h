//*****************************************************************************
//
// Copyright (C) 2015 - 2016 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  Redistributions of source code must retain the above copyright
//  notice, this list of conditions and the following disclaimer.
//
//  Redistributions in binary form must reproduce the above copyright
//  notice, this list of conditions and the following disclaimer in the
//  documentation and/or other materials provided with the
//  distribution.
//
//  Neither the name of Texas Instruments Incorporated nor the names of
//  its contributors may be used to endorse or promote products derived
//  from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* Constants related to the behaviour or the scheduler. */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION	1
#define configUSE_PREEMPTION					1
#define configUSE_TIME_SLICING					1
#define configMAX_PRIORITIES					( 7 )
#define configIDLE_SHOULD_YIELD					1
#define configUSE_16_BIT_TICKS					0

/* Constants that describe the hardware and memory usage. */
#define configCPU_CLOCK_HZ						MAP_CS_getMCLK()
#define configMINIMAL_STACK_SIZE				( ( uint16_t ) 100 )
#define configMAX_TASK_NAME_LEN					( 12 )
#define configTOTAL_HEAP_SIZE					( ( size_t ) ( 48 * 1024 ) )

/* Constants that build features in or out. */
#define configUSE_MUTEXES						1
#define configUSE_TICKLESS_IDLE					0
#define configUSE_APPLICATION_TASK_TAG			0
#define configUSE_NEWLIB_REENTRANT 				0
#define configUSE_CO_ROUTINES 					0
#define configUSE_COUNTING_SEMAPHORES 			1
#define configUSE_RECURSIVE_MUTEXES				1
#define configUSE_QUEUE_SETS					0
#define configUSE_TASK_NOTIFICATIONS			1

#define configUSE_IDLE_HOOK						0
#define configUSE_TICK_HOOK						0
#define configUSE_MALLOC_FAILED_HOOK			0

#define configCHECK_FOR_STACK_OVERFLOW			0
#define configASSERT( x ) if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }
#define configQUEUE_REGISTRY_SIZE				0

#define configUSE_TIMERS						0
#define configTIMER_TASK_PRIORITY				( 3 )
#define configTIMER_QUEUE_LENGTH				5
#define configTIMER_TASK_STACK_DEPTH			( configMINIMAL_STACK_SIZE  )

#define INCLUDE_vTaskPrioritySet				1
#define INCLUDE_uxTaskPriorityGet				1
#define INCLUDE_vTaskDelete						1
#define INCLUDE_vTaskCleanUpResources			0
#define INCLUDE_vTaskSuspend					1
#define INCLUDE_vTaskDelayUntil					1
#define INCLUDE_vTaskDelay						1
#define INCLUDE_uxTaskGetStackHighWaterMark		0
#define INCLUDE_xTaskGetIdleTaskHandle			0
#define INCLUDE_eTaskGetState					1
#define INCLUDE_xTaskResumeFromISR				0
#define INCLUDE_xTaskGetCurrentTaskHandle		1
#define INCLUDE_xTaskGetSchedulerState			0
#define INCLUDE_xSemaphoreGetMutexHolder		0
#define INCLUDE_xTimerPendFunctionCall			0

#define configUSE_STATS_FORMATTING_FUNCTIONS	0

#define configCOMMAND_INT_MAX_OUTPUT_SIZE		2048

/* Use the system definition, if there is one. */
#ifdef __NVIC_PRIO_BITS
	#define configPRIO_BITS       __NVIC_PRIO_BITS
#else
	#define configPRIO_BITS       3     /* 8 priority levels */
#endif

/* The lowest interrupt priority that can be used in a call to a "set priority"
function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY			0x07

/* The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
PRIORITY THAN THIS! (higher priorities are lower numeric values. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY	5

#define configKERNEL_INTERRUPT_PRIORITY 		( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names - can't be used with CCS due to limitations in the assemblers
pre-processing. */
#ifndef __TI_COMPILER_VERSION__
	#define xPortPendSVHandler 	PendSV_Handler
	#define vPortSVCHandler 	SVC_Handler
	#define xPortSysTickHandler	SysTick_Handler
#endif

/* The trace facility is turned on to make some functions available for use in
CLI commands. */
#define configUSE_TRACE_FACILITY	1

/* TI driver library includes. */
#include <driverlib.h>

void vPreSleepProcessing( uint32_t ulExpectedIdleTime );
#define configPRE_SLEEP_PROCESSING( x ) vPreSleepProcessing( x )

/* Constants related to the generation of run time stats.  Run time stats
are gathered in the full demo, not the blinky demo. */
#define configGENERATE_RUN_TIME_STATS			0
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()
#define portGET_RUN_TIME_COUNTER_VALUE()		0

/* The blinky demo can use a slow tick rate to save power. */
#define configTICK_RATE_HZ						( ( TickType_t ) 100 )

#endif /* FREERTOS_CONFIG_H */
