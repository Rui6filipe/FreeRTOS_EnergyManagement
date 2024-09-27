/*
 * FreeRTOS V202212.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */


/******************************************************************************
 * See https://www.freertos.org/freertos-on-qemu-mps2-an385-model.html for
 * instructions.
 *
 * This project provides two demo applications.  A simple blinky style project,
 * and a more comprehensive test and demo application.  The
 * mainCREATE_SIMPLE_BLINKY_DEMO_ONLY constant, defined in this file, is used to
 * select between the two.  The simply blinky demo is implemented and described
 * in main_blinky.c.  The more comprehensive test and demo application is
 * implemented and described in main_full.c.
 *
 * This file implements the code that is not demo specific, including the
 * hardware setup and FreeRTOS hook functions.
 *
 * Running in QEMU:
 * Use the following commands to start the application running in a way that
 * enables the debugger to connect, omit the "-s -S" to run the project without
 * the debugger:
 * qemu-system-arm -machine mps2-an385 -cpu cortex-m3 -kernel [path-to]/RTOSDemo.out -monitor none -nographic -serial stdio -s -S
 */

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"

/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

/* Demo app includes. */
#include "death.h"
#include "blocktim.h"
#include "semtest.h"
#include "PollQ.h"
#include "GenQTest.h"
#include "QPeek.h"
#include "recmutex.h"
#include "IntQueue.h"
#include "QueueSet.h"
#include "EventGroupsDemo.h"
#include "MessageBufferDemo.h"
#include "StreamBufferDemo.h"
#include "AbortDelay.h"
#include "countsem.h"
#include "dynamic.h"
#include "MessageBufferAMP.h"
#include "QueueOverwrite.h"
#include "QueueSetPolling.h"
#include "StaticAllocation.h"
#include "TaskNotify.h"
#include "TaskNotifyArray.h"
#include "TimerDemo.h"
#include "StreamBufferInterrupt.h"
#include "IntSemTest.h"

/* This project provides two demo applications.  A simple blinky style demo
 * application, and a more comprehensive test and demo application.  The
 * mainCREATE_SIMPLE_BLINKY_DEMO_ONLY setting is used to select between the two.
 *
 * If mainCREATE_SIMPLE_BLINKY_DEMO_ONLY is 1 then the blinky demo will be built.
 * The blinky demo is implemented and described in main_blinky.c.
 *
 * If mainCREATE_SIMPLE_BLINKY_DEMO_ONLY is not 1 then the comprehensive test and
 * demo application will be built.  The comprehensive test and demo application is
 * implemented and described in main_full.c. */
#define mainCREATE_SIMPLE_BLINKY_DEMO_ONLY    1

/* printf() output uses the UART.  These constants define the addresses of the
 * required UART registers. */
#define UART0_ADDRESS                         ( 0x40004000UL )
#define UART0_DATA                            ( *( ( ( volatile uint32_t * ) ( UART0_ADDRESS + 0UL ) ) ) )
#define UART0_STATE                           ( *( ( ( volatile uint32_t * ) ( UART0_ADDRESS + 4UL ) ) ) )
#define UART0_CTRL                            ( *( ( ( volatile uint32_t * ) ( UART0_ADDRESS + 8UL ) ) ) )
#define UART0_BAUDDIV                         ( *( ( ( volatile uint32_t * ) ( UART0_ADDRESS + 16UL ) ) ) )
#define TX_BUFFER_MASK                        ( 1UL )

/* The number of items the queues can hold at once. */
#define mainPOWER_QUEUE_LENGTH                   ( 2 )
#define mainGRID_QUEUE_LENGTH                   ( 4 )

/* Priorities at which the tasks are created. */
#define mainSOLAR_GEN_TASK_PRIORITY    ( tskIDLE_PRIORITY + 1 )
#define mainBATTERY_MGMT_TASK_PRIORITY  ( tskIDLE_PRIORITY + 2 )
#define mainLOAD_MGMT_TASK_PRIORITY  ( tskIDLE_PRIORITY + 3 )
#define mainGRID_INTERACT_TASK_PRIORITY  ( tskIDLE_PRIORITY + 4 )

/* The rate at which data is sent to the queue, and the rate at which battery level is checked. 
The times are converted from milliseconds to ticks using the pdMS_TO_TICKS() macro. */
#define TASK_SOLAR_GEN_FREQUENCY_MS    pdMS_TO_TICKS( 200UL )
#define TASK_LOAD_MAN_FREQUENCY_MS     pdMS_TO_TICKS( 200UL )

/* Macro to calculate the solar power in W, assuming a maximum of 5000 W */
#define AMPLITUDE 5000
#define PERIOD 24
#define PHASE M_PI/2
#define mainSOLAR_POWER(tick) ( (TickType_t) ( AMPLITUDE * sin(2 * M_PI / PERIOD * (tick) / configTICK_RATE_HZ - PHASE) ) )

/* The rate at which the battery is updated in this simulation is 12 minutes in real life time.
 * We have the power, to get energy we just need to multiply by 0.2 (W.h). Since that is floating point,
 * we will multiply by 20 and divide by 100 */
#define TIME_DENOMINATOR 100
#define TIME_NUMERATOR 20

/* Battery max capacity in W.h */
#define CAPACITY 10000

/* Number of devices being considered */
#define NUM_DEVICES ( sizeof(devices) / sizeof(devices[0]) )

/* Macro to calculate price of energy at a given time in the day. It is in miliCents /W */
#define PRICE_AMPLITUDE 0.07    // Represents the maximum price fluctuation (±7 cents)
#define BASE_PRICE 0.22         // Base price in €/kWh (22 cents)
#define PRICE_PERIOD 24         
#define PRICE_PHASE M_PI/2           
#define ENERGY_PRICE(tick) ( (TickType_t) (( BASE_PRICE + PRICE_AMPLITUDE * sin(2 * M_PI / PRICE_PERIOD * (tick) / configTICK_RATE_HZ - PRICE_PHASE) )*100) )

/*
 * Only the comprehensive demo uses application hook (callback) functions.  See
 * https://www.FreeRTOS.org/a00016.html for more information.
 */
void vFullDemoTickHookFunction( void ); // PROBABLY CAN DELETE THESEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
void vFullDemoIdleFunction( void );

/*
 * Printf() output is sent to the serial port.  Initialise the serial hardware.
 */
static void prvUARTInit( void );

/* Tasks */
void vTaskSolarPowerGeneration( void * pvParameters );
void vTaskBatteryManagement( void * pvParameters );
void vTaskLoadManagement( void * pvParameters );
void vTaskGridInteraction( void * pvParameters );

/* Queue for power values. Queue to signal grid interaction. */
static QueueHandle_t xQueuePower = NULL;
static QueueHandle_t xQueueGrid = NULL;

/* The binary semaphore used to update the battery level. */
static SemaphoreHandle_t xSemaphore = NULL;

/* Battery level in W.h */
static int32_t batteryLevel = 0;

/* Expenditure or profit with energy */
static int32_t lBill = 0;

/* Structure of an appliance */
typedef struct {
    char name[20];
    uint16_t power; // Power consuption in W
    bool status; // On or Off
    uint8_t priority; // Lowest number means higher priority
} Appliance;

/* List of devices */
Appliance devices[] = {

    {"Lighting", 100, false, 1}, // 10 LEDs consuming 10W each
    {"Refrigerator", 300, true, 1},
    {"Wahsing Machine", 1000, false, 2}
};
/*-----------------------------------------------------------*/

void main( void )
{
    /* See https://www.freertos.org/freertos-on-qemu-mps2-an385-model.html for
     * instructions. */

    /* Hardware initialisation.  printf() output uses the UART for IO. */
    prvUARTInit();

    /* Create the queue. */
    xQueuePower = xQueueCreate( mainPOWER_QUEUE_LENGTH, sizeof( uint32_t ) );
    xQueueGrid = xQueueCreate( mainGRID_QUEUE_LENGTH, sizeof( uint32_t ) );

    xSemaphore = xSemaphoreCreateMutex();

    if( (xQueuePower != NULL) && (xQueueGrid != NULL) && (xSemaphore != NULL) ){

        xTaskCreate( vTaskSolarPowerGeneration,     /* The function that implements the task. */
                    "SolarGen",                     /* The text name assigned to the task - for debug only as it is not used by the kernel. */
                    1048,                           /* The size of the stack to allocate to the task. */
                    NULL,                           /* The parameter passed to the task - not used in this simple case. */
                    mainSOLAR_GEN_TASK_PRIORITY,    /* The priority assigned to the task. */
                    NULL );                         /* The task handle is not required, so NULL is passed. */

        
        xTaskCreate( vTaskBatteryManagement, "BatteryMgmt", 1048, NULL, mainBATTERY_MGMT_TASK_PRIORITY, NULL );

        xTaskCreate( vTaskLoadManagement, "LoadMgmt", 1048, NULL, mainLOAD_MGMT_TASK_PRIORITY, NULL );

        xTaskCreate( vTaskGridInteraction, "GridInteract", 1048, NULL, mainGRID_INTERACT_TASK_PRIORITY, NULL );

        // Create tasks and start scheduler
        vTaskStartScheduler();
    }

    // The scheduler should never return. If it does, loop indefinitely.
    for( ; ; ){}
}
/*-----------------------------------------------------------*/

void vTaskSolarPowerGeneration( void * pvParameters )
{
    TickType_t xNextWakeTime;
    const TickType_t xBlockTime = TASK_SOLAR_GEN_FREQUENCY_MS;

    /* Prevent the compiler warning about the unused parameter. */
    ( void ) pvParameters;

    /* Initialise xNextWakeTime - this only needs to be done once. */
    xNextWakeTime = xTaskGetTickCount();

    for( ; ; )
    {
        /* Place this task in the blocked state until it is time to run again */      
        vTaskDelayUntil( &xNextWakeTime, xBlockTime );

        /* Calculate the Solar Power delivered to the cell in this time */
        uint16_t usValueToSend = mainSOLAR_POWER(xNextWakeTime); 

        /* Send to the queue - causing the queue receive task to unblock and
         * write to the console.  0 is used as the block time so the send operation
         * will not block - it shouldn't need to block as the queue should always
         * have at least one space at this point in the code. */
        xQueueSend( xQueuePower, &usValueToSend, 0U );
    }   
}
/*-----------------------------------------------------------*/

void vTaskBatteryManagement( void * pvParameters )
{
    uint16_t usReceivedValue;

    /* Prevent the compiler warning about the unused parameter. */
    ( void ) pvParameters;

    for( ; ; )
    {
        /* Wait until something arrives in the queue - this task will block
         * indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
         * FreeRTOSConfig.h. */
        xQueueReceive( xQueuePower, &usReceivedValue, portMAX_DELAY );

        /*  Check if received value is an expected value, and if battery is not full. 
         * Take the mutex and update the battery level */
        uint16_t usEnergy = usReceivedValue * TIME_NUMERATOR / TIME_DENOMINATOR;
        uint32_t localBatteryLevel = 0; 
       
        if( (usReceivedValue <= AMPLITUDE) && ((batteryLevel + usEnergy) < CAPACITY) ) 
        {   
            if( xSemaphoreTake( xSemaphore, ( TickType_t ) pdMS_TO_TICKS(10UL) ) == pdTRUE )
            {
                batteryLevel += usEnergy;
                localBatteryLevel = batteryLevel; 
                xSemaphoreGive( xSemaphore );
            }
            else
            {
                printf( "Could not update battery\r\n" );
            }
        }
        else if ((usReceivedValue <= AMPLITUDE))
        {
            // Signal to the Grid Interaction Task we are selling energy
            xQueueSend( xQueueGrid, &usEnergy, 0U );
            localBatteryLevel = batteryLevel; 
        }
        else
        {
            printf( "Unexpected message\r\n" );
        }

        /* Print battery level outside the critical region, with a local variable equal to the batteryLevel updated by this task */
        printf("Battery Level: %u\n", localBatteryLevel); 
    }
}
/*-----------------------------------------------------------*/

void vTaskLoadManagement( void * pvParameters )
{
    TickType_t xNextWakeTime;
    const TickType_t xBlockTime = TASK_LOAD_MAN_FREQUENCY_MS;

    /* Prevent the compiler warning about the unused parameter. */
    ( void ) pvParameters;

    /* Initialise xNextWakeTime - this only needs to be done once. */
    xNextWakeTime = xTaskGetTickCount();

    for( ; ; )
    {
        /* Place this task in the blocked state until it is time to run again */      
        vTaskDelayUntil( &xNextWakeTime, xBlockTime );

        /* Check the state of each appliance and calculate the total power consumption based on which devices are active.*/
        uint16_t usConsumedPower = 0;
        for (uint8_t i=0; i < NUM_DEVICES; i++){
            if (devices[i].status == 1){
                usConsumedPower += devices[i].power;
            }
        }

        uint16_t usEnergy = usConsumedPower * TIME_NUMERATOR / TIME_DENOMINATOR;
        
        /*  Update battery level if we have enough battery */
        if (batteryLevel > usEnergy)
        {
            if( xSemaphoreTake( xSemaphore, ( TickType_t ) pdMS_TO_TICKS(10UL) ) == pdTRUE )
            {
                batteryLevel -= usEnergy;
                xSemaphoreGive( xSemaphore );
            }
            else
            {
                printf( "Could not update battery\r\n" );
            }
        }
        else
        {
            // Signal to the Grid Interaction Task we are buying energy
            int16_t sValueToSend = -usEnergy; 
            xQueueSend( xQueueGrid, &sValueToSend, 0U );
            
        }
    }
}
/*-----------------------------------------------------------*/

void vTaskGridInteraction( void * pvParameters )
{
    int16_t usReceivedValue;

    /* Prevent the compiler warning about the unused parameter. */
    ( void ) pvParameters;

    for( ; ; )
    {
        /* Wait until something arrives in the queue - this task will block indefinitely */
        xQueueReceive( xQueueGrid, &usReceivedValue, portMAX_DELAY );

        /* Add to the bill. Divide by 100 to convert from cent/1000 to cent/10 */
        lBill += usReceivedValue * ENERGY_PRICE(xTaskGetTickCount());

        printf("Bill: %d\n", lBill/100); 
    }
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
    /* vApplicationMallocFailedHook() will only be called if
     * configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
     * function that will get called if a call to pvPortMalloc() fails.
     * pvPortMalloc() is called internally by the kernel whenever a task, queue,
     * timer or semaphore is created using the dynamic allocation (as opposed to
     * static allocation) option.  It is also called by various parts of the
     * demo application.  If heap_1.c, heap_2.c or heap_4.c is being used, then the
     * size of the	heap available to pvPortMalloc() is defined by
     * configTOTAL_HEAP_SIZE in FreeRTOSConfig.h, and the xPortGetFreeHeapSize()
     * API function can be used to query the size of free heap space that remains
     * (although it does not provide information on how the remaining heap might be
     * fragmented).  See http://www.freertos.org/a00111.html for more
     * information. */
    printf( "\r\n\r\nMalloc failed\r\n" );
    portDISABLE_INTERRUPTS();

    for( ; ; )
    {
    }
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
     * to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
     * task.  It is essential that code added to this hook function never attempts
     * to block in any way (for example, call xQueueReceive() with a block time
     * specified, or call vTaskDelay()).  If application tasks make use of the
     * vTaskDelete() API function to delete themselves then it is also important
     * that vApplicationIdleHook() is permitted to return to its calling function,
     * because it is the responsibility of the idle task to clean up memory
     * allocated by the kernel to any task that has since deleted itself. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask,
                                    char * pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
     * configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
     * function is called if a stack overflow is detected. */
    printf( "\r\n\r\nStack overflow in %s\r\n", pcTaskName );
    portDISABLE_INTERRUPTS();

    for( ; ; )
    {
    }
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
    /* This function will be called by each tick interrupt if
    * configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
    * added here, but the tick hook is called from an interrupt context, so
    * code must not attempt to block, and only the interrupt safe FreeRTOS API
    * functions can be used (those that end in FromISR()). */

    #if ( mainCREATE_SIMPLE_BLINKY_DEMO_ONLY != 1 )
    {
        extern void vFullDemoTickHookFunction( void );

        vFullDemoTickHookFunction();
    }
    #endif /* mainCREATE_SIMPLE_BLINKY_DEMO_ONLY */
}
/*-----------------------------------------------------------*/

void vApplicationDaemonTaskStartupHook( void )
{
    /* This function will be called once only, when the daemon task starts to
     * execute (sometimes called the timer task).  This is useful if the
     * application includes initialisation code that would benefit from executing
     * after the scheduler has been started. */
}
/*-----------------------------------------------------------*/

void vAssertCalled( const char * pcFileName,
                    uint32_t ulLine )
{
    volatile uint32_t ulSetToNonZeroInDebuggerToContinue = 0;

    /* Called if an assertion passed to configASSERT() fails.  See
     * http://www.freertos.org/a00110.html#configASSERT for more information. */

    printf( "ASSERT! Line %d, file %s\r\n", ( int ) ulLine, pcFileName );

    taskENTER_CRITICAL();
    {
        /* You can step out of this function to debug the assertion by using
         * the debugger to set ulSetToNonZeroInDebuggerToContinue to a non-zero
         * value. */
        while( ulSetToNonZeroInDebuggerToContinue == 0 )
        {
            __asm volatile ( "NOP" );
            __asm volatile ( "NOP" );
        }
    }
    taskEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
 * implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
 * used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t ** ppxIdleTaskTCBBuffer,
                                    StackType_t ** ppxIdleTaskStackBuffer,
                                    uint32_t * pulIdleTaskStackSize )
{
/* If the buffers to be provided to the Idle task are declared inside this
 * function then they must be declared static - otherwise they will be allocated on
 * the stack and so not exists after this function exits. */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
     * state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
 * application must provide an implementation of vApplicationGetTimerTaskMemory()
 * to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory( StaticTask_t ** ppxTimerTaskTCBBuffer,
                                     StackType_t ** ppxTimerTaskStackBuffer,
                                     uint32_t * pulTimerTaskStackSize )
{
/* If the buffers to be provided to the Timer task are declared inside this
 * function then they must be declared static - otherwise they will be allocated on
 * the stack and so not exists after this function exits. */
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

    /* Pass out a pointer to the StaticTask_t structure in which the Timer
     * task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
/*-----------------------------------------------------------*/

static void prvUARTInit( void )
{
    UART0_BAUDDIV = 16;
    UART0_CTRL = 1;
}
/*-----------------------------------------------------------*/

int __write( int iFile,
             char * pcString,
             int iStringLength )
{
    int iNextChar;

    /* Avoid compiler warnings about unused parameters. */
    ( void ) iFile;

    /* Output the formatted string to the UART. */
    for( iNextChar = 0; iNextChar < iStringLength; iNextChar++ )
    {
        while( ( UART0_STATE & TX_BUFFER_MASK ) != 0 )
        {
        }

        UART0_DATA = *pcString;
        pcString++;
    }

    return iStringLength;
}
/*-----------------------------------------------------------*/

void * malloc( size_t size )
{
    ( void ) size;

    /* This project uses heap_4 so doesn't set up a heap for use by the C
     * library - but something is calling the C library malloc().  See
     * https://freertos.org/a00111.html for more information. */
    printf( "\r\n\r\nUnexpected call to malloc() - should be usine pvPortMalloc()\r\n" );
    portDISABLE_INTERRUPTS();

    for( ; ; )
    {
    }
}
