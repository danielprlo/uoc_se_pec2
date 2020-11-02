/*
 * Copyright (C) 2017 Universitat Oberta de Catalunya - http://www.uoc.edu/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Universitat Oberta de Catalunya nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*----------------------------------------------------------------------------*/
#include "driverlib.h"
/* Standard includes */
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>


/* Free-RTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "portmacro.h"

/* MSP432 drivers includes */
#include "msp432_launchpad_board.h"
#include "uart_driver.h"
#include "helper.h"

// defines
/*----------------------------------------------------------------------------*/
#define TASK_PRIORITY               ( tskIDLE_PRIORITY + 2 )
#define HEARTBEAT_TASK_PRIORITY     ( tskIDLE_PRIORITY + 1 )
#define READ_TASK_PRIORITY          ( tskIDLE_PRIORITY + 2 )

#define TASK_STACK_SIZE             ( 1024 )
#define HEARTBEAT_STACK_SIZE        ( 128 )

#define QUEUE_SIZE                  ( 50 )

#define HEART_BEAT_ON_TIME          ( 10 )
#define HEART_BEAT_OFF_TIME         ( 990 )
/*----------------------------------------------------------------------------*/



// Tasks
static void HeartBeatTask(void *pvParameters);
static void ReadingTask(void *pvParameters);
static void PrintingTask(void *pvParameters);

// callbacks & functions
void uart_rx_callback(void);

//Task sync tools and variables
SemaphoreHandle_t xComputationComplete;
QueueHandle_t xQueueUART;

/*----------------------------------------------------------------------------*/

static void HeartBeatTask(void *pvParameters){

    for(;;){
        led_toggle(MSP432_LAUNCHPAD_LED_RED);
        vTaskDelay( pdMS_TO_TICKS(HEART_BEAT_ON_TIME) );
        led_toggle(MSP432_LAUNCHPAD_LED_RED);
        vTaskDelay( pdMS_TO_TICKS(HEART_BEAT_OFF_TIME) );
    }
}

static void ReadingTask(void *pvParameters) {
    const TickType_t xPeriod = pdMS_TO_TICKS(100);
    char newCaracter;

    for(;;){
        if (xQueueReceive(xQueueUART, &newCaracter, portMAX_DELAY) == pdPASS) {
            uart_print("Caracter received");
        }
    }
}

static void PrintingTask(void *pvParameters) {
    const TickType_t xPeriod = pdMS_TO_TICKS(100);
    for(;;){
        if (xSemaphoreTake(xComputationComplete, portMAX_DELAY) == pdPASS) {
            uart_print("A leer");
        }
    }
}


void uart_rx_callback(void){
    char data;
    uart_get_char(&data);
    if (data == 'p') {
        xQueueSendFromISR(xQueueUART, data, pdFALSE);
        uart_print("p");
    } else {
        uart_print(data);
    }
}


/*----------------------------------------------------------------------------*/

int main(int argc, char** argv)
{
    int32_t retVal = -1;

    // Initialize semaphores and queue
    xComputationComplete = xSemaphoreCreateBinary ();
    xQueueUART = xQueueCreate( QUEUE_SIZE, sizeof( char ) );

    uart_init(uart_rx_callback);

    /* Initialize the board */
    board_init();

    if ( (xComputationComplete != NULL) && (xQueueUART != NULL)) {

        /* Create HeartBeat task */
        retVal = xTaskCreate(HeartBeatTask, "HeartBeatTask", HEARTBEAT_STACK_SIZE, NULL, HEARTBEAT_TASK_PRIORITY, NULL );
        if(retVal < 0) {
            led_on(MSP432_LAUNCHPAD_LED_RED);
            while(1);
        }

        /* Create Reading task */
        retVal = xTaskCreate(ReadingTask, "ReadingTask", TASK_STACK_SIZE, NULL, READ_TASK_PRIORITY, NULL );
        if(retVal < 0) {
            led_on(MSP432_LAUNCHPAD_LED_RED);
            while(1);
        }

        /* Create Printing task */
        retVal = xTaskCreate(PrintingTask, "PrintingTask", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL );
        if(retVal < 0) {
            led_on(MSP432_LAUNCHPAD_LED_RED);
            while(1);
        }


        /* Start the task scheduler */
        vTaskStartScheduler();
    }

    return 0;
}

/*----------------------------------------------------------------------------*/
