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
#define READ_TASK_PRIORITY          ( tskIDLE_PRIORITY + 5 )

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
static bool verifyOperation(void);
static int executeOperation(void);
static void printInt(int data);
static char getCharacter(char);
static void printChar(char data);
//Task sync tools and variables
SemaphoreHandle_t xComputationComplete;
QueueHandle_t xQueueUART;
char buffer[50];
int bufferPointer;

/*----------------------------------------------------------------------------*/

static void HeartBeatTask(void *pvParameters){

    for(;;){
        led_toggle(MSP432_LAUNCHPAD_LED_RED);
        vTaskDelay( pdMS_TO_TICKS(HEART_BEAT_ON_TIME) );
        led_toggle(MSP432_LAUNCHPAD_LED_RED);
        vTaskDelay( pdMS_TO_TICKS(HEART_BEAT_OFF_TIME) );
    }
}

void uart_rx_callback(void){
    char data;
    uart_get_char(&data);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(xQueueUART, &data, xHigherPriorityTaskWoken);
    printChar(data);
}

static void ReadingTask(void *pvParameters) {
    char newCharacter;
    for(;;){
        if (xQueueReceive(xQueueUART, &newCharacter, portMAX_DELAY) == pdPASS) {
            if (newCharacter == 13) {
                xSemaphoreGive(xComputationComplete);
            } else {
                buffer[bufferPointer] = newCharacter;
                bufferPointer = bufferPointer+1;
                if (bufferPointer == 50) {
                    bufferPointer = 0;
                }
            }
        }
    }
}

static void PrintingTask(void *pvParameters) {
    char message[10];
    for(;;){
        if (xSemaphoreTake(xComputationComplete, portMAX_DELAY) == pdPASS) {
            if(verifyOperation()) {
                int result = executeOperation();
                printInt(result);
            }
        }
    }
}

static bool verifyOperation(void) {
    if (bufferPointer < 2) {
        uart_print("No hay 3 caracteres");
        return false;
    }

    char buffedChar = buffer[bufferPointer-2];

    if (buffedChar != '+' &&
        buffedChar != '/' &&
        buffedChar != '-' &&
        buffedChar != '*') {

        uart_print("Codigo de operacion no valido");
        return false;
    }

    char buffedOp1 = buffer[bufferPointer-1];
    char buffedOp2 = buffer[bufferPointer-3];

    if (buffedOp1 < '0' ||
        buffedOp1 > '9' ||
        buffedOp2 < '0' ||
        buffedOp2 > '9') {

        uart_print("Alguno de los operandos o ambos no es un numero entre 0 y 9");
        return false;
    }
}

static int executeOperation(void) {
    char buffedOperator = buffer[bufferPointer-2];
    char buffedOp1      = buffer[bufferPointer-1];
    char buffedOp2      = buffer[bufferPointer-3];

    int intOp1 = (int)buffedOp1-48;
    int intOp2 = (int)buffedOp2-48;

    int result;

    if (buffedOperator == '+') {
        result = intOp2 + intOp1;
    }

    if (buffedOperator == '-') {
        result = intOp2 - intOp1;
    }

    if (buffedOperator == '*') {
        result = intOp2 * intOp1;
    }

    if (buffedOperator == '/') {
        return (int)(intOp2 / intOp1);
    }

    return result;
}

static void printInt(int data) {
    char output[1];
    sprintf(output, " = %d", data);
    uart_print(output);
}


static void printChar(char data) {
    char output[1];
    sprintf(output, "%c", data);
    uart_print(output);
}


/*----------------------------------------------------------------------------*/

int main(int argc, char** argv)
{
    int32_t retVal = -1;
    bufferPointer = 0;
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
