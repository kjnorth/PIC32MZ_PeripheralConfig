/* 
 * File:   Print.h
 * Author: Kodiak North
 * 
 * Description: The Print module uses UART5 in non-blocking mode to transmit
 * printf style messages to another unit to display over a serial port. When
 * a message is received, the other unit must respond with PRINT_ACK (defined
 * in Print.c) to inform the sending device that the message was received.
 * 
 * Usage: The Print_EnqueueMsg function adds a message to a queue to be
 * transmitted once all other transmissions are complete. Since this module is
 * non-blocking, the Print_Task function must be called repetitively to handle
 * the sending and receiving of messages and acknowledgments. ARDUINO LOOKING FOR NULL CHARACTER??
 *
 * Created on July 13, 2021, 10:06 AM
 */

#include <stdint.h>
#include <string.h>

#include "Print.h"
#include "definitions.h" // does it make sense to include all of these??

// **** MODULE DEFINES ****
#define PRINT_MAX_MSGS 5u
#define PRINT_MAX_STR_LEN 256u
#define PRINT_START 0xA5u
#define PRINT_ACK 0xF9u
// **** END MODULE DEFINES ****

// **** MODULE STRUCTS ****

typedef struct {
    char msgQueue[PRINT_MAX_MSGS][PRINT_MAX_STR_LEN];
    uint8_t msgSize[PRINT_MAX_MSGS]; // holds the length of each message at the corresponding index
    uint8_t index; // current index in the msgQueue
    uint8_t size; // number of messages enqueued
} print_queue_t;
print_queue_t Queue;

typedef enum {
    SEND_START,
    SEND_MSG,
    SEND_START_VERIFY_ACK,
} print_state_t;
print_state_t printState;

typedef struct {
    volatile bool isRxErrorDetected;
    volatile bool isTxFinished;
    volatile bool isRxFinished;
} uart_flags_t;
uart_flags_t Uart5;
// **** END MODULE STRUCTS ****

// **** MODULE GLOBAL VARIABLES ****
UART_ERROR Errors;
// **** END MODULE GLOBAL VARIABLES ****

// **** MODULE FUNCTION PROTOTYPES ****
void UART5_WriteCallback(uintptr_t context);
void UART5_ReadCallback(uintptr_t context);
// **** END MODULE FUNCTION PROTOTYPES ****

void Print_Init(void) {
    Queue.index = 0;
    Queue.size = 0;
    printState = SEND_START;
    Uart5.isRxErrorDetected = false;
    Uart5.isTxFinished = false;
    Uart5.isRxFinished = true; // init to true for Print_Task to kick off first message that is enqueued
    UART5_WriteCallbackRegister(UART5_WriteCallback, 0);
    UART5_ReadCallbackRegister(UART5_ReadCallback, 0);
}

bool Print_EnqueueMsg(const char* fmt, ...) {
    bool returnVal = false;

    // allocate 2x space in case fmt passed is too long to avoid errors before length check
    char message[2 * PRINT_MAX_STR_LEN];
    // format message based on the fmt passed along with variadic arguments
    va_list args;
    va_start(args, fmt);
    vsprintf(message, fmt, args);
    va_end(args);
    uint16_t sizeMessage = strlen(message); // do I want +1 to send \0 char? what happens if msg contains a 0 in the middle
    // when receiver is looking for \0 as a terminator?

    if ((sizeMessage <= PRINT_MAX_STR_LEN) && !Print_IsQueueFull()) {
        // copy the message into the queue
        memcpy(Queue.msgQueue[Queue.index], message, sizeMessage);
        Queue.msgSize[Queue.index] = sizeMessage;
        Queue.size++;
        returnVal = true;
    }
    return returnVal;
}

bool Print_IsQueueFull(void) {
    return (Queue.size >= PRINT_MAX_MSGS);
}

void Print_Task(void) {
    static uint8_t response = 0;

    // wait for a message to be enqueued
    if (Queue.size > 0) { // THIS WILL WORK, BUT ACK IS NOT CHECKED UNTIL ANOTHER MESSAGE IS ENQUEUED.. OKAY WITH THAT?
        if (uart5.isRxErrorDetected) {
            uart5.isRxErrorDetected = false;
            // do something to indicate that an error occurred
            //        LED1_Set();
        } else if (uart5.isRxFinished) {
            /* send start byte or msg or toggle LED if ack not received correctly */
            uart5.isRxFinished = false;

            char* nextWrite = NULL;
            uint8_t sizeNextWrite = 0;
            const static uint8_t startByte = PRINT_START; // static so memory is not erased before UART can write it

            switch (printState) {
                case SEND_START:
                    nextWrite = (char*) &startByte;
                    sizeNextWrite = 1;
                    response = 0;
                    printState = SEND_MSG;
                    break;
                case SEND_MSG:
                    if (response == PRINT_ACK) {
                        // TODO: create a dequeue function to clean up this case AFTER IT WORKS
                        nextWrite = (char*) Queue.msgQueue[Queue.index]; // need to cast?
                        sizeNextWrite = Queue.msgSize[Queue.index];
                        Queue.index++;
                        if (Queue.index >= PRINT_MAX_MSGS) {
                            // perform the modulo operation when necessary
                            Queue.index = Queue.index % PRINT_MAX_MSGS;
                        }
                        Queue.size--;
                        printState = SEND_START_VERIFY_ACK;
                    } else {
                        LED1_Set();
                    }
                    break;
                case SEND_START_VERIFY_ACK:
                    if (response == PRINT_ACK) {
                        nextWrite = (char*) &startByte;
                        sizeNextWrite = 1;
                        response = 0;
                        printState = SEND_MSG;
                    } else {
                        LED1_Set();
                    }
                    break;
            }
            // check if write is busy?.. then what to do if it is busy? rxFinished false now
            UART5_Write(nextWrite, sizeNextWrite);

        } else if (uart5.isTxFinished) {
            /* initiate read of ack */
            uart5.isTxFinished = false;
            UART5_Read(&response, 1);
        }
    }
}

/**
 * function called when UART5 finishes transmitting data
 */
void UART5_WriteCallback(uintptr_t context) {
    Uart5.isTxFinished = true;
}

/**
 * function called when UART5 finishes reading data
 */
void UART5_ReadCallback(uintptr_t context) {
    Errors = UART5_ErrorGet();
    if (Errors != UART_ERROR_NONE) {
        /* ErrorGet clears errors, set error flag to notify console */
        Uart5.isRxErrorDetected = true;
    } else {
        uart5.isRxFinished = true;
    }
}