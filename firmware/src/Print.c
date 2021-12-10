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
 * the sending and receiving of messages and acknowledgments. 
 * 
 * Notes:
 * Maximum message size is limited to PRINT_MAX_STR_LEN.
 * The receiving unit reads bytes until the string NULL terminator '\0' is read
 *
 * Created on July 13, 2021, 10:06 AM
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "Print.h"
#include "Time.h"
#include "UART_Common.h"

#include "peripheral/gpio/plib_gpio.h"
#include "peripheral/uart/plib_uart5.h"

// **** MODULE DEFINES ****
#define PRINT_MAX_MSGS (50u)
#define PRINT_MAX_STR_LEN (256u)
#define PRINT_START (0xA5u)
#define PRINT_ACK (0xF9u)
#define PRINT_READ_TIMEOUT_MS (10u)
// **** END MODULE DEFINES ****

// **** MODULE TYPEDEFS ****

typedef struct {
    char msgQueue[PRINT_MAX_MSGS][PRINT_MAX_STR_LEN];
    uint8_t msgSize[PRINT_MAX_MSGS]; // holds the length of each message at the corresponding index
    uint8_t index; // current index in the msgQueue
    uint8_t size; // number of messages enqueued
} print_queue_t;
static print_queue_t Queue;

typedef enum {
    SEND_START,
    VERIFY_ACK_START_SEND_MSG,
    VERIFY_ACK_MSG,
} print_state_t;
// **** END MODULE TYPEDEFS ****

// **** MODULE GLOBAL VARIABLES ****
static print_state_t PrintState;
static uart_interrupt_flags_t Uart5;
// **** END MODULE GLOBAL VARIABLES ****

// **** MODULE FUNCTION PROTOTYPES ****
static bool Print_DequeueMsg(char** msg, uint8_t* size); // returns a pointer to the msg, and stores the size of the message in uint8_t* size
static void UART5_WriteCallback(uintptr_t context);
static void UART5_ReadCallback(uintptr_t context);
// **** END MODULE FUNCTION PROTOTYPES ****

void Print_Init(void) {
    Queue.index = 0;
    Queue.size = 0;
    PrintState = SEND_START;
    Uart5.IsRxErrorDetected = false;
    Uart5.IsTxFinished = false;
    Uart5.IsRxFinished = true; // init to true so Print_Task kicks off the first message that is enqueued
    Uart5.Errors = UART_ERROR_NONE;
    UART5_WriteCallbackRegister(UART5_WriteCallback, (uintptr_t) NULL);
    UART5_ReadCallbackRegister(UART5_ReadCallback, (uintptr_t) NULL);
    Time_Init();
}

bool Print_EnqueueMsg(const char* fmt, ...) {
    bool returnVal = false;

    // allocate 2x space in case fmt passed is too long to avoid errors before length check
    char message[2 * PRINT_MAX_STR_LEN] = {0};
    // format message based on the fmt passed along with variadic arguments
    va_list args;
    va_start(args, fmt);
    vsprintf(message, fmt, args);
    uint16_t sizeMessage = strlen(message) + sizeof (uint8_t); // + sizeof (uint8_t) to include NULL terminator
    va_end(args);

    if ((sizeMessage <= PRINT_MAX_STR_LEN) && !Print_IsQueueFull()) {
        // copy the message into the queue
        uint8_t messageToAddIndex = Queue.index + Queue.size;
        if (messageToAddIndex >= PRINT_MAX_MSGS) {
            // perform the modulo operation when necessary
            messageToAddIndex = messageToAddIndex % PRINT_MAX_MSGS;
        }
        memcpy(Queue.msgQueue[messageToAddIndex], message, sizeMessage);
        Queue.msgSize[messageToAddIndex] = sizeMessage;
        Queue.size++;
        returnVal = true;
    }
    return returnVal;
}

void Print_EnqueueBuffer(uint8_t* b, uint8_t len) {
    if (len != PRINT_BUFFER_SIZE) {
        Print_EnqueueMsg("ERROR - cannot enqueue buffer, size must be %u\n", PRINT_BUFFER_SIZE);
        return;
    }
    // 32 bytes of buf per message
//    uint8_t i;
//    for (i = 0; i < 1/*PRINT_BUFFER_SIZE*/; i += 32) {
//        Print_EnqueueMsg("%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
//                b[i], b[i+1], b[i+2], b[i+3], b[i+4], b[i+5], b[i+6], b[i+7], b[i+8], b[i+9], b[i+10], b[i+11], b[i+12], b[i+13], b[i+14], b[i+15],
//                b[i+16], b[i+17], b[i+18], b[i+19], b[i+20], b[i+21], b[i+22], b[i+23], b[i+24], b[i+25], b[i+26], b[i+27], b[i+28], b[i+29], b[i+30], b[i+31]);
//    } 
}

bool Print_DequeueMsg(char** msg, uint8_t* size) {
    bool returnVal = false;
    if (!Print_IsQueueEmpty()) {
        *msg = Queue.msgQueue[Queue.index];
        *size = Queue.msgSize[Queue.index];
        Queue.index++;
        if (Queue.index >= PRINT_MAX_MSGS) {
            // perform the modulo operation when necessary
            Queue.index = Queue.index % PRINT_MAX_MSGS;
        }
        Queue.size--;
        returnVal = true;
    }
    return returnVal;
}

bool Print_IsQueueEmpty(void) {
    return (Queue.size == 0);
}

bool Print_IsQueueFull(void) {
    return (Queue.size >= PRINT_MAX_MSGS);
}

void Print_Task(void) {
    static uint8_t response = 0;
    static uint32_t readStartTime = 0;

    if (Uart5.IsRxErrorDetected) {
        Uart5.IsRxErrorDetected = false;
        // do something to indicate that an error occurred and start over
        LED1_Set();
        UART5_ReadAbort();
        PrintState = SEND_START;
    } else if (Uart5.IsRxFinished) {
        /* send start byte or msg or illuminate LED if ack not received correctly */
        char* nextWrite = NULL;
        uint8_t sizeNextWrite = 0;
        const static uint8_t startByte = PRINT_START; // static so memory is not erased before UART can write it

        switch (PrintState) {
            case SEND_START:
                if (!Print_IsQueueEmpty()) {
                    // queue is non-empty, initiate the send of a message
                    nextWrite = (char*) &startByte;
                    sizeNextWrite = sizeof (uint8_t);
                    PrintState = VERIFY_ACK_START_SEND_MSG;
                }
                break;
            case VERIFY_ACK_START_SEND_MSG:
                if (response == PRINT_ACK) {
                    response = 0;
                    if (Print_DequeueMsg(&nextWrite, &sizeNextWrite)) {
                        PrintState = VERIFY_ACK_MSG;
                    }
                } else {
                    // error occurred
                    LED1_Set();
                }
                break;
            case VERIFY_ACK_MSG:
                if (response == PRINT_ACK) {
                    response = 0;
                    PrintState = SEND_START;
                } else {
                    // error occurred
                    LED1_Set();
                }
                break;
        }

        if (nextWrite != NULL) {
            UART5_Write(nextWrite, sizeNextWrite);
            Uart5.IsRxFinished = false; // only set IsRxFinished to false once data is sent
        }
    } else if (Uart5.IsTxFinished) {
        /* initiate read of ack */
        Uart5.IsTxFinished = false;
        UART5_Read(&response, sizeof (uint8_t));
        readStartTime = Time_GetMs();
    } else if (UART5_ReadIsBusy()) {
        uint32_t curTime = Time_GetMs();
        if (curTime - readStartTime > PRINT_READ_TIMEOUT_MS) {
            // receiving unit is not responding, abort the read and start over, note that the most recent dequeued message may be lost
            UART5_ReadAbort();
            PrintState = SEND_START;
            Uart5.IsRxFinished = true; // set to true to send start byte again
        }
    }
}

/**
 * function called when UART5 finishes transmitting data
 */
void UART5_WriteCallback(uintptr_t context) {
    Uart5.IsTxFinished = true;
}

/**
 * function called when UART5 finishes reading data
 */
void UART5_ReadCallback(uintptr_t context) {
    Uart5.Errors = UART5_ErrorGet();
    if (Uart5.Errors != UART_ERROR_NONE) {
        /* ErrorGet clears errors, set error flag to notify console */
        Uart5.IsRxErrorDetected = true;
    } else {
        Uart5.IsRxFinished = true;
    }
}
