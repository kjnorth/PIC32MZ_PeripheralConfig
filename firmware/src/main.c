/* 
 * File:   main.c
 * Author: Kodiak North
 *
 * Created on June 10, 2021, 4:07 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>

#include "peripheral/gpio/plib_gpio.h"
#include "peripheral/coretimer/plib_coretimer.h"
#include "peripheral/spi/spi_master/plib_spi4_master.h"
#include "definitions.h"

#define PRINT_STR_MAX_LEN 255u
#define PRINT_WAIT_FOR_ACK_DELAY 5u // ms
#define PRINT_WAIT_FOR_ACK_NUM_RETRIES 3u
#define PRINT_EXPECTED_ACK 0xACu

#define UART_BLOCKING 0 // set to 1 for blocking uart config, 0 for non-blocking uart config

// **** UART Definitions for Blocking/NonBlocking ****
#if UART_BLOCKING
void Print_BlockingUART(const char* fmt, ...);
#else
bool Print_NonBlockingUART(const char* fmt, ...);
void UART5_WriteCallback(uintptr_t context);
void UART5_ReadCallback(uintptr_t context);

UART_ERROR errors;

typedef struct {
    volatile bool isRxErrorDetected;
    volatile bool isTxFinished;
    volatile bool isRxFinished;
} uart_flags_t;
uart_flags_t uart5;
void UART_FlagsInit(uart_flags_t* uart);

uint8_t ack;
#endif
// **** **************************************** *****

volatile uint32_t msTicks; // notes - callback is necessary to track time, do not give yourself access to coreTmr obj.. msTicks must be volatile!
// why does it not work when using the coreTmr object??? confused
void CORETIMER_InterruptCallback(uint32_t status, uintptr_t context);

void SPIComm(uint8_t* sendData);

#define START 0xA5u
#define ACK 0xF9u

typedef enum {
    SEND_START,
    SEND_MSG,
    SEND_START_VERIFY_ACK,
} uart_send_t;
uart_send_t sendState = SEND_START;

void UARTComm(void);

/*
 * 
 */
int main(int argc, char** argv) {
    SYS_Initialize(NULL);
    CORETIMER_CallbackSet(CORETIMER_InterruptCallback, (uintptr_t) NULL);
    CORETIMER_Start();
    LED1_Clear();

    // test the Print_BlockingUART function.. it works like a charm ;)
#if UART_BLOCKING
    Print_BlockingUART("Hello Arduino, %d, %u, %0.3f\n", (int) - 12, (uint8_t) 158, 54.368);
#else
    UART_FlagsInit(&uart5);
    UART5_WriteCallbackRegister(UART5_WriteCallback, 0);
    UART5_ReadCallbackRegister(UART5_ReadCallback, 0);
#endif

    while (1) {
        unsigned long ct = msTicks;
        static unsigned long pt = 0;
        if (ct - pt >= 100) {
            pt = ct;
            UARTComm();
        }
    }

    return (EXIT_SUCCESS);
}

void UARTComm(void) {
    static uint8_t response = 0;

    if (uart5.isRxErrorDetected) {
        /* Send error message to console */
        uart5.isRxErrorDetected = false;
        //        LED1_Set();
    } else if (uart5.isRxFinished) {
        /* send start byte or msg or toggle LED if ack not received correctly */
        uart5.isRxFinished = false;

        //        char filler[255] = {0};
        char msg[] = "Hello\n";
        char* nextWrite = (char*) &msg; //&filler;
        uint8_t sizeNextWrite = 0;
        uint8_t start = START; // once working, make this a local var and see what happens

        SPIComm(&response);

        switch (sendState) {
            case SEND_START:
            {
                LED1_Clear();
                //                uint8_t start = START;
                nextWrite = (char*) &start;
                sizeNextWrite = 1;
                sendState = SEND_MSG;
                break;
            }
            case SEND_MSG:
                LED1_Set();
//                if (response == ACK) {
                    nextWrite = (char*) &msg;
                    sizeNextWrite = 7; //sizeof (msg);
                    sendState = SEND_START_VERIFY_ACK;
                    //                    LED1_Clear();
//                } else {
                    //                    LED1_Set();
                    //                    SPIComm(response);
                    //                    LED1_Toggle();
//                }
                break;
            case SEND_START_VERIFY_ACK:
                LED1_Clear();
//                if (response == ACK) {
                    nextWrite = (char*) &start;
                    sizeNextWrite = 1;
                    sendState = SEND_MSG;
//                } else {
                    //                    SPIComm(response);
                    //                    LED1_Toggle();
//                }
                break;
        }

        UART5_Write(nextWrite, sizeNextWrite);
    } else if (uart5.isTxFinished) {
        /* initiate read of ack */
        uart5.isTxFinished = false;
        UART5_Read(&response, 1);
    }
}

void SPIComm(uint8_t* sendData) {
    uint8_t recData = 0;
    SPI4_WriteRead(sendData, 1, &recData, 1);
}

#if UART_BLOCKING

/** 
 * function to write a formatted string over the UART5 peripheral
 * and wait for an ACK
 */
void Print_BlockingUART(const char* fmt, ...) {
    // format message based on the fmt passed along with variadic arguments
    char message[PRINT_STR_MAX_LEN];
    va_list args;
    va_start(args, fmt);
    vsprintf(message, fmt, args);
    // write message via UART5
    UART5_Write(message, strlen(message));
    va_end(args);
    // wait for ACK
    uint8_t retries = PRINT_WAIT_FOR_ACK_DELAY;
    do {
        CORETIMER_DelayMs(PRINT_WAIT_FOR_ACK_NUM_RETRIES);
        retries--;
    } while (!UART5_ReceiverIsReady() && (retries > 0));
    // verify ACK
    uint8_t ack = (uint8_t) UART5_ReadByte();
    if (ack != PRINT_EXPECTED_ACK) {
        // ACK not correct, illuminate built in LED1 for a short period of time
        LED1_Set();
        CORETIMER_DelayMs(2500);
        LED1_Clear();
    }
}

#else

bool Print_NonBlockingUART(const char* fmt, ...) {
    bool returnVal = false;
    if (!UART5_WriteIsBusy()) {
        // format message based on the fmt passed along with variadic arguments
        char message[PRINT_STR_MAX_LEN];
        va_list args;
        va_start(args, fmt);
        vsprintf(message, fmt, args);
        // write message via UART5
        UART5_Write(message, strlen(message));
        va_end(args);
        returnVal = true;
    }
    return returnVal;
}

/**
 * function called when UART5 finishes transmitting data I THINK
 */
void UART5_WriteCallback(uintptr_t context) {
    uart5.isTxFinished = true;
}

/**
 * function called when UART5 finishes reading data I THINK
 */
void UART5_ReadCallback(uintptr_t context) {
    errors = UART5_ErrorGet();
    if (errors != UART_ERROR_NONE) {
        /* ErrorGet clears errors, set error flag to notify console */
        uart5.isRxErrorDetected = true;
    } else {
        uart5.isRxFinished = true;
    }
}

void UART_FlagsInit(uart_flags_t* uart) {
    uart->isRxErrorDetected = false;
    uart->isRxFinished = true;
    uart->isTxFinished = false;
}

#endif

void CORETIMER_InterruptCallback(uint32_t status, uintptr_t context) {
    msTicks++;
}
