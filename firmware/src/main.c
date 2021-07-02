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

#define UART_BLOCKING 1 // set to 1 for blocking uart config, 0 for non-blocking uart config

// **** UART Definitions for Blocking/NonBlocking ****
#if UART_BLOCKING
void Print_BlockingUART(const char* fmt, ...);
#else
void Print_NonBlockingUART(const char* fmt, ...);
void UART1_WriteCallback(uintptr_t context);
void UART1_ReadCallback(uintptr_t context);

extern UART_OBJECT uart1Obj;

bool errorStatus = false;
bool writeStatus = false;
bool readStatus = false;
uint8_t ack;
#endif
// **** **************************************** *****

volatile uint32_t msTicks; // notes - callback is necessary to track time, do not give yourself access to coreTmr obj.. msTicks must be volatile!
// why does it not work when using the coreTmr object??? confused
void CORETIMER_InterruptCallback(uint32_t status, uintptr_t context);

void FlashLED1(void);

/*
 * 
 */
int main(int argc, char** argv) {
    SYS_Initialize(NULL);
    CORETIMER_CallbackSet(CORETIMER_InterruptCallback, (uintptr_t) NULL);
    CORETIMER_Start();

    // test the Print_BlockingUART function.. it works like a charm ;)
#if UART_BLOCKING
    Print_BlockingUART("Hello Arduino, %d, %u, %0.3f\n", (int) - 12, (uint8_t) 158, 54.368);
#else
    //    UART1_WriteCallbackRegister(UART1_WriteCallback, 0);
    //    UART1_ReadCallbackRegister(UART1_ReadCallback, 0);
    Print_NonBlockingUART("Hello Arduino, %d, %u, %0.3f\n", (int) - 999, (uint8_t) 77, 49.187);
#endif

    uint8_t sendData = 0xA5;
    uint8_t recData = 0;

    while (1) {
        unsigned long ct = msTicks;
        static unsigned long pt = 0;
        if (ct - pt >= 1000) {
            LED1_Set();
            pt = ct;

            SPI4_WriteRead(&sendData, 1, &recData, 1);
            sendData++;
            // write recData to Arduino via UART to display on a serial port
#if UART_BLOCKING
//            Print_BlockingUART("PIC32 received 0x%02X, ms count %lu\n", recData, ct);
#else
            //        Print_NonBlockingUART("PIC32 received 0x%02X\n", recData);
                    Print_NonBlockingUART("PIC32 received some data\n");

            if (errorStatus) {
                errorStatus = false;
                // access the errors
                UART_ERROR errors = uart1Obj.errors;
                if ((errors & UART_ERROR_OVERRUN) > 0) {
                    //                LED1_Set();
                }
                if ((errors & UART_ERROR_FRAMING) > 0) {
                    //                LED1_Set();
                }
                if ((errors & UART_ERROR_PARITY) > 0) {
                    //                LED1_Set();
                }
            } else if (readStatus) {
                readStatus = false;
                LED1_Set();
                // Rx complete, no errors, verify ack
                if (ack != PRINT_EXPECTED_ACK) {
                    // ACK not correct, illuminate built in LED1 for a short period of time
                    //                LED1_Set();
                }
            } else if (writeStatus) {
                writeStatus = false;
                // Tx complete, read ack
                ack = 0;
                //            UART1_Read(&ack, 1);
                //    LED1_Clear();
            }
#endif
        }
    }

    return (EXIT_SUCCESS);
}

#if UART_BLOCKING

/** 
 * function to write a formatted string over the UART1 peripheral
 * and wait for an ACK
 */
void Print_BlockingUART(const char* fmt, ...) {
    // format message based on the fmt passed along with variadic arguments
    char message[PRINT_STR_MAX_LEN];
    va_list args;
    va_start(args, fmt);
    vsprintf(message, fmt, args);
    // write message via UART1
    UART1_Write(message, strlen(message));
    va_end(args);
    // wait for ACK
    uint8_t retries = PRINT_WAIT_FOR_ACK_DELAY;
    do {
        CORETIMER_DelayMs(PRINT_WAIT_FOR_ACK_NUM_RETRIES);
        retries--;
    } while (!UART1_ReceiverIsReady() && (retries > 0));
    // verify ACK
    uint8_t ack = (uint8_t) UART1_ReadByte();
    if (ack != PRINT_EXPECTED_ACK) {
        // ACK not correct, illuminate built in LED1 for a short period of time
        LED1_Set();
        CORETIMER_DelayMs(2500);
        LED1_Clear();
    }
}

#else

void Print_NonBlockingUART(const char* fmt, ...) {
    // format message based on the fmt passed along with variadic arguments
    char message[PRINT_STR_MAX_LEN];
    va_list args;
    va_start(args, fmt);
    vsprintf(message, fmt, args);
    // write message via UART1
    UART1_Write(message, strlen(message));
    va_end(args);
}

/**
 * function called when UART1 finishes transmitting data I THINK
 */
void UART1_WriteCallback(uintptr_t context) {
    writeStatus = true;
}

/**
 * function called when UART1 finishes reading data I THINK
 */
void UART1_ReadCallback(uintptr_t context) {
    if (UART1_ErrorGet() != UART_ERROR_NONE) {
        /* ErrorGet clears errors, set error flag to notify console */
        errorStatus = true;
    } else {
        readStatus = true;
    }
}

#endif

void CORETIMER_InterruptCallback(uint32_t status, uintptr_t context) {
    msTicks++;
}

void FlashLED1(void) {
    LED1_Set();
    CORETIMER_DelayMs(500);
    LED1_Clear();
    CORETIMER_DelayMs(500);
}
