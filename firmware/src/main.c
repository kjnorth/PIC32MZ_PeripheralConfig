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
#include "Print.h"

#define PRINT_WAIT_FOR_ACK_DELAY 5u // ms
#define PRINT_WAIT_FOR_ACK_NUM_RETRIES 3u
#define PRINT_EXPECTED_ACK 0xACu

#define UART_BLOCKING 0 // set to 1 for blocking uart config, 0 for non-blocking uart config

// **** UART Definitions for Blocking/NonBlocking ****
#if UART_BLOCKING
void Print_BlockingUART(const char* fmt, ...);
#else

#endif
// **** **************************************** *****

volatile uint32_t msTicks = 0;
void CORETIMER_InterruptCallback(uint32_t status, uintptr_t context);

void SPIComm(uint8_t* sendData);

/*
 * 
 */
int main(int argc, char** argv) {
    SYS_Initialize(NULL);
    Print_Init();
    CORETIMER_CallbackSet(CORETIMER_InterruptCallback, (uintptr_t) NULL);
    CORETIMER_Start();
    LED1_Clear();

    // test the Print_BlockingUART function.. it works like a charm ;)
#if UART_BLOCKING
    Print_BlockingUART("Hello Arduino, %d, %u, %0.3f\n", (int) - 12, (uint8_t) 158, 54.368);
#else
#endif
    
    Print_EnqueueMsg("Hola Arduino from the new print module\n");

    while (1) {
        unsigned long ct = msTicks;
        static unsigned long pt = 0;
        if (ct - pt >= 100) {
            pt = ct;
        }
        Print_Task();
    }

    return (EXIT_SUCCESS);
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

#endif

void CORETIMER_InterruptCallback(uint32_t status, uintptr_t context) {
    msTicks++;
}
