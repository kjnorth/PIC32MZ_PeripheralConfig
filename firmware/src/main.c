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

void Print(const char* fmt, ...);
void FlashLED1(void);

/*
 * 
 */
int main(int argc, char** argv) {
    SYS_Initialize(NULL);
    // test the Print function.. it works like a charm ;)
    Print("Hello Arduino, %d, %u, %0.3f\n", (int) - 12, (uint8_t) 158, 54.368);

    uint8_t sendData = 0xA5;
    uint8_t recData = 0;

    while (1) {
        CORETIMER_DelayMs(1000);
        SPI4_WriteRead(&sendData, 1, &recData, 1);
        sendData++;
        // write recData to Arduino via UART to display on a serial port
        Print("PIC32 received 0x%02X\n", recData);
    }

    return (EXIT_SUCCESS);
}

/** 
 * function to write a formatted string over the UART1 peripheral
 * and wait for an ACK
 */
#define PRINT_STR_MAX_LEN 255u
#define PRINT_WAIT_FOR_ACK_DELAY 5u // ms
#define PRINT_WAIT_FOR_ACK_NUM_RETRIES 3u
#define PRINT_EXPECTED_ACK 0xACu
void Print(const char* fmt, ...) {
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

void FlashLED1(void) {
    LED1_Set();
    CORETIMER_DelayMs(500);
    LED1_Clear();
    CORETIMER_DelayMs(500);
}
