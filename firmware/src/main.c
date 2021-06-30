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
#define PRINT_ACK 0xACu

void Print(const char* fmt, ...);
void FlashLED1(void);

/*
 * 
 */
int main(int argc, char** argv) {
    SYS_Initialize(NULL);
    Print("Hello Arduino, %d, %u, %0.3f\n", (int) - 12, (uint8_t) 158, 54.368);

    uint8_t sendData = 0xA5;
    uint8_t recData = 0;

    while (1) {
        FlashLED1();
//        SPI4_Write(&sendData, 1);
//        sendData++;
//        SPI4_Read(&recData, 1);
        SPI4_WriteRead(&sendData, 1, &recData, 1);
        sendData++;
        Print("PIC32 received 0x%02X\n", recData);
    }

    return (EXIT_SUCCESS);
}

void Print(const char* fmt, ...) {
    // format message based on the fmt passed along with variadic arguments
    char message[PRINT_STR_MAX_LEN];
    va_list args;
    va_start(args, fmt);
    vsprintf(message, fmt, args);
    // write to Arduino via UART1
    UART1_Write(message, strlen(message));
    va_end(args);
    // wait for Arduino to ACK
    uint8_t ack = 0;
    do {
        ack = (uint8_t) UART1_ReadByte();
    } while (ack != PRINT_ACK);
}

void FlashLED1(void) {
    LED1_Set();
    CORETIMER_DelayMs(500);
    LED1_Clear();
    CORETIMER_DelayMs(500);
}
