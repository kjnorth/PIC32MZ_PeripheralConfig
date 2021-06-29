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

#define PRINT_ACK 0xAC

void Print(const char* str, ...);
void FlashLED1(void);

/*
 * 
 */
int main(int argc, char** argv) {
    //    printf("hello world\r\n");
    SYS_Initialize(NULL);

    uint8_t data = 0xA5;

    while (1) {
        FlashLED1();
        SPI4_Write(&data, 1);
        data++;
        Print("Hello Arduino, %d, %u\n\e", (int)-12, (uint8_t)158);
    }

    return (EXIT_SUCCESS);
}

void Print(const char* str, ...) {
    const uint8_t length = strlen(str);
    char message[length];
    va_list args;
    va_start(args, str);
    sprintf(message, str, args);
    UART1_Write(message, length);
    va_end(args);
    
    // simple test
//    UART1_Write((char*)str, strlen(str));
    
    // write str, loop until ack is received, illuminate red LED if ack never received
//    UART1_Read()
}

void FlashLED1(void) {
    LED1_Set();
    CORETIMER_DelayMs(500);
    LED1_Clear();
    CORETIMER_DelayMs(500);
}
