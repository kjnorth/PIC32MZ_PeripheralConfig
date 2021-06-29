/* 
 * File:   main.c
 * Author: Kodiak North
 *
 * Created on June 10, 2021, 4:07 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "peripheral/gpio/plib_gpio.h"
#include "peripheral/coretimer/plib_coretimer.h"
#include "peripheral/spi/spi_master/plib_spi4_master.h"
#include "definitions.h"

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
        CORETIMER_DelayMs(100);
    }

    return (EXIT_SUCCESS);
}

void FlashLED1(void) {
    LED1_Set();
    CORETIMER_DelayMs(500);
    LED1_Clear();
    CORETIMER_DelayMs(500);
}
