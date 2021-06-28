/* 
 * File:   main.c
 * Author: Kodiak North
 *
 * Created on June 10, 2021, 4:07 PM
 */

#include <stdio.h>
#include <stdlib.h>

#include "peripheral/gpio/plib_gpio.h"
#include "peripheral/coretimer/plib_coretimer.h"

/*
 * 
 */
int main(int argc, char** argv) {
//    printf("hello world\r\n");

    GPIO_Initialize();

    uint16_t delay = 250;

    while (1) {
        LED1_Set();
        CORETIMER_DelayMs(delay);
        LED1_Clear();
        CORETIMER_DelayMs(delay);
    }

    return (EXIT_SUCCESS);
}
