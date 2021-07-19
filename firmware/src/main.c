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

#include "definitions.h"
#include "Print.h"

volatile uint32_t msTicks = 0;
void CORETIMER_InterruptCallback(uint32_t status, uintptr_t context);

/*
 * 
 */
int main(int argc, char** argv) {
    SYS_Initialize(NULL);
    Print_Init();
    CORETIMER_CallbackSet(CORETIMER_InterruptCallback, (uintptr_t) NULL);
    CORETIMER_Start();
    LED1_Clear();

    Print_EnqueueMsg("Hola Arduino from the new print module\n");
    Print_EnqueueMsg("I am trying my best\n");
    Print_EnqueueMsg("29 is all that I have\n");
    Print_EnqueueMsg("This is round %u\n", 4u);
    Print_EnqueueMsg("What if there is a %0.3f involved?\n", 7.123);
    Print_EnqueueMsg("this should NOT get sent\n");

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

void CORETIMER_InterruptCallback(uint32_t status, uintptr_t context) {
    msTicks++;
}
