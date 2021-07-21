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

#include "Encoders.h"
#include "Print.h"
#include "Time.h"

#include "definitions.h"

#define SW_VERSION 0.1f

/*
 * 
 */
int main(int argc, char** argv) {
    SYS_Initialize(NULL);
    Encoders_Init();
    Print_Init();
    LED1_Clear();

    Print_EnqueueMsg("Hello Arduino from the new print module version %0.2f\n", SW_VERSION);

    while (1) {
        unsigned long ct = Time_GetMs();
        static unsigned long pt = 0;
        if (ct - pt >= 500) {
            pt = ct;
            Print_EnqueueMsg("enc1 count is %ld\n", Encoders_GetCount(ENC1));
        }
        Print_Task();
    }

    return (EXIT_SUCCESS);
}
