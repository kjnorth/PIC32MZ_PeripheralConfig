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
#include "Time.h"

#define SW_VERSION 0.1f

void EncChangeNoticeConfig(void) {
    CNCONJbits.ON = 1;
    CNCONJbits.EDGEDETECT = 1;
    Print_EnqueueMsg("cnconj is %u\n", CNCONJ);
}

/*
 * 
 */
int main(int argc, char** argv) {
    SYS_Initialize(NULL);
    Print_Init();
    LED1_Clear();

    Print_EnqueueMsg("Hello Arduino from the new print module version %0.2f\n", SW_VERSION);
    EncChangeNoticeConfig();

    while (1) {
        unsigned long ct = Time_GetMs();
        static unsigned long pt = 0;
        if (ct - pt >= 100) {
            pt = ct;
        }
        Print_Task();
    }

    return (EXIT_SUCCESS);
}
