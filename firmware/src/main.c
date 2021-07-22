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

#define ADC_VREF                (3.3f)
#define ADC_MAX_COUNT           (4095)
#define ADC_VOLTAGE_THRESH_HIGH (2100)
#define ADC_VOLTAGE_THRESH_LOW  (2000)

bool g_adc_voltage_high = false;
uint16_t g_adc_count;
float g_input_voltage;
/*
 * 
 */
int main(int argc, char** argv) {
    SYS_Initialize(NULL);
    Encoders_Init();
    Print_Init();
    LED1_Clear();
    TMR3_Start();

    Print_EnqueueMsg("Hello Arduino from the new print module version %0.2f\n", SW_VERSION);

    while (1) {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks ( );
        
        unsigned long ct = Time_GetMs();
        static unsigned long pt = 0;
        if (ct - pt >= 500) {
            pt = ct;
            Print_EnqueueMsg("enc1 count is %ld\n", Encoders_GetCount(ENC1));
        }
        Print_Task();
        
        if (ADCHS_ChannelResultIsReady(ADCHS_CH3)) {
            /* Read the ADC result */
            g_adc_count = ADCHS_ChannelResultGet(ADCHS_CH3);
            bool current_voltage_state;
            if (g_adc_count >= ADC_VOLTAGE_THRESH_HIGH) {
                current_voltage_state = true;
            } else if (g_adc_count <= ADC_VOLTAGE_THRESH_LOW) {
                current_voltage_state = false;
            } else {
                current_voltage_state = g_adc_voltage_high;
            }
            if (current_voltage_state != g_adc_voltage_high) {
                LED1_Toggle();
                g_adc_voltage_high = current_voltage_state;
            }
        }
    }

    return (EXIT_SUCCESS);
}
