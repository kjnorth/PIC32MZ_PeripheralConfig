/* 
 * File:   main.c
 * Author: Kodiak North
 *
 * Created on June 10, 2021, 4:07 PM
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Encoders.h"
#include "NVData.h"
#include "Print.h"
#include "Time.h"

#include "definitions.h"

#define SW_VERSION 0.1f

#define ADC_VREF                (3.3f)
#define ADC_MAX_COUNT           (4095)
#define ADC_VOLTAGE_THRESH_HIGH (2100)
#define ADC_VOLTAGE_THRESH_LOW  (2000)

#define PWM_MAX_DUTY_CYCLE      (100u)

uint16_t g_adc_count = 0;
volatile uint8_t dutyCycle = 0;

void ADCHS_Callback(ADCHS_CHANNEL_NUM channel, uintptr_t context);
uint16_t PWM_CompareValueGet(uint8_t dutyCycle);
void SPIComm(uint8_t sendData);

/*
 * 
 */
int main(int argc, char** argv) {
    SYS_Initialize(NULL); // inits all peripherals - UART, SPI, TMR, OCMP, etc.
    LED1_Clear();
    Encoders_Init();
    Print_Init();
    ADCHS_CallbackRegister(ADCHS_CH3, ADCHS_Callback, (uintptr_t) NULL);
    TMR3_Start(); // for ADC    
    OCMP4_Enable();
    TMR2_Start(); // for OCMP4
    NVData_Init();

    Print_EnqueueMsg("Hello Arduino from the new print module version %0.2f\n", SW_VERSION);

    while (1) {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks();

        static bool isTested = false;
        if (dutyCycle > 90 && !isTested) {
            NVData_Test();
            isTested = true;
        }
        
        unsigned long ct = Time_GetMs();
        static unsigned long pt = 0;
        if (ct - pt >= 5000) {
            pt = ct;
            Print_EnqueueMsg("enc1 count %ld, enc2 count %ld, duty cycle %u\n",
                    Encoders_GetCount(ENC1), Encoders_GetCount(ENC2), dutyCycle);
        }
        Print_Task();
    }

    return (EXIT_SUCCESS);
}

/**
 * apply a variable voltage to pin RB3 with a potentiometer or signal, then
 * route OC4 to an LED and watch the brightness change as input voltage varies :)
 */
void ADCHS_Callback(ADCHS_CHANNEL_NUM channel, uintptr_t context) {
    g_adc_count = ADCHS_ChannelResultGet(ADCHS_CH3);
    // set pwm based on g_adc_count
    dutyCycle = (uint8_t) (g_adc_count * ((float) PWM_MAX_DUTY_CYCLE / (float) ADC_MAX_COUNT));
    OCMP4_CompareSecondaryValueSet(PWM_CompareValueGet(dutyCycle));
}

/**
 * 
 * @param dutyCycle - value from 0 to 100
 * @return OCxRS - OCMPx secondary register value for the desired duty cycle
 */
uint16_t PWM_CompareValueGet(uint8_t dutyCycle) {
    uint16_t OCxRS = 0;
    if (dutyCycle <= PWM_MAX_DUTY_CYCLE) {
        // OCxRS = [dutyCycle * (PR + 1)] : 0 <= dutyCycle <= 1
        OCxRS = (uint16_t) (((float) dutyCycle / (float) PWM_MAX_DUTY_CYCLE) * (TMR2_PeriodGet() + 1));
    }
    return OCxRS;
}

void SPIComm(uint8_t sendData) {
    uint8_t recData = 0;
    SPI4_WriteRead(&sendData, 1, &recData, 1);
}