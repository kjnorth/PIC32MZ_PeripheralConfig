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
#include "IMU_HWT901B.h"
#include "NVData.h"
#include "Print.h"
#include "SoftPWM.h"
#include "Time.h"

#include "definitions.h"

#define SW_VERSION 0.1f

#define ADC_VREF                (3.3f)
#define ADC_MAX_COUNT           (4095)
#define ADC_VOLTAGE_THRESH_HIGH (2100)
#define ADC_VOLTAGE_THRESH_LOW  (2000)

#define PWM_MAX_DUTY_CYCLE      (100u)

uint16_t g_adc_count = 0;
volatile uint8_t curDutyCycle = 0;
uint8_t preDutyCycle = 0;

void ADCHS_Callback(ADCHS_CHANNEL_NUM channel, uintptr_t context);
uint16_t PWM_CompareValueGet(uint8_t curDutyCycle);
void SPIComm(uint8_t sendData);
bool IsValueWithinRange(int32_t valueToCheck, int32_t valueForCompare, int32_t leftSlop, int32_t rightSlop);

/*
 * 
 */
int main(int argc, char** argv) {
    SYS_Initialize(NULL); // inits all peripherals - UART, SPI, TMR, OCMP, etc.
    LED1_Clear();
    Encoders_Init();
    Print_Init();
    IMU_Init();
    ADCHS_CallbackRegister(ADCHS_CH3, ADCHS_Callback, (uintptr_t) NULL);
    TMR3_Start(); // for ADC    
    OCMP4_Enable();
    TMR2_Start(); // for OCMP4
    NVData_Init();
    SoftPWM_Init();
    SoftPWM_PinAdd(PWM_SOFT1_PIN, SOFT_PWM_PIN_NORMAL);
    SoftPWM_PinAdd(PWM_SOFT2_PIN, SOFT_PWM_PIN_INVERTED);

    SoftPWM_PinSetDuty(PWM_SOFT1_PIN, 25);
    SoftPWM_PinEnable(PWM_SOFT1_PIN);

    SoftPWM_PinSetDuty(PWM_SOFT2_PIN, 50);
    SoftPWM_PinEnable(PWM_SOFT2_PIN);

    Print_EnqueueMsg("Hello Arduino from the new print module version %0.2f\n", SW_VERSION);

    while (1) {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks();
        
        if (!IsValueWithinRange(curDutyCycle, preDutyCycle, 2, 2)) {
            preDutyCycle = curDutyCycle;
            if (curDutyCycle <= 5) {
                SoftPWM_SetFrequency(1000);
            } else if (curDutyCycle > 5 && curDutyCycle <= 35) {
                SoftPWM_SetFrequency(2000);
            } else if (curDutyCycle > 35 && curDutyCycle <= 60) {
                SoftPWM_SetFrequency(3000);
            } else if (curDutyCycle > 60 && curDutyCycle <= 80) {
                SoftPWM_SetFrequency(4000);
            } else if (curDutyCycle > 80 && curDutyCycle <= 100) {
                SoftPWM_SetFrequency(5000);
            }
        }

        /* test of the nvm module
        static bool isTested = false;
        if (curDutyCycle > 90 && !isTested) {
            NVData_Test();
            isTested = true;
        } //*/

        unsigned long ct = Time_GetMs();
        static unsigned long pt = 0;
        if (ct - pt >= 1000) {
            pt = ct;
//            Print_EnqueueMsg("enc1 count %ld, enc2 count %ld, duty cycle %u, freq %u\n",
//                    Encoders_GetCount(ENC1), Encoders_GetCount(ENC2), curDutyCycle, SoftPWM_GetFrequency());
            Print_EnqueueMsg("roll %0.2f, pitch %0.2f\n", IMU_RollGet(), IMU_PitchGet());
        }
        Print_Task();
        IMU_SampleTask();
    }

    return (EXIT_SUCCESS);
}

/**
 * apply a variable voltage to pin RB3 with a potentiometer or signal, then
 * route OC4 to an LED and watch the brightness change as input voltage varies :)
 */
void ADCHS_Callback(ADCHS_CHANNEL_NUM channel, uintptr_t context) {
    g_adc_count = ADC_MAX_COUNT - ADCHS_ChannelResultGet(ADCHS_CH3);
    // set pwm based on g_adc_count
    curDutyCycle = (uint8_t) (g_adc_count * ((float) PWM_MAX_DUTY_CYCLE / (float) ADC_MAX_COUNT));
    OCMP4_CompareSecondaryValueSet(PWM_CompareValueGet(curDutyCycle));
    curDutyCycle = PWM_MAX_DUTY_CYCLE - curDutyCycle;
}

/**
 * 
 * @param curDutyCycle - value from 0 to 100
 * @return OCxRS - OCMPx secondary register value for the desired duty cycle
 * @NOTE this function can be turned into a look up table to use less processing power
 */
uint16_t PWM_CompareValueGet(uint8_t curDutyCycle) {
    uint16_t OCxRS = 0;
    if (curDutyCycle <= PWM_MAX_DUTY_CYCLE) {
        // OCxRS = [duty * (PR + 1)] : 0 <= duty <= 1
        OCxRS = (uint16_t) (((float) curDutyCycle / (float) PWM_MAX_DUTY_CYCLE) * (TMR2_PeriodGet() + 1));
    }
    return OCxRS;
}

void SPIComm(uint8_t sendData) {
    uint8_t recData = 0;
    SPI4_WriteRead(&sendData, 1, &recData, 1);
}

bool IsValueWithinRange(int32_t valueToCheck, int32_t valueForCompare, int32_t leftSlop, int32_t rightSlop) {
    return (valueToCheck >= (valueForCompare - leftSlop)) && (valueToCheck <= (valueForCompare + rightSlop));
}