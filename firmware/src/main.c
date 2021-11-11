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
#include "MH_AX.h"
#include "NVData.h"
#include "Print.h"
#include "SoftPWM.h"
#include "Time.h"

#include "definitions.h"

#include "ax/ax.h"
#include "ax/ax_hw.h"
#include "ax/ax_modes.h"
#include "ax/ax_reg.h"
#include "ax/ax_reg_values.h"

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
bool IsValueWithinRange(int32_t valueToCheck, int32_t valueForCompare, int32_t leftSlop, int32_t rightSlop);

#define AX_RECEIVER
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
    
    Print_EnqueueMsg("Hello Arduino from the PIC32.\n");

    ax_mode_t mode;
    uint16_t checkTimeMs;
#ifdef AX_RECEIVER    
    mode = AX_MODE_PRX;
    checkTimeMs = 2500;
#else
    mode = AX_MODE_PTX;
    checkTimeMs = 5000;
#endif
    if (AX_Init(mode)) {
        Print_EnqueueMsg("ax init okay\n");
    } else {
        Print_EnqueueMsg("ax init falied!\n");
    }
    
    while (1) {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks();
        Print_Task();
        IMU_SampleTask();

        unsigned long ct = Time_GetMs();      
        static unsigned long pt = 0;
        if (ct - pt >= checkTimeMs) {
            pt = ct;
#ifdef AX_RECEIVER
            uint8_t temp[10];
            AX_ReceivePacket(temp);
#else
            /* ORDER
             * 0 - demo packet len
             * 1 - dest addr low
             * 2 - dest addr high
             * DEMO PACKET STARTS NOW
             * 3 - counter low
             * 4 - counter high
             * 5-8 - dummy data 
             */
            static uint8_t packet[9] = { 0x09, 0x33, 0x34, 0x00, 0x00, 0x55, 0x66, 0x77, 0x88 };
            static uint16_t counter = 0;
            counter++;
            packet[3] = (uint8_t) (counter & 0x00FF);
            packet[4] = (uint8_t) ((counter & 0xFF00) >> 8);
//            int ret = 
            AX_TransmitPacket(packet, 9);
//            Print_EnqueueMsg("tx good %d\n", ret);
            
            // how do i tell if i got an ACK??
#endif        
        }
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

bool IsValueWithinRange(int32_t valueToCheck, int32_t valueForCompare, int32_t leftSlop, int32_t rightSlop) {
    return (valueToCheck >= (valueForCompare - leftSlop)) && (valueToCheck <= (valueForCompare + rightSlop));
}