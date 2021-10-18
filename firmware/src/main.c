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

#include "ax/ax.h"
#include "ax/ax_hw.h"
#include "ax/ax_modes.h"
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

static unsigned char dataToReceive[256];
void mplab_spi_transfer(unsigned char* data, uint8_t length) {
    SPI1_WriteRead(data, length, dataToReceive, length);
    memcpy(data, dataToReceive, length);
}


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

    
//  ax_packet rx_pkt;
//  uint8_t tx_pkt[0x100];

  ax_config config;
  memset(&config, 0, sizeof(ax_config));

  config.clock_source = AX_CLOCK_SOURCE_TCXO;
  config.f_xtal = 16369000;

  config.synthesiser.A.frequency = 434600000;
  config.synthesiser.B.frequency = 434600000;

  config.spi_transfer = mplab_spi_transfer;

  config.pkt_store_flags = AX_PKT_STORE_RSSI |
    AX_PKT_STORE_RF_OFFSET;


  /* ------- init ------- */
  ax_init(&config);
  ax_default_params(&config, &gmsk_hdlc_fec_modulation);
  ax_rx_on(&config, &gmsk_hdlc_fec_modulation);

    
    
    
    Print_EnqueueMsg("Hello Arduino from the new print module version %0.2f\n", SW_VERSION);

    while (1) {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks();

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

bool IsValueWithinRange(int32_t valueToCheck, int32_t valueForCompare, int32_t leftSlop, int32_t rightSlop) {
    return (valueToCheck >= (valueForCompare - leftSlop)) && (valueToCheck <= (valueForCompare + rightSlop));
}