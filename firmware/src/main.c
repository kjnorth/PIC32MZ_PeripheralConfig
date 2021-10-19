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

// AX5243 Test Code
#define AX_RECEIVER
static unsigned char dataToReceive[256];
void masterhaul_spi_transfer(unsigned char* data, uint8_t length) {
    SPI1_WriteRead(data, length, dataToReceive, length);
    memcpy(data, dataToReceive, length);
}
// END AX5243 Test Code

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

    ax_config config;
    memset(&config, 0, sizeof(ax_config));

    config.pwrmode = AX_PWRMODE_POWERDOWN;
    config.clock_source = AX_CLOCK_SOURCE_CRYSTAL;
    config.f_xtal = 0;

    config.synthesiser.vco_type = AX_VCO_INTERNAL;
    config.synthesiser.A.frequency = 815200000;
    config.synthesiser.B.frequency = 815200000;
    
    config.load_capacitance = 0;
    config.tcxo_enable = config.tcxo_disable = NULL;
    config.transmit_path = AX_TRANSMIT_PATH_SE;
    config.transmit_power_limit = 0.0f;
    config.spi_transfer = masterhaul_spi_transfer;

    config.pkt_store_flags = AX_PKT_STORE_RSSI | AX_PKT_STORE_RF_OFFSET;
    config.pkt_accept_flags = 0;
    config.wakeup_period_ms = 0;
    config.wakeup_xo_early_ms = 0;

    /* ------- init ------- */
    int initStatus = ax_init(&config);
    ax_default_params(&config, &gmsk_hdlc_fec_modulation);
    
#ifdef AX_RECEIVER    
    ax_packet rx_pkt;
    ax_rx_on(&config, &gmsk_hdlc_fec_modulation);
#else
    uint8_t tx_pkt[0x100];
    ax_tx_on(&config, &gmsk_hdlc_fec_modulation);
#endif
    
    Print_EnqueueMsg("Hello Arduino from the PIC32. RF init status %d\n", initStatus);

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
        
#ifdef AX_RECEIVER
        if (ax_rx_packet(&config, &rx_pkt)) {
            Print_EnqueueMsg("rx!\n");
        }
#else
        strcpy((char*) tx_pkt, "ughdffgiuhdfudshfdjshfdjshfsudhfdskjfdfd");
        ax_tx_packet(&config, &gmsk_hdlc_fec_modulation, tx_pkt, 40);
#endif        
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