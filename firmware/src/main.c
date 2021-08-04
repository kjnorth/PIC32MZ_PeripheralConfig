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

#define PWM_MAX_DUTY_CYCLE      (100u)

#define NVM_PRESERVE_MEMORY_START_ADDR (0x9D00C000u)

uint16_t g_adc_count = 0;
volatile uint8_t dutyCycle = 0;

void ADCHS_Callback(ADCHS_CHANNEL_NUM channel, uintptr_t context);
uint16_t PWM_CompareValueGet(uint8_t dutyCycle);
void SPIComm(uint8_t sendData);

static volatile bool transferDone = false;
#define NVM
#ifdef NVM
static void eventHandler(uintptr_t context);
static void NVM_Test(void);
#endif

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

#ifdef NVM
    NVM_CallbackRegister(eventHandler, (uintptr_t) NULL);
    NVM_Test();
#endif

    Print_EnqueueMsg("Hello Arduino from the new print module version %0.2f\n", SW_VERSION);

    while (1) {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks();

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

#ifdef NVM

void eventHandler(uintptr_t context) {
    transferDone = true;
}

#define NVM_READ_WRITE_SIZE (NVM_FLASH_PAGESIZE)
#define BUFFER_SIZE (NVM_READ_WRITE_SIZE / sizeof (uint32_t))

void NVM_Test(void) {
    uint32_t address = NVM_PRESERVE_MEMORY_START_ADDR;
    static uint32_t writeData[BUFFER_SIZE] CACHE_ALIGN;
    static uint32_t readData[BUFFER_SIZE];
    uint32_t i;

    writeData[0] = 12835;
    writeData[1] = 43110;
//    writeData[BUFFER_SIZE-1] = 3210;
//    writeData[BUFFER_SIZE] = 699001;

    while (NVM_IsBusy() == true);
    LED1_Set();

    NVM_PageErase(NVM_PRESERVE_MEMORY_START_ADDR);
    while (transferDone == false);
    transferDone = false;

    for (i = 0; i < NVM_READ_WRITE_SIZE; i += NVM_FLASH_ROWSIZE) {
        uint32_t ptrInc = 0;
        // program a row of data
        NVM_RowWrite(writeData + ptrInc, address);
        // wait for program to finish
        while (transferDone == false);
        transferDone = false;

        ptrInc += NVM_FLASH_ROWSIZE;
        address += NVM_FLASH_ROWSIZE;
    }
    LED1_Clear();

    NVM_Read(readData, NVM_READ_WRITE_SIZE, NVM_PRESERVE_MEMORY_START_ADDR);
    if (!memcmp(writeData, readData, NVM_READ_WRITE_SIZE)) {
        LED1_Set();
    } else {
        Print_EnqueueMsg("data read does not match data written.. sizeof writeData %u\n", sizeof(writeData));
    }
    Print_EnqueueMsg("final row addr 0x%X, rd[0] %lu, rd[1] %lu, rd[buf size-1] %lu, rd[buf size] %lu\n",
            address, readData[0], readData[1], readData[BUFFER_SIZE-1], readData[BUFFER_SIZE]);
}
#endif