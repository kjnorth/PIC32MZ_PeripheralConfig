/* 
 * File:   SoftPWM.c
 * Author: Kodiak North
 * 
 * Description:
 *
 * Created on August 6, 2021, 3:26 PM
 */

// **** MODULE INCLUDE DIRECTIVES ****
#include "SoftPWM.h"

#include "peripheral/tmr/plib_tmr4.h"
// **** END MODULE INCLUDE DIRECTIVES ****
// -----------------------------------------------------------------------------
// **** MODULE MACROS ****
#define PBCLK3_FREQ_HZ (100000000ul)
#define TMR4_PRESCALER (1u)
#define SOFT_PWM_MAX_FREQ_HZ (10000u) // for 1:1 prescaler
#define SOFT_PWM_MIN_FREQ_HZ (16u) // for 1:1 prescaler
#define SOFT_PWM_MAX_PINS (16u)
#define SOFT_PWM_MAX_DUTY (100u)
#define SOFT_PWM_PERIOD_COMPLETE_TICKS (100u)
#define MICRO_NUM_PINS (145u)
// **** END MODULE MACROS ****
// -----------------------------------------------------------------------------
// **** MODULE TYPEDEFS ****

typedef struct {
    GPIO_PIN pin;
    uint8_t duty;
    bool isEnabled;
    uint8_t ticks;
} soft_pwm_pin_t;
// **** END MODULE TYPEDEFS ****
// -----------------------------------------------------------------------------
// **** MODULE GLOBAL VARIABLES ****
uint8_t PinIndexLUT[MICRO_NUM_PINS] = {0}; // stores index of the added pin in PinsAdded array
soft_pwm_pin_t PinsAdded[SOFT_PWM_MAX_PINS];
uint8_t TotalPinsAdded = 0;
// **** END MODULE GLOBAL VARIABLES ****
// -----------------------------------------------------------------------------
// **** MODULE FUNCTION PROTOTYPES ****
void PWM_TMR_Callback(uint32_t status, uintptr_t context);
// **** MODULE FUNCTION PROTOTYPES ****
// -----------------------------------------------------------------------------

void SoftPWM_Init(void) {
    TMR4_CallbackRegister(PWM_TMR_Callback, (uintptr_t) NULL);
    TMR4_Start();
}

/**
 * 
 * @param pin
 * @return false if total pins are already added
 */
bool SoftPWM_PinAdd(GPIO_PIN pin) {
    bool returnVal = false;
    if (TotalPinsAdded <= SOFT_PWM_MAX_PINS) {
        returnVal = true;
        PinIndexLUT[pin] = TotalPinsAdded;
        PinsAdded[PinIndexLUT[pin]].pin = pin;
        PinsAdded[PinIndexLUT[pin]].duty = 0;
        PinsAdded[PinIndexLUT[pin]].isEnabled = false;
        PinsAdded[PinIndexLUT[pin]].ticks = 0;
        TotalPinsAdded++;
    }
    return returnVal;
}

void SoftPWM_PinEnable(GPIO_PIN pin) {
    PinsAdded[PinIndexLUT[pin]].isEnabled = true;
    GPIO_PinSet(PinsAdded[PinIndexLUT[pin]].pin);
}

void SoftPWM_PinDisable(GPIO_PIN pin) {
    PinsAdded[PinIndexLUT[pin]].isEnabled = false;
    GPIO_PinClear(PinsAdded[PinIndexLUT[pin]].pin);
}

void SoftPWM_PinDisableAll(void) {
    uint8_t i;
    for (i = 0; i < TotalPinsAdded; i++) {
        PinsAdded[i].isEnabled = false;
    }
}

void SoftPWM_PinSetDuty(GPIO_PIN pin, uint8_t duty) {
    if (duty <= SOFT_PWM_MAX_DUTY) {
        PinsAdded[PinIndexLUT[pin]].duty = duty;
    }
}

// TODO: TEST

bool SoftPWM_SetFrequency(uint16_t freqHz) {
    bool returnVal = false;
    if (freqHz >= SOFT_PWM_MIN_FREQ_HZ && freqHz <= SOFT_PWM_MAX_FREQ_HZ) {
        TMR4_Stop();
        uint16_t tmrFreqHz = freqHz * 100; // timer must count 100x faster than desired PWM to achieve 1% duty cycle resolution
        uint16_t tmrPR = PBCLK3_FREQ_HZ / (tmrFreqHz * TMR4_PRESCALER) - 1;
        TMR4_PeriodSet(tmrPR);
        TMR4_Start();
        returnVal = true;
    }
    return returnVal;
}

// TODO: TEST

uint16_t SoftPWM_GetFrequency(void) {
    uint32_t tmrFreq = TMR4_FrequencyGet();
    return (uint16_t) (tmrFreq / 100); // timer frequency is 100x faster than PWM frequency
}

void PWM_TMR_Callback(uint32_t status, uintptr_t context) {
    // what would be the purpose of sending the 'status' argument.. it always equals IFS0bits.T4IF
    uint8_t i;
    for (i = 0; i < TotalPinsAdded; i++) {
        soft_pwm_pin_t *curPin = &PinsAdded[PinIndexLUT[i]];
        if ((curPin->isEnabled) && (curPin->duty != SOFT_PWM_MAX_DUTY)) {
            curPin->ticks++;
            if (curPin->ticks == curPin->duty) {
                GPIO_PinClear(curPin->pin);
            } else if (curPin->ticks == SOFT_PWM_PERIOD_COMPLETE_TICKS) {
                GPIO_PinSet(curPin->pin);
                curPin->ticks = 0;
            }
        }
    }
}