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

typedef enum {
    INIT,
    ENABLED,
    DISABLE_NOW,
    DISABLED,
} soft_pwm_state_t;

typedef enum {
    PIN_DISABLED,
    ACTIVE,
    INACTIVE,
} soft_pwm_pin_enabled_state_t;

typedef struct {
    GPIO_PIN pin;
    soft_pwm_type_t type;
    uint8_t duty;
    uint8_t ticks;
    soft_pwm_state_t state;
    soft_pwm_pin_enabled_state_t enabledState;
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
void SoftPWM_TMR_Callback(uint32_t status, uintptr_t context);
void SoftPWM_StateMachine(soft_pwm_pin_t* curPin);
void SoftPWM_StateMachineHelper(soft_pwm_pin_t* curPin);
// **** MODULE FUNCTION PROTOTYPES ****
// -----------------------------------------------------------------------------

void SoftPWM_Init(void) {
    TMR4_CallbackRegister(SoftPWM_TMR_Callback, (uintptr_t) NULL);
    TMR4_Start();
}

/**
 * 
 * @param pin
 * @return false if total pins are already added
 */
bool SoftPWM_PinAdd(GPIO_PIN pin, soft_pwm_type_t type) {
    bool returnVal = false;
    if (TotalPinsAdded <= SOFT_PWM_MAX_PINS) {
        returnVal = true;
        PinIndexLUT[pin] = TotalPinsAdded;
        soft_pwm_pin_t *curPin = &PinsAdded[PinIndexLUT[pin]];
        curPin->pin = pin;
        curPin->type = type;
        curPin->duty = 0;
        curPin->ticks = 0;
        curPin->state = DISABLED;
        curPin->enabledState = PIN_DISABLED;
        TotalPinsAdded++;
    }
    return returnVal;
}

void SoftPWM_PinEnable(GPIO_PIN pin) {
    soft_pwm_pin_t *curPin = &PinsAdded[PinIndexLUT[pin]];
    GPIO_PinSet(curPin->pin);
    curPin->state = INIT;
}

void SoftPWM_PinDisable(GPIO_PIN pin) {
    soft_pwm_pin_t *curPin = &PinsAdded[PinIndexLUT[pin]];
    GPIO_PinClear(curPin->pin);
    curPin->state = DISABLE_NOW;
}

void SoftPWM_PinEnableAll(void) {
    uint8_t i;
    for (i = 0; i < TotalPinsAdded; i++) {
        PinsAdded[i].state = INIT;
    }
}

void SoftPWM_PinDisableAll(void) {
    uint8_t i;
    for (i = 0; i < TotalPinsAdded; i++) {
        PinsAdded[i].state = DISABLE_NOW;
    }
}

void SoftPWM_PinSetDuty(GPIO_PIN pin, uint8_t duty) {
    if (duty <= SOFT_PWM_MAX_DUTY) {
        soft_pwm_pin_t *curPin = &PinsAdded[PinIndexLUT[pin]];
        curPin->duty = duty;
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

uint32_t SoftPWM_GetFrequency(void) {
    //    uint32_t tmrFreq = TMR4_FrequencyGet();
    //    return (uint32_t) (tmrFreq / 100); // timer frequency is 100x faster than PWM frequency
    return TMR4_PeriodGet();
}

void SoftPWM_TMR_Callback(uint32_t status, uintptr_t context) {
    uint8_t i;
    for (i = 0; i < TotalPinsAdded; i++) {
        soft_pwm_pin_t* curPin = &PinsAdded[i];
        SoftPWM_StateMachine(curPin);
    }
}

void SoftPWM_StateMachine(soft_pwm_pin_t* curPin) {
    switch (curPin->state) {
        case INIT:
            // set pin to active state based on type
            if (curPin->type == SOFT_PWM_PIN_NORMAL) {
                GPIO_PinSet(curPin->pin);
            } else if (curPin->type == SOFT_PWM_PIN_INVERTED) {
                GPIO_PinClear(curPin->pin);
            }
            curPin->state = ENABLED;
            curPin->enabledState = ACTIVE;
            break;
        case ENABLED:
            SoftPWM_StateMachineHelper(curPin);
            break;
        case DISABLE_NOW:
            // return pin to inactive state based on type
            if (curPin->type == SOFT_PWM_PIN_NORMAL) {
                GPIO_PinClear(curPin->pin);
            } else if (curPin->type == SOFT_PWM_PIN_INVERTED) {
                GPIO_PinSet(curPin->pin);
            }
            curPin->state = DISABLED;
            curPin->enabledState = PIN_DISABLED;
            curPin->ticks = 0;
            break;
        case DISABLED:
            break;
    }
}

void SoftPWM_StateMachineHelper(soft_pwm_pin_t* curPin) {
    curPin->ticks++;
    switch (curPin->enabledState) {
        case PIN_DISABLED:
            break;
        case ACTIVE:
            if (curPin->ticks == curPin->duty) {
                GPIO_PinToggle(curPin->pin);
                curPin->enabledState = INACTIVE;
            }
            break;
        case INACTIVE:
            if (curPin->ticks == SOFT_PWM_PERIOD_COMPLETE_TICKS) {
                GPIO_PinToggle(curPin->pin);
                curPin->enabledState = ACTIVE;
                curPin->ticks = 0;
            }
            break;
    }
}