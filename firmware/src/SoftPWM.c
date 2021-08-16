/* 
 * File:   SoftPWM.c
 * Author: Kodiak North
 * 
 * Description: Module utilizes the TMR4 peripheral to generate PWM with GPIO
 * pins.
 * 
 * PWM frequency is set by using the PBCLK3_FREQ_HZ and TMR4_PRESCALER macros.
 * If the developer decides to change the PBCLK3 frequency or timer4's
 * prescaler, these macros must be updated to the appropriate values.
 * 
 * Due to hardware constraints, maximum PWM frequency is limited to 5kHz.
 * Default frequency is 1kHz. Maximum number of pins is 16 but this can be
 * changed if desired. MICRO_NUM_PINS should be set with the number of
 * available pins on the chosen microcontroller.
 * 
 * When enabling a pin for the first time, its duty cycle should be set first:
 *      SoftPWM_PinSetDuty(<pin>, <duty>);
 *      SoftPWM_PinEnable(<pin>);
 * 
 * When a pin's duty cycle is changed, the wave will complete its current cycle
 * before the changes take affect.
 *
 * Created on August 6, 2021, 3:26 PM
 */

// **** MODULE INCLUDE DIRECTIVES ****
#include "SoftPWM.h"

#include "peripheral/tmr/plib_tmr4.h"
#include "Print.h"
// **** END MODULE INCLUDE DIRECTIVES ****
// -----------------------------------------------------------------------------
// **** MODULE MACROS ****
#define PBCLK3_FREQ_HZ (100000000ul)
#define TMR4_PRESCALER (1u)
#define TMR_FREQ_MULTIPLIER (100u) // TMR peripheral runs 100x faster than the frequency of the PWM to achieve 1% duty resolution
#define MICRO_NUM_PINS (145u)

#define SOFT_PWM_MAX_FREQ_HZ (5000u) // for 1:1 prescaler
#define SOFT_PWM_MIN_FREQ_HZ (16u) // for 1:1 prescaler
#define SOFT_PWM_MAX_PINS (16u)
#define SOFT_PWM_MAX_DUTY (100u)
#define SOFT_PWM_PERIOD_COMPLETE_TICKS (99u)
// **** END MODULE MACROS ****
// -----------------------------------------------------------------------------
// **** MODULE TYPEDEFS ****

typedef enum {
    INIT,
    ENABLED,
    DISABLE_NOW,
    DISABLED,
} soft_pwm_state_t;

typedef void (*gpio_pin_set_clear_func_t)(GPIO_PIN pin);

typedef struct {
    GPIO_PIN pin;
    soft_pwm_type_t type;
    gpio_pin_set_clear_func_t Activate;
    gpio_pin_set_clear_func_t Deactivate;
    volatile uint8_t curDuty;
    volatile uint8_t newDuty;
    volatile soft_pwm_state_t state;
} soft_pwm_pin_t;
// **** END MODULE TYPEDEFS ****
// -----------------------------------------------------------------------------
// **** MODULE GLOBAL VARIABLES ****
static uint8_t PinIndexLUT[MICRO_NUM_PINS] = {0}; // stores index of the added pin in PinsAdded array
static soft_pwm_pin_t PinsAdded[SOFT_PWM_MAX_PINS];
static uint8_t TotalPinsAdded = 0;
static int8_t TmrTicks = -1; // init to -1, count from 0 to 99
// **** END MODULE GLOBAL VARIABLES ****
// -----------------------------------------------------------------------------
// **** MODULE FUNCTION PROTOTYPES ****
void SoftPWM_TMR_Callback(uint32_t status, uintptr_t context);
void SoftPWM_StateMachine(soft_pwm_pin_t* curPin);
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
        switch (type) {
            case SOFT_PWM_PIN_NORMAL:
                curPin->Activate = GPIO_PinSet;
                curPin->Deactivate = GPIO_PinClear;
                break;
            case SOFT_PWM_PIN_INVERTED:
                curPin->Activate = GPIO_PinClear;
                curPin->Deactivate = GPIO_PinSet;
                break;
        }
        curPin->curDuty = 0;
        curPin->newDuty = 0;
        curPin->state = DISABLED;
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

/**
 * @note pin is disabled if the duty argument is out of range
 * @param pin - a pin that has been added to the module with SoftPWM_PinAdd
 * @param duty - range from 1 to SOFT_PWM_MAX_DUTY
 */
void SoftPWM_PinSetDuty(GPIO_PIN pin, uint8_t duty) {
    if (duty > 0 && duty <= SOFT_PWM_MAX_DUTY) {
        soft_pwm_pin_t *curPin = &PinsAdded[PinIndexLUT[pin]];
        curPin->newDuty = duty;
    } else {
        SoftPWM_PinDisable(pin);
    }
}

bool SoftPWM_SetFrequency(uint16_t freqHz) {
    bool returnVal = false;
    if (freqHz >= SOFT_PWM_MIN_FREQ_HZ && freqHz <= SOFT_PWM_MAX_FREQ_HZ) {
        TMR4_Stop();
        uint32_t tmrFreqHz = freqHz * TMR_FREQ_MULTIPLIER;
        uint16_t tmrPR = (PBCLK3_FREQ_HZ / (tmrFreqHz * TMR4_PRESCALER)) - 1;
        TMR4_PeriodSet(tmrPR);
        TMR4_Start();
        returnVal = true;
    }
    return returnVal;
}

uint16_t SoftPWM_GetFrequency(void) {
    float tmrPeriodSeconds = (float) ((TMR4_PeriodGet() + 1) / (float) PBCLK3_FREQ_HZ) * TMR_FREQ_MULTIPLIER;
    return (uint16_t) ((float) 1 / tmrPeriodSeconds);
}

void SoftPWM_TMR_Callback(uint32_t status, uintptr_t context) {
    uint8_t i;
    TmrTicks++;
    for (i = 0; i < TotalPinsAdded; i++) {
        soft_pwm_pin_t* curPin = &PinsAdded[i];
        SoftPWM_StateMachine(curPin);
    }
    if (TmrTicks == SOFT_PWM_PERIOD_COMPLETE_TICKS) {
        TmrTicks = -1;
    }
}

void SoftPWM_StateMachine(soft_pwm_pin_t* curPin) {
    switch (curPin->state) {
        case INIT:
            curPin->curDuty = curPin->newDuty;
            curPin->Activate(curPin->pin);
            curPin->state = ENABLED;
            break;
        case ENABLED:
            if (TmrTicks == curPin->curDuty) {
                curPin->Deactivate(curPin->pin);
            } else if (TmrTicks == SOFT_PWM_PERIOD_COMPLETE_TICKS) {
                curPin->state = INIT;
            }
            break;
        case DISABLE_NOW:
            curPin->Deactivate(curPin->pin);
            curPin->state = DISABLED;
            break;
        case DISABLED:
            break;
    }
}