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
#define MICRO_NUM_PINS (152u)
#define SOFT_PWM_MAX_FREQ_HZ (10000u)
#define SOFT_PWM_MAX_PINS (16u)
// **** END MODULE MACROS ****
// -----------------------------------------------------------------------------
// **** MODULE TYPEDEFS ****
typedef struct {
    GPIO_PIN pin;
    uint8_t duty;
    bool isEnabled;
    bool isInverted;
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

// **** MODULE FUNCTION PROTOTYPES ****
// -----------------------------------------------------------------------------

void SoftPWM_Init(void) {
    TMR4_Start();
    // NEED TO ATTACH A CALLBACK
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
        PinsAdded[PinIndexLUT[pin]].isInverted = false;
        TotalPinsAdded++;
    }
    return returnVal;
}

void SoftPWM_PinEnable(GPIO_PIN pin) {
    PinsAdded[PinIndexLUT[pin]].isEnabled = true;
}

void SoftPWM_PinDisable(GPIO_PIN pin) {
    PinsAdded[PinIndexLUT[pin]].isEnabled = false;
}

// TODO: TEST
void SoftPWM_PinInvert(GPIO_PIN pin) {
    bool *status = &PinsAdded[PinIndexLUT[pin]].isInverted;
    *status = !*status;
}

void SoftPWM_PinSetDuty(GPIO_PIN pin, uint8_t duty);
bool SoftPWM_SetFrequency(uint16_t freqHz);
uint16_t SoftPWM_GetFrequency(void);
