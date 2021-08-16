/* 
 * File:   SoftPWM.h
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
 * Created on August 6, 2021, 3:08 PM
 */

#ifndef SOFTPWM_H
#define	SOFTPWM_H

// **** GLOBAL INCLUDE DIRECTIVES ****
#include <stdbool.h>
#include <stdint.h>

#include "peripheral/gpio/plib_gpio.h"
// **** END GLOBAL INCLUDE DIRECTIVES ****
// -----------------------------------------------------------------------------
// **** GLOBAL MACROS ****
#define SOFT_PWM_DEFAULT_FREQ_HZ (5000u)
// **** END GLOBAL MACROS ****

#ifdef	__cplusplus
extern "C" {
#endif

    // **** GLOBAL TYPEDEFS ****

    typedef enum {
        SOFT_PWM_PIN_NORMAL,
        SOFT_PWM_PIN_INVERTED,
    } soft_pwm_type_t;
    // **** END GLOBAL TYPEDEFS ****
    // -------------------------------------------------------------------------
    // **** GLOBAL VARIABLES ****

    // **** END GLOBAL VARIABLES ****
    // -------------------------------------------------------------------------
    // **** GLOBAL FUNCTION PROTOTYPES ****
    void SoftPWM_Init(void);
    bool SoftPWM_PinAdd(GPIO_PIN pin, soft_pwm_type_t type);
    void SoftPWM_PinEnable(GPIO_PIN pin);
    void SoftPWM_PinDisable(GPIO_PIN pin);
    void SoftPWM_PinEnableAll(void);
    void SoftPWM_PinDisableAll(void);
    void SoftPWM_PinSetDuty(GPIO_PIN pin, uint8_t duty);
    bool SoftPWM_SetFrequency(uint16_t freqHz);
    uint16_t SoftPWM_GetFrequency(void);
    // **** END GLOBAL FUNCTION PROTOTYPES ****
    // -------------------------------------------------------------------------
#ifdef	__cplusplus
}
#endif

#endif	/* SOFTPWM_H */

