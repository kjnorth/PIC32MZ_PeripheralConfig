/* 
 * File:   Time.c
 * Author: Kodiak North
 * 
 * Description: The Time module is used to track time in milliseconds for use
 * throughout the program. It requires that the CORE TIMER module be configured
 * to generate a periodic interrupt every 1 millisecond (i.e. at 1 kHz).
 *
 * Created on July 19, 2021, 12:00 PM
 */

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "Time.h"

#include "peripheral/coretimer/plib_coretimer.h"

// **** MODULE GLOBAL VARIABLES ****
static volatile uint32_t MsTicks = 0;
// **** END MODULE GLOBAL VARIABLES ****

// **** MODULE FUNCTION PROTOTYPES ****
static void CORETIMER_InterruptCallback(uint32_t status, uintptr_t context);
// **** END MODULE FUNCTION PROTOTYPES ****

void Time_Init(void) {
    static bool IsInitialized = false;
    if (!IsInitialized) {
        IsInitialized = true;
        MsTicks = 0;
        CORETIMER_CallbackSet(CORETIMER_InterruptCallback, (uintptr_t) NULL);
        CORETIMER_Start();       
    }
}

uint32_t Time_GetMs(void) {
    return MsTicks;
}

static void CORETIMER_InterruptCallback(uint32_t status, uintptr_t context) {
    MsTicks++;
}
