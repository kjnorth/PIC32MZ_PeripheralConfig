/* 
 * File:   Encoders.c
 * Author: Kodiak North
 * 
 * Description: The Encoders module provides functions for sampling, tracking,
 * and getting the system's encoders. Each encoder is configured to trigger a
 * Change Notice interrupt on its A and B phases. A callback is attached to
 * each pair of A and B phases (i.e. callback1 attached to A1 and B1, callback2
 * attached to A2 and B2, etc.). Each callback will increment/decrement the
 * count for its encoder based on the current state of the encoder's A and B
 * phases.
 * 
 * Usage: For each encoder in the system, add a value to the SYSTEM_ENCODER
 * before the TOTAL_ENCODERS item. In Encoders.c, create a callback for each
 * encoder and populate the descriptor list with all necessary items. UPDATE IN .H/.C
 *
 * Created on July 21, 2021, 1:30 PM
 */

#include "Encoders.h"

#include "peripheral/gpio/plib_gpio.h"

// **** MODULE ENUMS ****

typedef enum {
    A0B0, A0B1, A1B1, A1B0,
} encoder_state_t;
static encoder_state_t curState = A0B0;
// **** END MODULE ENUMS ****

// **** MODULE GLOBAL VARIABLES ****
static int32_t Enc1Count = 0;
// **** END MODULE GLOBAL VARIABLES ****

// **** MODULE FUNCTION PROTOTYPES ****
static void Enc1Callback(GPIO_PIN pin, uintptr_t context);
// **** END MODULE FUNCTION PROTOTYPES ****

void Encoders_Init(void) {
    GPIO_PinInterruptCallbackRegister(ENC_A_TEST_PIN, Enc1Callback, (uintptr_t) NULL);
    GPIO_PinInterruptCallbackRegister(ENC_B_TEST_PIN, Enc1Callback, (uintptr_t) NULL);
    GPIO_PinInterruptEnable(ENC_A_TEST_PIN);
    GPIO_PinInterruptEnable(ENC_B_TEST_PIN);
}

int32_t Encoders_GetCount(system_encoder_t enc) {
    return Enc1Count;
}

/**
 * E2 encoders - B leads A for clockwise shaft rotation, and A leads B for
 * counterclockwise rotation viewed from the cover/label side of the encoder
 */
void Enc1Callback(GPIO_PIN pin, uintptr_t context) {
    uint8_t A = ENC_A_TEST_Get();
    uint8_t B = ENC_B_TEST_Get();

    switch (curState) {
        case A0B0:
            if ((A == 0) && (B == 1)) {
                // CW spin
                Enc1Count++;
                curState = A0B1;
            } else if ((A == 1) && (B == 0)) {
                // CCW spin
                Enc1Count--;
                curState = A1B0;
            }
            break;
        case A0B1:
            if ((A == 1) && (B == 1)) {
                // CW spin
                Enc1Count++;
                curState = A1B1;
            } else if ((A == 0) && (B == 0)) {
                // CCW spin
                Enc1Count--;
                curState = A0B0;
            }
            break;
        case A1B1:
            if ((A == 1) && (B == 0)) {
                // CW spin
                Enc1Count++;
                curState = A1B0;
            } else if ((A == 0) && (B == 1)) {
                // CCW spin
                Enc1Count--;
                curState = A0B1;
            }
            break;
        case A1B0:
            if ((A == 0) && (B == 0)) {
                // CW spin
                Enc1Count++;
                curState = A0B0;
            } else if ((A == 1) && (B == 1)) {
                // CCW spin
                Enc1Count--;
                curState = A1B1;
            }
            break;
    }
}
