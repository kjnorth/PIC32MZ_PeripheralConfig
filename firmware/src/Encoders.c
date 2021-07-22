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

// **** MODULE INCLUDE DIRECTIVES ****
#include "Encoders.h"

#include "peripheral/gpio/plib_gpio.h"
// **** END MODULE INCLUDE DIRECTIVES ****
// -----------------------------------------------------------------------------
// **** MODULE DEFINES ****

// **** END MODULE DEFINES ****
// -----------------------------------------------------------------------------
// **** MODULE TYPEDEFS ****
typedef uint8_t(*encoder_phase_getter_t)(void);

typedef enum {
    A0B0, A0B1, A1B1, A1B0,
} encoder_state_t;

typedef struct {
    encoder_phase_getter_t aGet; // is there a better way to do this other than creating a function for ever single encoder phase? would like to use the generated
                                // macros but I don't think that is possible...
    encoder_phase_getter_t bGet;
    encoder_state_t curState;
    int32_t count;
} encoder_t;
// **** END MODULE TYPEDEFS ****
// -----------------------------------------------------------------------------
// **** MODULE GLOBAL VARIABLES ****
static encoder_t enc1;
//static encoder_t enc2;
// **** END MODULE GLOBAL VARIABLES ****
// -----------------------------------------------------------------------------
// **** MODULE FUNCTION PROTOTYPES ****
static void EncCallback(GPIO_PIN pin, uintptr_t context);
static uint8_t Enc1AGet(void);
static uint8_t Enc1BGet(void);
// **** MODULE FUNCTION PROTOTYPES ****
// -----------------------------------------------------------------------------

void Encoders_Init(void) {
    enc1.aGet = Enc1AGet;
    enc1.bGet = Enc1BGet;
    enc1.curState = A0B0;
    enc1.count = 0;
    GPIO_PinInterruptCallbackRegister(ENC_A_TEST_PIN, EncCallback, (uintptr_t) & enc1);
    GPIO_PinInterruptCallbackRegister(ENC_B_TEST_PIN, EncCallback, (uintptr_t) & enc1);
    //    GPIO_PinInterruptCallbackRegister(ENC2_A_TEST_PIN, EncCallback, (uintptr_t) &enc2);
    //    GPIO_PinInterruptCallbackRegister(ENC2_B_TEST_PIN, EncCallback, (uintptr_t) &enc2);
    GPIO_PinInterruptEnable(ENC_A_TEST_PIN);
    GPIO_PinInterruptEnable(ENC_B_TEST_PIN);
}

int32_t Encoders_GetCount(system_encoder_t enc) {
    return enc1.count; // TODO: update with better implementation
}

/**
 * E2 encoders - B leads A for clockwise shaft rotation, and A leads B for
 * counterclockwise rotation viewed from the cover/label side of the encoder
 */
void EncCallback(GPIO_PIN pin, uintptr_t context) {
    encoder_t* enc = (encoder_t*) context;
    uint8_t A = enc->aGet();
    uint8_t B = enc->bGet();

    encoder_state_t curState = enc->curState;
    int32_t count = enc->count;
    switch (curState) {
        case A0B0:
            if ((A == 0) && (B == 1)) {
                // CW spin
                count++;
                curState = A0B1;
            } else if ((A == 1) && (B == 0)) {
                // CCW spin
                count--;
                curState = A1B0;
            }
            break;
        case A0B1:
            if ((A == 1) && (B == 1)) {
                // CW spin
                count++;
                curState = A1B1;
            } else if ((A == 0) && (B == 0)) {
                // CCW spin
                count--;
                curState = A0B0;
            }
            break;
        case A1B1:
            if ((A == 1) && (B == 0)) {
                // CW spin
                count++;
                curState = A1B0;
            } else if ((A == 0) && (B == 1)) {
                // CCW spin
                count--;
                curState = A0B1;
            }
            break;
        case A1B0:
            if ((A == 0) && (B == 0)) {
                // CW spin
                count++;
                curState = A0B0;
            } else if ((A == 1) && (B == 1)) {
                // CCW spin
                count--;
                curState = A1B1;
            }
            break;
    }
    enc->curState = curState;
    enc->count = count;
}

uint8_t Enc1AGet(void) {
    return ENC_A_TEST_Get();
}

uint8_t Enc1BGet(void) {
    return ENC_B_TEST_Get();
}