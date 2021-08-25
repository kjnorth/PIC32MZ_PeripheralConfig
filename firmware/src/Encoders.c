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
 * Usage: For each encoder in the system, add a value to the system_encoder_t
 * enum before the TOTAL_ENCODERS item. In Encoders.c, define an encoder_t
 * struct for each encoder, and then populate the EncDescList with a pointer to
 * each struct along with the A and B pins configured for that encoder.
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

typedef enum {
    A0B0, A0B1, A1B1, A1B0,
} encoder_state_t;

typedef struct {
    GPIO_PIN aPin;
    GPIO_PIN bPin;
    encoder_state_t curState;
    int32_t count;
} encoder_t;

typedef struct {
    encoder_t* enc;
    GPIO_PIN aPinToSet;
    GPIO_PIN bPinToSet;
} encoder_desc_t;
// **** END MODULE TYPEDEFS ****
// -----------------------------------------------------------------------------
// **** MODULE GLOBAL VARIABLES ****
static encoder_t Enc1;
static encoder_t Enc2;
static encoder_desc_t EncDescList[TOTAL_ENCODERS] = {
    {(encoder_t*) NULL, (GPIO_PIN) NULL, (GPIO_PIN) NULL}, // system_encoder_t values start at 1, so populate 0 index of this list with NULL
    {&Enc1, ENC1_A_TEST_PIN, ENC1_B_TEST_PIN},
    {&Enc2, ENC2_A_TEST_PIN, ENC2_B_TEST_PIN}
};
// **** END MODULE GLOBAL VARIABLES ****
// -----------------------------------------------------------------------------
// **** MODULE FUNCTION PROTOTYPES ****
static void EncCallback(GPIO_PIN pin, uintptr_t context);
// **** MODULE FUNCTION PROTOTYPES ****
// -----------------------------------------------------------------------------

void Encoders_Init(void) {
    uint8_t i;
    for (i = ENC1; i < TOTAL_ENCODERS; i++) {
        encoder_t* enc = EncDescList[i].enc;
        enc->aPin = EncDescList[i].aPinToSet;
        enc->bPin = EncDescList[i].bPinToSet;
        enc->curState = A0B0;
        enc->count = 0;
        GPIO_PinInterruptCallbackRegister(enc->aPin, EncCallback, (uintptr_t) enc);
        GPIO_PinInterruptCallbackRegister(enc->bPin, EncCallback, (uintptr_t) enc);
        GPIO_PinInterruptEnable(enc->aPin);
        GPIO_PinInterruptEnable(enc->bPin);
    }
}

int32_t Encoders_GetCount(system_encoder_t enc) {
    int32_t returnVal = 0;
    if (enc < TOTAL_ENCODERS) {
        returnVal = EncDescList[enc].enc->count;
    }
    return returnVal;
}

/**
 * E2 encoders - B leads A for clockwise shaft rotation, and A leads B for
 * counterclockwise rotation viewed from the cover/label side of the encoder
 */
void EncCallback(GPIO_PIN pin, uintptr_t context) {
    encoder_t* enc = (encoder_t*) context;
    uint8_t A = GPIO_PinRead(enc->aPin);
    uint8_t B = GPIO_PinRead(enc->bPin);

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