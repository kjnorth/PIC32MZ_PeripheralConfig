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
#if defined(PL_MAIN_BOARD)

#define NUM_APPLICATION_ENCODERS (6u)

#elif defined(BR_MAIN_BOARD)

#define NUM_APPLICATION_ENCODERS (2u)

#else
#define NUM_APPLICATION_ENCODERS (0u)
#endif
// **** END MODULE DEFINES ****
// -----------------------------------------------------------------------------
// **** MODULE TYPEDEFS ****
typedef enum {
    A0B0, A0B1, A1B1, A1B0,
} encoder_state_t;

typedef struct {
    encoder_id_t id;
    GPIO_PIN aPin;
    GPIO_PIN bPin;
    encoder_state_t curState;
} encoder_descriptor_t;
// **** END MODULE TYPEDEFS ****
// -----------------------------------------------------------------------------
// **** MODULE GLOBAL VARIABLES ****
#if defined(PL_MAIN_BOARD)

encoder_descriptor_t G_EncoderDesc[NUM_APPLICATION_ENCODERS] = {
    {ENC1, ENC1_A_TEST_PIN, ENC1_B_TEST_PIN, A0B0},
    {ENC2, ENC2_A_TEST_PIN, ENC2_B_TEST_PIN, A0B0},
    {ENC3, 0, 3, A0B0},
    {ENC4, 0, 4, A0B0},
    {ENC5, 0, 5, A0B0},
    {ENC6, 0, 6, A0B0}
};

#elif defined(BR_MAIN_BOARD)

encoder_descriptor_t G_EncoderDesc[NUM_APPLICATION_ENCODERS] = {
    {ENC9, ENC1_A_TEST_PIN, ENC1_B_TEST_PIN, A0B0},
    {ENC10, ENC2_A_TEST_PIN, ENC2_B_TEST_PIN, A0B0}
};

#else
encoder_descriptor_t G_EncoderDesc[NUM_APPLICATION_ENCODERS];
#endif

int32_t G_EncoderCounts[TOTAL_ENCODERS];
// **** END MODULE GLOBAL VARIABLES ****
// -----------------------------------------------------------------------------
// **** MODULE FUNCTION PROTOTYPES ****
void Encoders_InterruptHandler(GPIO_PIN pin, uintptr_t context);
// **** MODULE FUNCTION PROTOTYPES ****
// -----------------------------------------------------------------------------

void Encoders_Init(void) {
    uint8_t i;
    for (i = 0; i < NUM_APPLICATION_ENCODERS; i++) {
        encoder_descriptor_t* enc = &G_EncoderDesc[i];
        
        // determine the encoder's current state
        bool A = GPIO_PinRead(enc->aPin);
        bool B = GPIO_PinRead(enc->bPin);
        if (A == 0) {
            enc->curState = (B == 0) ? A0B0 : A0B1;
        } else {
            enc->curState = (B == 0) ? A1B0 : A1B1;
        }
        
        // TODO: Pull previous encoder data from NVM
//        G_EncoderCounts[enc->id] = ......
        
        GPIO_PinInterruptCallbackRegister(enc->aPin, Encoders_InterruptHandler, (uintptr_t) enc);
        GPIO_PinInterruptCallbackRegister(enc->bPin, Encoders_InterruptHandler, (uintptr_t) enc);
        GPIO_PinInterruptEnable(enc->aPin);
        GPIO_PinInterruptEnable(enc->bPin);
    }
}

void Encoders_SetCount(encoder_id_t id, int32_t count) {
    G_EncoderCounts[id] = count;
}

int32_t Encoders_GetCount(encoder_id_t id) {
    return G_EncoderCounts[id];
}

/**
 * E2 encoders - B leads A for clockwise shaft rotation, and A leads B for
 * counterclockwise rotation viewed from the cover/label side of the encoder
 */
void Encoders_InterruptHandler(GPIO_PIN pin, uintptr_t context) {
    encoder_descriptor_t* enc = (encoder_descriptor_t*) context;
    uint8_t A = GPIO_PinRead(enc->aPin);
    uint8_t B = GPIO_PinRead(enc->bPin);

    encoder_state_t curState = enc->curState;
    int32_t count = G_EncoderCounts[enc->id];
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
    G_EncoderCounts[enc->id] = count;
}