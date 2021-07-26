/* 
 * File:   Encoders.h
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
 * Created on July 21, 2021, 1:13 PM
 */

#ifndef NEWFILE_H
#define	NEWFILE_H

#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif

    typedef enum {
        ENC1,
        ENC2,
        TOTAL_ENCODERS, // only two encoders are wired up now. move down as more encoders are added
        ENC3,
        ENC4,
        ENC5,
        ENC6,
        ENC7,
        ENC8,
    } system_encoder_t;

    void Encoders_Init(void);
    int32_t Encoders_GetCount(system_encoder_t enc);

#ifdef	__cplusplus
}
#endif

#endif	/* NEWFILE_H */

