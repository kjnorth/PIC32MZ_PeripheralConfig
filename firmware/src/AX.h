/* 
 * File:   MH_AX.h
 * Author: Kodiak North
 * 
 * Description: Masterhaul's implementation of a library to interface with
 * OnSemiconductor's cumbersome AX-5043/5243 transceivers.
 * 
 * Module blocks when reading and writing via SPI, therefore SPI peripheral
 * configured with MHC should not be in interrupt mode.
 * 
 * NOTE: these AX chips are configured to transmit LSB of data first
 *
 * Created on October 28, 2021, 1:07 PM
 */

#ifndef AX_H
#define	AX_H

// **** GLOBAL INCLUDE DIRECTIVES ****
#include <stdbool.h>
#include <stdint.h>

#include "AX_Config.h"
// **** END GLOBAL INCLUDE DIRECTIVES ****
// -----------------------------------------------------------------------------
// **** GLOBAL MACROS ****
#define PTX_PRX_TEST_PACKET_SIZE (100u) //12u) // NOTE: the size includes 3 byte FIFO DATA command that is also transmitted
// **** END GLOBAL MACROS ****

#ifdef	__cplusplus
extern "C" {
#endif

    // **** GLOBAL TYPEDEFS ****
    typedef enum {
        AX_MODE_NONE = 0,
        AX_MODE_PTX, // primary transmitter
        AX_MODE_PRX, // primary receiver
    } ax_mode_t;
    // **** END GLOBAL TYPEDEFS ****
    // -------------------------------------------------------------------------
    // **** GLOBAL VARIABLES ****

    // **** END GLOBAL VARIABLES ****
    // -------------------------------------------------------------------------
    // **** GLOBAL FUNCTION PROTOTYPES ****
    bool AX_Init(ax_mode_t _mode);
    
    // interrupt driven
    void AX_CommTask(void);
    
    uint16_t GetState(void); // TODO: Delete
    void SetDebugLEDs(uint8_t val); // TODO: Delete
    
    // polling / blocking functions
    int AX_TransmitPacket(uint8_t* txPacket, uint8_t length);
    void AX_Receive(void);
    
    extern bool AX_EnqueuePacket(uint8_t* txPacket, uint8_t length);
    
    // **** END GLOBAL FUNCTION PROTOTYPES ****
    // -------------------------------------------------------------------------
#ifdef	__cplusplus
}
#endif

#endif	/* AX_H */