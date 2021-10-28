/* 
 * File:   MH_AX.h
 * Author: Kodiak North
 * 
 * Description: Masterhaul's implementation of a library to interface with
 * OnSemiconductor's cumbersome AX-5043/5243 transceivers.
 * 
 * This is currently designed to use the AX's internal oscillator and does not
 * support the use of an external oscillator at this time.
 * 
 * Module blocks when reading and writing via SPI, therefore SPI peripheral
 * configured with MHC should not be in interrupt mode.
 *
 * Created on October 28, 2021, 1:07 PM
 */

#ifndef MH_AX_H
#define	MH_AX_H

// **** GLOBAL INCLUDE DIRECTIVES ****
#include <stdbool.h>
#include <stdint.h>
// **** END GLOBAL INCLUDE DIRECTIVES ****
// -----------------------------------------------------------------------------
// **** GLOBAL MACROS ****
#define AX_TX_PACKET_MAX_SIZE (200u)
// **** END GLOBAL MACROS ****

#ifdef	__cplusplus
extern "C" {
#endif

    // **** GLOBAL TYPEDEFS ****

    // **** END GLOBAL TYPEDEFS ****
    // -------------------------------------------------------------------------
    // **** GLOBAL VARIABLES ****

    // **** END GLOBAL VARIABLES ****
    // -------------------------------------------------------------------------
    // **** GLOBAL FUNCTION PROTOTYPES ****
    void AX_InitTx(void);
    void AX_InitRx(void);
    
    int AX_TransmitPacket(uint8_t* txPacket, uint8_t length);
    void AX_ReceivePacket(uint8_t* rxPacket);
    
    uint16_t AX_GetStatus(void);
    void AX_PrintStatus(void);
    // **** END GLOBAL FUNCTION PROTOTYPES ****
    // -------------------------------------------------------------------------
#ifdef	__cplusplus
}
#endif

#endif	/* MH_AX_H */

