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
#define AX_FIFO_MAX_SIZE (256u)
// **** END GLOBAL MACROS ****

#ifdef	__cplusplus
extern "C" {
#endif

    // **** GLOBAL TYPEDEFS ****
    typedef enum {
        AX_MODE_NONE = 0,
        AX_MODE_PTX,
        AX_MODE_PRX,
    } ax_mode_t;
    // **** END GLOBAL TYPEDEFS ****
    // -------------------------------------------------------------------------
    // **** GLOBAL VARIABLES ****

    // **** END GLOBAL VARIABLES ****
    // -------------------------------------------------------------------------
    // **** GLOBAL FUNCTION PROTOTYPES ****
    bool AX_Init(ax_mode_t _mode);
    void AX_InitConfigRegisters(void);
    void AX_InitTxRegisters(void);
    void AX_InitRxRegisters(void);
    
    uint16_t AX_GetStatus(void);
    void AX_PrintStatus(void);
    
    void AX_ReceivePacket(uint8_t* rxPacket);
    int AX_TransmitPacket(uint8_t* txPacket, uint8_t length);
    // **** END GLOBAL FUNCTION PROTOTYPES ****
    // -------------------------------------------------------------------------
#ifdef	__cplusplus
}
#endif

#endif	/* MH_AX_H */

