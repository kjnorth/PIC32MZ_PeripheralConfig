/* 
 * File:   MH_AX_RegisterAPI.h
 * Author: Kodiak North
 * 
 * Description: Defines functions to interface directly with AX5043/5243 registers
 *
 * Created on November 15, 2021, 9:25 AM
 */

#ifndef AX_REGISTERAPI_H
#define	AX_REGISTERAPI_H

// **** GLOBAL INCLUDE DIRECTIVES ****
#include <stdbool.h>
#include <stdint.h>
// **** END GLOBAL INCLUDE DIRECTIVES ****
// -----------------------------------------------------------------------------
// **** GLOBAL MACROS ****

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
    uint16_t AX_GetStatus(void);
    
    void AX_Write8(uint16_t reg, uint8_t value);
    void AX_Write16(uint16_t reg, uint16_t value);
    void AX_Write24(uint16_t reg, uint32_t value);
    void AX_Write32(uint16_t reg, uint32_t value);

    uint8_t AX_Read8(uint16_t reg);
    uint16_t AX_Read16(uint16_t reg);
    uint32_t AX_Read24(uint16_t reg);
    uint32_t AX_Read32(uint16_t reg);
    
    void AX_WriteFifo(uint8_t* buffer, uint8_t length);
    
    bool AX_SPITest(uint16_t reg, uint8_t value);
    // **** END GLOBAL FUNCTION PROTOTYPES ****
    // -------------------------------------------------------------------------
#ifdef	__cplusplus
}
#endif

#endif	/* AX_REGISTERAPI_H */