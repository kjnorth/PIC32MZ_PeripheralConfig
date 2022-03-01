/* 
 * File:   AX_Config.h
 * Author: Kodiak North
 * 
 * Description: defines configuration data for AX5043/5243 transceivers
 *
 * Created on November 18, 2021, 2:06 PM
 */

#ifndef AX_CONFIG_H
#define	AX_CONFIG_H

// **** GLOBAL INCLUDE DIRECTIVES ****
#include <stdbool.h>
#include <stdint.h>
// **** END GLOBAL INCLUDE DIRECTIVES ****
// -----------------------------------------------------------------------------
// **** GLOBAL MACROS ****
// chip properties, DO NOT CHANGE
#define AX_FIFO_MAX_SIZE (255u) // size of the chip's FIFO, 0-indexed

// properties configurable by the developer
#define AX_PTX_PKTADDR (0xBBEE3432u)
#define AX_PTX_ACK_TIMEOUT_MS (100u)

#define AX_PRX_PKTADDR (0xDDCC3433u)

#define AX_MAX_ENQUEUED_PACKETS (5u)
#define AX_PAYLOAD_MAX_SIZE (200u) // payload + packet metadata must be <= AX_FIFO_MAX_SIZE

// settings to switch between crystal or TCXO
#define PINFUNCPWRAMP_VAL (0x07u) // 0x~7 to enable TCXO, 0x~0 for internal oscillator use
#define XTALCAP_VAL (0x00u) // 0x00 for TCXO, some other value for crystal
#define PLLVCODIV_VAL (0x21u) // set to 0x20 if Fxtal is less than 24.8MHz, or 0x21 otherwise
#define F10_VAL (0x0Du) // set to 0x04 for TCXO, 0x0D if frequency of TCXO or internal osc is > 43MHz, or 0x03 otherwise
#define F11_VAL (0x00u) // set to 0x00 for TCXO, 0x07 if crystal is used
#define F35_VAL (0x11u) // set to 0x10 if Fxtal is less than 24.8MHz, or 0x11 otherwise
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

    // **** END GLOBAL FUNCTION PROTOTYPES ****
    // -------------------------------------------------------------------------
#ifdef	__cplusplus
}
#endif

#endif	/* AX_CONFIG_H */

