/* 
 * File:   UART_Comon.h
 * Author: Kodiak North
 * 
 * Description: Defines common data structures used by modules that interface
 * with UART peripherals.
 *
 * Created on September 10, 2021, 2:26 PM
 */

#ifndef UART_COMON_H
#define	UART_COMON_H

// **** GLOBAL INCLUDE DIRECTIVES ****
#include <stdbool.h>
#include <stdint.h>

#include "config/default/peripheral/uart/plib_uart_common.h"
// **** END GLOBAL INCLUDE DIRECTIVES ****
// -----------------------------------------------------------------------------
// **** GLOBAL MACROS ****

// **** END GLOBAL MACROS ****

#ifdef	__cplusplus
extern "C" {
#endif

    // **** GLOBAL TYPEDEFS ****
    typedef struct {
        volatile bool IsRxErrorDetected;
        volatile bool IsTxFinished;
        volatile bool IsRxFinished;
        volatile UART_ERROR Errors;
    } uart_interrupt_flags_t;
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

#endif	/* UART_COMON_H */

