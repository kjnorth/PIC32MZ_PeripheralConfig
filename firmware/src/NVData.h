/* 
 * File:   NVData.h
 * Author: Kodiak North
 * 
 * Description: Module implements Non-Volatile Data storage using a portion of
 * the system's flash memory. The chosen programmer/debugger must be configured
 * to preserve a program memory range within the Project Properties menu.
 * A minimum of 1 page of flash must be preserved.
 * NVM_PRESERVE_MEMORY_START_ADDR is the virtual address that corresponds to
 * the physical start address that is preserved.
 *
 * Created on August 6, 2021, 11:22 AM
 */

#ifndef NVDATA_H
#define	NVDATA_H

// **** GLOBAL INCLUDE DIRECTIVES ****
#include <stdbool.h>
#include <stdint.h>
// **** END GLOBAL INCLUDE DIRECTIVES ****
// -----------------------------------------------------------------------------
// **** GLOBAL MACROS ****
#define NVM_PRESERVE_MEMORY_START_ADDR (0x9D00C000u)
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
    void NVData_Init(void);
    void NVData_Test(void);
    // **** END GLOBAL FUNCTION PROTOTYPES ****
    // -------------------------------------------------------------------------

#ifdef	__cplusplus
}
#endif

#endif	/* NVDATA_H */

