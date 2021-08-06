/* 
 * File:   NVData.c
 * Author: Kodiak North
 * 
 * Description: Module implements Non-Volatile Data storage using a portion of
 * the system's flash memory. The chosen programmer/debugger must be configured
 * to preserve a program memory range within the Project Properties menu.
 * A minimum of 1 page of flash must be preserved.
 * NVM_PRESERVE_MEMORY_START_ADDR is the virtual address that corresponds to
 * the physical start address that is preserved.
 *
 * Created on August 6, 2021, 11:36 AM
 */

// **** MODULE INCLUDE DIRECTIVES ****
#include <string.h>

#include "NVData.h"
#include "Print.h"

#include "peripheral/nvm/plib_nvm.h"
#include "peripheral/gpio/plib_gpio.h"
// **** END MODULE INCLUDE DIRECTIVES ****
// -----------------------------------------------------------------------------
// **** MODULE MACROS ****
#define BUFFER_SIZE (100u)
#define NVM_READ_SIZE (BUFFER_SIZE * sizeof(uint32_t))
// **** END MODULE MACROS ****
// -----------------------------------------------------------------------------
// **** MODULE TYPEDEFS ****

// **** END MODULE TYPEDEFS ****
// -----------------------------------------------------------------------------
// **** MODULE GLOBAL VARIABLES ****
volatile bool transferDone = false;
// **** END MODULE GLOBAL VARIABLES ****
// -----------------------------------------------------------------------------
// **** MODULE FUNCTION PROTOTYPES ****
void NVData_Callback(uintptr_t context);
// **** MODULE FUNCTION PROTOTYPES ****
// -----------------------------------------------------------------------------

void NVData_Init(void) {
    NVM_CallbackRegister(NVData_Callback, (uintptr_t) NULL);
}

void NVData_Test(void) {
    static uint32_t writeData[BUFFER_SIZE] CACHE_ALIGN;
    static uint32_t readData[BUFFER_SIZE];
    uint32_t i;

    for (i = 0; i < BUFFER_SIZE; i++) {
        writeData[i] = i + 1;
    }

    while (NVM_IsBusy() == true);

    //* write
    NVM_PageErase(NVM_PRESERVE_MEMORY_START_ADDR);
    while (transferDone == false);
    transferDone = false;

    // write 100 chunks of uint32_t
    uint32_t address = NVM_PRESERVE_MEMORY_START_ADDR;
    for (i = 0; i < BUFFER_SIZE; i++) {
        // write a word into NVM
        NVM_WordWrite(writeData[i], address);
        // wait for write to finish
        while (transferDone == false);
        transferDone = false;

        address += sizeof (uint32_t);
    }
    //*/

    NVM_Read(readData, NVM_READ_SIZE, NVM_PRESERVE_MEMORY_START_ADDR);

    if (!memcmp(writeData, readData, NVM_READ_SIZE)) {
        LED1_Set();
    }
    Print_EnqueueMsg("rd[0] %lu, rd[1] %lu, rd[50] %lu, rd[99] %lu\n",
            readData[0], readData[1], readData[50], readData[99]);
}

void NVData_Callback(uintptr_t context) {
    transferDone = true;
}