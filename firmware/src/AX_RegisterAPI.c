/* 
 * File:   MH_AX_RegisterAPI.c
 * Author: Kodiak North
 * 
 * Description: Defines functions to interface directly with AX5043/5243 registers
 *
 * Created on November 15, 2021, 9:26 AM
 */

// **** MODULE INCLUDE DIRECTIVES ****
#include <string.h>

#include "AX_RegisterAPI.h"
#include "AX_Registers.h"

#include "peripheral/gpio/plib_gpio.h"
#include "peripheral/spi/spi_master/plib_spi1_master.h"
#include "Time.h"
// **** END MODULE INCLUDE DIRECTIVES ****
// -----------------------------------------------------------------------------
// **** MODULE MACROS ****
#define USE_8_BIT_WRITES
// **** END MODULE MACROS ****
// -----------------------------------------------------------------------------
// **** MODULE TYPEDEFS ****

// **** END MODULE TYPEDEFS ****
// -----------------------------------------------------------------------------
// **** MODULE GLOBAL VARIABLES ****
uint16_t AXStatus = 0;
// **** END MODULE GLOBAL VARIABLES ****
// -----------------------------------------------------------------------------
// **** MODULE FUNCTION PROTOTYPES ****
void AX_SPI_Transfer(unsigned char* data, uint8_t length);
// **** END MODULE FUNCTION PROTOTYPES ****
// -----------------------------------------------------------------------------

uint16_t AX_GetStatus(void) {
    return AXStatus;
}

void AX_WriteLong8(uint16_t reg, uint8_t value) {
    uint8_t buf[3];

    buf[0] = (((uint8_t) (reg >> 8) & 0x007F) | 0xF0); // set top four bits for write operation and to follow along with programming manual protocol
    buf[1] = (uint8_t) (reg & 0x00FF);
    buf[2] = value;
    AX_SPI_Transfer(buf, 3);

    AXStatus = ((uint16_t) (buf[0] << 8)) | buf[1];
}

void AX_Write8(uint16_t reg, uint8_t value) {
    if (reg > 0x0070) { /* long access */
        AX_WriteLong8(reg, value);
    } else { /* short access */
        uint8_t buf[2];

        buf[0] = (((uint8_t) (reg & 0x007F)) | 0x80); // set MSB to indicate write operation on MOSI
        buf[1] = value;
        AX_SPI_Transfer(buf, 2);

        AXStatus = ((uint16_t) (buf[0] << 8));
    }
}

#ifndef USE_8_BIT_WRITES
void AX_WriteLong16(uint16_t reg, uint16_t value) {
    uint8_t buf[4];

    buf[0] = (((uint8_t) (reg >> 8) & 0x007F) | 0xF0); // set top four bits for write operation and to follow along with programming manual protocol
    buf[1] = (uint8_t) (reg & 0x00FF);
    buf[2] = (uint8_t) (value >> 8);
    buf[3] = (uint8_t) (value >> 0);
    AX_SPI_Transfer(buf, 4);

    AXStatus = ((uint16_t) (buf[0] << 8)) | buf[1];
}
#endif

void AX_Write16(uint16_t reg, uint16_t value) {
#ifndef USE_8_BIT_WRITES
    if (reg > 0x0070) { /* long access */
        AX_WriteLong16(reg, value);
    } else { /* short access */
        uint8_t buf[3];

        buf[0] = (((uint8_t) (reg & 0x007F)) | 0x80); // set MSB to indicate write operation on MOSI
        buf[1] = (uint8_t) (value >> 8);
        buf[2] = (uint8_t) (value >> 0);
        AX_SPI_Transfer(buf, 3);

        AXStatus = ((uint16_t) (buf[0] << 8));
    }
#else
    AX_Write8(reg,   (uint8_t) ((value & 0xFF00) >> 8));
    AX_Write8(reg+1, (uint8_t) ((value & 0x00FF) >> 0));
#endif
}

#ifndef USE_8_BIT_WRITES
void AX_WriteLong24(uint16_t reg, uint32_t value) {
    uint8_t buf[5];

    buf[0] = (((uint8_t) (reg >> 8) & 0x007F) | 0xF0); // set top four bits for write operation and to follow along with programming manual protocol
    buf[1] = (uint8_t) (reg & 0x00FF);
    buf[2] = (uint8_t) (value >> 16);
    buf[3] = (uint8_t) (value >> 8);
    buf[4] = (uint8_t) (value >> 0);
    AX_SPI_Transfer(buf, 5);

    AXStatus = ((uint16_t) (buf[0] << 8)) | buf[1];
}
#endif

void AX_Write24(uint16_t reg, uint32_t value) {
#ifndef USE_8_BIT_WRITES
    if (reg > 0x0070) { /* long access */
        AX_WriteLong24(reg, value);
    } else { /* short access */
        uint8_t buf[4];

        buf[0] = (((uint8_t) (reg & 0x007F)) | 0x80); // set MSB to indicate write operation on MOSI
        buf[1] = (uint8_t) (value >> 16);
        buf[2] = (uint8_t) (value >> 8);
        buf[3] = (uint8_t) (value >> 0);
        AX_SPI_Transfer(buf, 4);

        AXStatus = ((uint16_t) (buf[0] << 8));
    }
#else
    AX_Write8(reg,   (uint8_t) ((value & 0x00FF0000) >> 16));
    AX_Write8(reg+1, (uint8_t) ((value & 0x0000FF00) >> 8));
    AX_Write8(reg+2, (uint8_t) ((value & 0x000000FF) >> 0));
#endif
}

#ifndef USE_8_BIT_WRITES
void AX_WriteLong32(uint16_t reg, uint32_t value) {
    uint8_t buf[6];

    buf[0] = (((uint8_t) (reg >> 8) & 0x007F) | 0xF0); // set top four bits for write operation and to follow along with programming manual protocol
    buf[1] = (uint8_t) (reg & 0x00FF);
    buf[2] = (uint8_t) (value >> 24);
    buf[3] = (uint8_t) (value >> 16);
    buf[4] = (uint8_t) (value >> 8);
    buf[5] = (uint8_t) (value >> 0);
    AX_SPI_Transfer(buf, 6);

    AXStatus = ((uint16_t) (buf[0] << 8)) | buf[1];
}
#endif

void AX_Write32(uint16_t reg, uint32_t value) {
#ifndef USE_8_BIT_WRITES
    if (reg > 0x0070) { /* long access */
        AX_WriteLong32(reg, value);
    } else { /* short access */
        uint8_t buf[5];

        buf[0] = (((uint8_t) (reg & 0x007F)) | 0x80); // set MSB to indicate write operation on MOSI
        buf[1] = (uint8_t) (value >> 24);
        buf[2] = (uint8_t) (value >> 16);
        buf[3] = (uint8_t) (value >> 8);
        buf[4] = (uint8_t) (value >> 0);
        AX_SPI_Transfer(buf, 5);

        AXStatus = ((uint16_t) (buf[0] << 8));
    }
#else
    AX_Write8(reg,   (uint8_t) ((value & 0xFF000000) >> 24));
    AX_Write8(reg+1, (uint8_t) ((value & 0x00FF0000) >> 16));
    AX_Write8(reg+2, (uint8_t) ((value & 0x0000FF00) >> 8));
    AX_Write8(reg+3, (uint8_t) ((value & 0x000000FF) >> 0));
#endif
}

uint8_t AX_ReadLong8(uint16_t reg) {
    uint8_t buf[3];

    buf[0] = (((uint8_t) (reg >> 8) & 0x007F) | 0x70);
    buf[1] = (uint8_t) (reg & 0x00FF);
    buf[2] = 0xFF;
    AX_SPI_Transfer(buf, 3);

    AXStatus = ((uint16_t) (buf[0] << 8)) | buf[1];
    return buf[2];
}

uint8_t AX_Read8(uint16_t reg) {
    uint8_t returnVal = 0;
    if (reg > 0x70) { /* long access */
        returnVal = AX_ReadLong8(reg);
    } else { /* short access */
        uint8_t buf[2];

        buf[0] = (uint8_t) (reg & 0x007F);
        buf[1] = 0xFF;
        AX_SPI_Transfer(buf, 2);

        AXStatus = ((uint16_t) buf[0] << 8);
        returnVal = buf[1];
    }
    return returnVal;
}

uint16_t AX_ReadLong16(uint16_t reg) {
    uint8_t buf[4];

    buf[0] = (((uint8_t) (reg >> 8) & 0x007F) | 0x0070);
    buf[1] = (uint8_t) (reg & 0x00FF);
    buf[2] = 0xFF;
    buf[3] = 0xFF;
    AX_SPI_Transfer(buf, 4);

    AXStatus = ((uint16_t) (buf[0] << 8)) | buf[1];
    return (uint16_t) (buf[2] << 8) | (buf[3] << 0);
}

uint16_t AX_Read16(uint16_t reg) {
    uint16_t returnVal = 0;

    if (reg > 0x70) { /* long access */
        returnVal = AX_ReadLong16(reg);
    } else { /* short access */
        uint8_t buf[3];

        buf[0] = (reg & 0x7F);
        buf[1] = 0xFF;
        buf[2] = 0xFF;
        AX_SPI_Transfer(buf, 3);

        AXStatus = ((uint16_t) buf[0] << 8);
        returnVal = (uint16_t) (buf[1] << 8) | (buf[2] << 0);
    }
    return returnVal;
}

uint32_t AX_ReadLong24(uint16_t reg) {
    uint8_t buf[5];

    buf[0] = (((uint8_t) (reg >> 8) & 0x007F) | 0x0070);
    buf[1] = (uint8_t) (reg & 0x00FF);
    buf[2] = 0xFF;
    buf[3] = 0xFF;
    buf[4] = 0xFF;
    AX_SPI_Transfer(buf, 5);

    AXStatus = ((uint16_t) (buf[0] << 8)) | buf[1];
    return (uint32_t) (buf[2] << 16) | (buf[3] << 8) | (buf[4] << 0);
}

uint32_t AX_Read24(uint16_t reg) {
    uint32_t returnVal = 0;

    if (reg > 0x70) { /* long access */
        returnVal = AX_ReadLong24(reg);
    } else { /* short access */
        uint8_t buf[4];

        buf[0] = (reg & 0x7F);
        buf[1] = 0xFF;
        buf[2] = 0xFF;
        buf[3] = 0xFF;
        AX_SPI_Transfer(buf, 4);

        AXStatus = ((uint16_t) buf[0] << 8);
        returnVal = (uint32_t) (buf[1] << 16) | (buf[2] << 8) | (buf[3] << 0);
    }
    return returnVal;
}

uint32_t AX_ReadLong32(uint16_t reg) {
    uint8_t buf[6];

    buf[0] = (((uint8_t) (reg >> 8) & 0x007F) | 0x0070);
    buf[1] = (uint8_t) (reg & 0x00FF);
    buf[2] = 0xFF;
    buf[3] = 0xFF;
    buf[4] = 0xFF;
    buf[5] = 0xFF;
    AX_SPI_Transfer(buf, 6);

    AXStatus = ((uint16_t) (buf[0] << 8)) | buf[1];
    return (uint32_t) (buf[2] << 24) | (buf[3] << 16) | (buf[4] << 8) | (buf[5] << 0);
}

uint32_t AX_Read32(uint16_t reg) {
    uint32_t returnVal = 0;

    if (reg > 0x70) { /* long access */
        returnVal = AX_ReadLong32(reg);
    } else { /* short access */
        uint8_t buf[5];

        buf[0] = (reg & 0x7F);
        buf[1] = 0xFF;
        buf[2] = 0xFF;
        buf[3] = 0xFF;
        buf[4] = 0xFF;
        AX_SPI_Transfer(buf, 5);

        AXStatus = ((uint16_t) buf[0] << 8);
        returnVal = (uint32_t) (buf[1] << 24) | (buf[2] << 16) | (buf[3] << 8) | (buf[4] << 0);
    }
    return returnVal;
}

void AX_WriteFifo(uint8_t* buffer, uint8_t length) {
    // One SPI transaction for each byte in the buffer, as per easyax example code
    uint8_t i;
    for (i = 0; i < length; i++) {
        AX_Write8(AX_REG_FIFODATA, buffer[i]);
    }
}

bool AX_SPITest(uint16_t reg, uint8_t value) {
    /* this should be used on 1 byte registers only */
    AX_Write8(reg, value);
    uint8_t test = AX_Read8(reg);
    return (test == value);
}

// **** MODULE FUNCTION IMPLEMENTATIONS ****
bool IsSPIBusy(void) {
    // return true if the SPI SS line is active (LOW)
    return (SPI1_SS_Get() == 0);
}

bool IsSPIIdle(void) {
    // return true if the SPI SS line is inactive (HIGH)
    return (SPI1_SS_Get() == 1);
}

void AX_SPI_Transfer(unsigned char* data, uint8_t length) {
    static unsigned char dataToReceive[256];
    SPI1_WriteRead(data, length, dataToReceive, length);
    memcpy(data, dataToReceive, length);
    while (!IsSPIIdle()) {};
}
// **** END MODULE FUNCTION IMPLEMENTATIONS ****
