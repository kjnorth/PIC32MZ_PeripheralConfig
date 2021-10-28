/* 
 * File:   MH_AX.c
 * Author: Kodiak North
 * 
 * Description: Masterhaul's implementation of a library to interface with
 * OnSemiconductor's cumbersome AX-5043/5243 transceivers
 *
 * Created on October 28, 2021, 1:13 PM
 */

// **** MODULE INCLUDE DIRECTIVES ****
#include <string.h>

#include "MH_AX.h"

#include "ax/ax_fifo.h"
#include "ax/ax.h"
#include "ax/ax_hw.h"
#include "ax/ax_modes.h"
#include "ax/ax_reg.h"
#include "ax/ax_reg_values.h"

#include "Print.h"

#include "peripheral/spi/spi_master/plib_spi1_master.h"
// **** END MODULE INCLUDE DIRECTIVES ****
// -----------------------------------------------------------------------------
// **** MODULE MACROS ****
//#define _DEBUG
#ifdef _DEBUG
#define debug Print_EnqueueMsg
#else
#define debug(...)
#endif
// **** END MODULE MACROS ****
// -----------------------------------------------------------------------------
// **** MODULE TYPEDEFS ****

// **** END MODULE TYPEDEFS ****
// -----------------------------------------------------------------------------
// **** MODULE GLOBAL VARIABLES ****
uint16_t AXStatus;
// **** END MODULE GLOBAL VARIABLES ****
// -----------------------------------------------------------------------------
// **** MODULE FUNCTION PROTOTYPES ****
int AX_WritePacketToFifo(uint8_t* txPacket, uint8_t length);
void AX_WriteFifo(uint8_t* buffer, uint8_t length);

void AX_WriteRegister8(uint16_t reg, uint8_t value);
void AX_WriteRegisterLong8(uint16_t reg, uint8_t value);
uint8_t AX_ReadRegister8(uint16_t reg);
uint8_t AX_ReadRegisterLong8(uint16_t reg);

void AX_SPI_Transfer(unsigned char* data, uint8_t length);
// **** END MODULE FUNCTION PROTOTYPES ****
// -----------------------------------------------------------------------------

void AX_InitTx(void);
void AX_InitRx(void);

int AX_TransmitPacket(uint8_t* txPacket, uint8_t length) {
    uint8_t returnVal = 0;
    
    /* Clear FIFO */
    AX_WriteRegister8(AX_REG_FIFOSTAT, AX_FIFOCMD_CLEAR_FIFO_DATA_AND_FLAGS);

    /* Place chip in FULLTX mode with crystal oscillator and reference circuitry enabled */
    AX_WriteRegister8(AX_REG_PWRMODE, AX_PWRMODE_XOEN | AX_PWRMODE_REFEN | AX_PWRMODE_FULLTX);

    /* enable the tcxo if used */
    // TODO: add functionality

    /* write preamble and packet to FIFO */
    if (AX_WritePacketToFifo(txPacket, length)) {
        /* Wait for oscillator to start running  */
        // block for now, can config to be interrupt driven later
        while (AX_ReadRegister8(AX_REG_XTALSTATUS) != 0x01) {};
        
        AX_PrintStatus();

        /* commit packet to the FIFO for tx */
        AX_WriteRegister8(AX_REG_FIFOSTAT, AX_FIFOCMD_COMMIT);

        /* wait for tx to complete */
        // block for now, can config to be interrupt driven later
        while (AX_ReadRegister8(AX_REG_RADIOSTATE) != 0x00) {};
        returnVal = 1;
    }

    /* disable the tcxo if used CHANGE USING FUNCTION POINTERS LATER */
    // TODO: add functionality

    /* Place chip in POWERDOWN mode */
    AX_WriteRegister8(AX_REG_PWRMODE, AX_PWRMODE_POWERDOWN);

    return returnVal;
}

void AX_ReceivePacket(uint8_t* rxPacket);

uint16_t AX_GetStatus(void) {
    return AXStatus;
}

void AX_PrintStatus(void) {
    uint16_t status = AXStatus;
    uint8_t fifoStat = AX_ReadRegister8(AX_REG_FIFOSTAT);
    Print_EnqueueMsg("hwStatus - %u%u%u%u %u%u%u%u %u%u%u%u %u%u%u%u, fifoStatus - %u%u%u%u %u%u%u%u\n",
            ((status & (1 << 15)) >> 15),
            ((status & (1 << 14)) >> 14),
            ((status & (1 << 13)) >> 13),
            ((status & (1 << 12)) >> 12),
            ((status & (1 << 11)) >> 11),
            ((status & (1 << 10)) >> 10),
            ((status & (1 << 9)) >> 9),
            ((status & (1 << 8)) >> 8),
            ((status & (1 << 7)) >> 7),
            ((status & (1 << 6)) >> 6),
            ((status & (1 << 5)) >> 5),
            ((status & (1 << 4)) >> 4),
            ((status & (1 << 3)) >> 3),
            ((status & (1 << 2)) >> 2),
            ((status & (1 << 1)) >> 1),
            ((status & (1 << 0)) >> 0),
            ((fifoStat & (1 << 7)) >> 7),
            ((fifoStat & (1 << 6)) >> 6),
            ((fifoStat & (1 << 5)) >> 5),
            ((fifoStat & (1 << 4)) >> 4),
            ((fifoStat & (1 << 3)) >> 3),
            ((fifoStat & (1 << 2)) >> 2),
            ((fifoStat & (1 << 1)) >> 1),
            ((fifoStat & (1 << 0)) >> 0));
}

// **** MODULE FUNCTION IMPLEMENTATIONS ****
int AX_WritePacketToFifo(uint8_t* txPacket, uint8_t length) {
    if (length > AX_TX_PACKET_MAX_SIZE) {
        debug("tx packet is too large, aborting\n");
        return 0;
    }
    
    /* disable Rx interrupts */
    // TODO

    /* write a preamble */ 
    uint8_t fifoCmd[4];
    fifoCmd[0] = AX_FIFO_CHUNK_REPEATDATA;
    fifoCmd[1] = AX_FIFO_TXDATA_UNENC | AX_FIFO_TXDATA_NOCRC;
    fifoCmd[2] = 20; // preamble length of 20 bytes
    fifoCmd[3] = 0xAA; // preamble value, sent preamble length times
    AX_WriteFifo(fifoCmd, 4);

    /* write commands to fifo to prepare it for the incoming packet */
    fifoCmd[0] = AX_FIFO_CHUNK_DATA; // fifo data command
    fifoCmd[1] = length + 1; // length byte, +1 to include control field byte
    fifoCmd[2] = (AX_FIFO_TXDATA_PKTSTART | AX_FIFO_TXDATA_PKTEND); // control field byte
    AX_WriteFifo(fifoCmd, 3);

    /* write the packet */
    AX_WriteFifo(txPacket, length);
    
    /* enable Tx interrupts */
    // TODO

    return 1;
}

void AX_WriteFifo(uint8_t* buffer, uint8_t length) {
    /* code from ax lib, I think it performs redundant and unnecessary operations
    uint8_t data[0x100];

    // write (short access)
    data[0] = ((AX_REG_FIFODATA & 0x7F) | 0x80);
    memcpy(data + 1, buffer, length);

    config->spi_transfer(data, length + 1);

    status &= 0xFF;
    status |= ((uint16_t) data[0] << 8);

    return status;
    */

    AX_SPI_Transfer(buffer, length);

    AXStatus &= 0x00FF;
    AXStatus |= ((uint16_t) buffer[0] << 8);
}

void AX_WriteRegister8(uint16_t reg, uint8_t value) {
    if (reg > 0x70) { /* long access */
        AX_WriteRegisterLong8(reg, value);
    } else { /* short access */
        unsigned char data[2];

        data[0] = ((reg & 0x7F) | 0x80);
        data[1] = value;
        AX_SPI_Transfer(data, 2);

        AXStatus &= 0xFF;
        AXStatus |= ((uint16_t) data[0] << 8);
    }
}

void AX_WriteRegisterLong8(uint16_t reg, uint8_t value) {
    unsigned char data[3];

    data[0] = ((reg >> 8) | 0xF0);
    data[1] = (reg & 0xFF);
    data[2] = value;
    AX_SPI_Transfer(data, 3);

    AXStatus = ((uint16_t) data[0] << 8) & data[1];
}

uint8_t AX_ReadRegister8(uint16_t reg) {
    uint8_t returnVal = 0;

    if (reg > 0x70) { /* long access */
        returnVal = AX_ReadRegisterLong8(reg);
    } else { /* short access */
        unsigned char data[2];

        data[0] = (reg & 0x7F);
        data[1] = 0xFF;
        AX_SPI_Transfer(data, 2);

        AXStatus &= 0xFF;
        AXStatus |= ((uint16_t) data[0] << 8);

        returnVal = (uint8_t) data[1];
    }

    return returnVal;
}

uint8_t AX_ReadRegisterLong8(uint16_t reg) {
    unsigned char data[3];

    data[0] = ((reg >> 8) | 0x70);
    data[1] = (reg & 0xFF);
    data[2] = 0xFF;
    AX_SPI_Transfer(data, 3);

    AXStatus = ((uint16_t) data[0] << 8) & data[1];

    return (uint8_t) data[2];
}

void AX_SPI_Transfer(unsigned char* data, uint8_t length) {
    static unsigned char dataToReceive[256];
    SPI1_WriteRead(data, length, dataToReceive, length);
    memcpy(data, dataToReceive, length);
}
// **** END MODULE FUNCTION IMPLEMENTATIONS ****
