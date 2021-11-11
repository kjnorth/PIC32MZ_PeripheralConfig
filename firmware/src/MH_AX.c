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
#include "Time.h"

#include "peripheral/gpio/plib_gpio.h"
#include "peripheral/spi/spi_master/plib_spi1_master.h"
// **** END MODULE INCLUDE DIRECTIVES ****
// -----------------------------------------------------------------------------
// **** MODULE MACROS ****
#define _DEBUG
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
ax_mode_t Mode = AX_MODE_NONE;
uint16_t AXStatus = 0;
uint8_t PllRangingA = 0;
uint8_t ChannelVcoi = 0; // access this variable when ax crazy library calls axradio_get_pllvcoi function
// **** END MODULE GLOBAL VARIABLES ****
// -----------------------------------------------------------------------------
// **** MODULE FUNCTION PROTOTYPES ****
uint8_t AX_InitRegistersCommon(void);

int AX_WritePacketToFifo(uint8_t* txPacket, uint8_t length);
void AX_WriteFifo(uint8_t* buffer, uint8_t length);

void AX_EnableOscillator(void);
void AX_DisableOscillator(void);

void AX_Write8(uint16_t reg, uint8_t value);
void AX_Write16(uint16_t reg, uint16_t value);
void AX_Write24(uint16_t reg, uint32_t value);
void AX_Write32(uint16_t reg, uint32_t value);

uint8_t AX_Read8(uint16_t reg);
uint16_t AX_Read16(uint16_t reg);
uint32_t AX_Read24(uint16_t reg);
uint32_t AX_Read32(uint16_t reg);

void AX_SPI_Transfer(unsigned char* data, uint8_t length);

uint8_t AX_AdjustVcoi(uint8_t rng);
int16_t AX_TuneVoltage(void);
// **** END MODULE FUNCTION PROTOTYPES ****
// -----------------------------------------------------------------------------

bool AX_Init(ax_mode_t _mode) {
    Mode = _mode;
    uint8_t revision = 0, scratch = 0;
    revision = AX_Read8(AX_REG_SILICONREVISION);
    scratch = AX_Read8(AX_REG_SCRATCH);
    if ((revision != AX_SILICONREVISION) || (scratch != AX_SCRATCH)) {
        // chip likely dead or not connected
        debug("ax chip not found... revision 0x%02X, scratch 0x%02X\n", revision, scratch);
        return false;
    }
    // TODO: reset chip now?
        
    AX_InitConfigRegisters();
    AX_InitTxRegisters();
    
    AX_Write8(AX_REG_PLLLOOP, 0x09);
    AX_Write8(AX_REG_PLLCPI, 0x08);
    AX_Write8(AX_REG_PWRMODE, AX_PWRMODE_XOEN);
    AX_Write8(AX_REG_MODULATION, 0x08);
    AX_Write24(AX_REG_FSKDEV, 0x000000);
    /* Wait for oscillator to start running  */
    while (AX_Read8(AX_REG_XTALSTATUS) != 0x01) {};
    
    // write synthesizer frequency A
    AX_Write32(AX_REG_FREQA, 0x1216EEEF); // NOTE: this may need to be changed when switching to internal crystal
    
    AX_Write8(AX_REG_PLLRANGINGA, 0x1A); // start the auto ranging process with VCO_A range of 10
    // wait for ranging to complete
    do {
        PllRangingA = AX_Read8(AX_REG_PLLRANGINGA);
    } while ((PllRangingA & AX_PLLRANGING_RNG_START) != 0x00);
    if ((PllRangingA & AX_PLLRANGING_RNGERR) == AX_PLLRANGING_RNGERR) {
        // range error, programmed frequency cannot be achieved
        debug("ax ranging failed, PLLRANGING = 0x%02X\n", PllRangingA);
        return false;
    }
    
    // VCOI calibration
    AX_InitTxRegisters();
    AX_Write8(AX_REG_PLLLOOP, 0x0D);
    AX_Write8(AX_REG_0xF35, 0x92);
    
    AX_Write8(AX_REG_PWRMODE, AX_PWRMODE_SYNTHTX);
    uint8_t vcoiSave = AX_Read8(AX_REG_PLLVCOI);
    AX_Write8(AX_REG_PLLRANGINGA, PllRangingA & 0x0F);
    AX_Write32(AX_REG_FREQA, 0x1216EEEF); // NOTE: this may need to be changed when switching to internal crystal
    
    uint8_t i;
    for (i = 0; i < 2; i++) {
        uint8_t x = 0x99;
        x += (PllRangingA & 0x0F) - 0x0A;
        ChannelVcoi = AX_AdjustVcoi(x);
    }
    
    AX_Write8(AX_REG_PLLVCOI, vcoiSave);
    AX_Write8(AX_REG_PWRMODE, AX_PWRMODE_POWERDOWN);
    AX_InitConfigRegisters();
    AX_Write32(AX_REG_FREQA, 0x1216EEEF); // NOTE: this may need to be changed when switching to internal crystal
    
    if (Mode == AX_MODE_PRX) {
        AX_InitRxRegisters();
    }
    
    return true;
}

void AX_InitConfigRegisters(void) {
    AX_Write8(AX_REG_POWIRQMASK, 0x00); // power related interrupts
    AX_Write16(AX_REG_IRQMASK, 0x0000); // enable specific interrupts with this register, upon IRQ, read IRQREQUEST register
    AX_Write16(AX_REG_RADIOEVENTMASK, 0x00); // enable more interrupts with this register
    AX_Write8(AX_REG_MODULATION, 0x08);
    AX_Write8(AX_REG_ENCODING, 0x00);
    AX_Write8(AX_REG_FRAMING, 0x26);
    AX_Write8(AX_REG_PINFUNCSYSCLK, 0x04); // set to output frequency of oscillator used (either internal or TCXO)
    AX_Write8(AX_REG_PINFUNCDCLK, 0x00); // set to output '0'
    AX_Write8(AX_REG_PINFUNCDATA, 0x00); // set to output '0'
    AX_Write8(AX_REG_PINFUNCIRQ, 0x00); // 0x~0 to output '0', 0x~3 to enable IRQ
    AX_Write8(AX_REG_PINFUNCANTSEL, 0x00); // set to output '0'
    AX_Write8(AX_REG_PINFUNCPWRAMP, 0x07); // 0x~0 for internal oscillator use, 0x~7 to enable TCXO
    AX_Write16(AX_REG_FIFOTHRESH, 0x0000);
    AX_Write8(AX_REG_WAKEUPXOEARLY, 0x01);
    AX_Write16(AX_REG_IFFREQ, 0x0106);
    AX_Write8(AX_REG_DECIMATION, 0x29);
    AX_Write24(AX_REG_RXDATARATE, 0x003CF9);
    AX_Write24(AX_REG_MAXDROFFSET, 0x000000);
    AX_Write24(AX_REG_MAXRFOFFSET, 0x80025F);
    AX_Write16(AX_REG_FSKDMAX, 0x00A6);
    AX_Write16(AX_REG_FSKDMIN, 0xFF5A);
    AX_Write8(AX_REG_AMPLFILTER, 0x00);
    AX_Write8(AX_REG_RXPARAMSETS, 0xF4);
    AX_Write8((AX_REG_RX_PARAM_SET0 | AX_RX_AGCGAIN), 0xD6);
    AX_Write8((AX_REG_RX_PARAM_SET0 | AX_RX_AGCTARGET), 0x84);
    AX_Write8((AX_REG_RX_PARAM_SET0 | AX_RX_TIMEGAIN), 0xF8);
    AX_Write8((AX_REG_RX_PARAM_SET0 | AX_RX_DRGAIN), 0xF2);
    AX_Write8((AX_REG_RX_PARAM_SET0 | AX_RX_PHASEGAIN), 0xC3);
    AX_Write8((AX_REG_RX_PARAM_SET0 | AX_RX_FREQUENCYGAINA), 0x0F);
    AX_Write8((AX_REG_RX_PARAM_SET0 | AX_RX_FREQUENCYGAINB), 0x1F);
    AX_Write8((AX_REG_RX_PARAM_SET0 | AX_RX_FREQUENCYGAINC), 0x09);
    AX_Write8((AX_REG_RX_PARAM_SET0 | AX_RX_FREQUENCYGAIND), 0x09);
    AX_Write8((AX_REG_RX_PARAM_SET0 | AX_RX_AMPLITUDEGAIN), 0x06);
    AX_Write16((AX_REG_RX_PARAM_SET0 | AX_RX_FREQDEV), 0x0000);
    AX_Write8((AX_REG_RX_PARAM_SET0 | AX_RX_BBOFFSRES), 0x00);
    AX_Write8((AX_REG_RX_PARAM_SET1 | AX_RX_AGCGAIN), 0xD6);
    AX_Write8((AX_REG_RX_PARAM_SET1 | AX_RX_AGCTARGET), 0x84);
    AX_Write8((AX_REG_RX_PARAM_SET1 | AX_RX_AGCAHYST), 0x00);
    AX_Write8((AX_REG_RX_PARAM_SET1 | AX_RX_AGCMINMAX), 0x00);
    AX_Write8((AX_REG_RX_PARAM_SET1 | AX_RX_TIMEGAIN), 0xF6);
    AX_Write8((AX_REG_RX_PARAM_SET1 | AX_RX_DRGAIN), 0xF1);
    AX_Write8((AX_REG_RX_PARAM_SET1 | AX_RX_PHASEGAIN), 0xC3);
    AX_Write8((AX_REG_RX_PARAM_SET1 | AX_RX_FREQUENCYGAINA), 0x0F);
    AX_Write8((AX_REG_RX_PARAM_SET1 | AX_RX_FREQUENCYGAINB), 0x1F);
    AX_Write8((AX_REG_RX_PARAM_SET1 | AX_RX_FREQUENCYGAINC), 0x09);
    AX_Write8((AX_REG_RX_PARAM_SET1 | AX_RX_FREQUENCYGAIND), 0x09);
    AX_Write8((AX_REG_RX_PARAM_SET1 | AX_RX_AMPLITUDEGAIN), 0x06);
    AX_Write16((AX_REG_RX_PARAM_SET1 | AX_RX_FREQDEV), 0x0043);
    AX_Write8((AX_REG_RX_PARAM_SET1 | AX_RX_FOURFSK), 0x16);
    AX_Write8((AX_REG_RX_PARAM_SET1 | AX_RX_BBOFFSRES), 0x00);
    AX_Write8((AX_REG_RX_PARAM_SET3 | AX_RX_AGCGAIN), 0xFF);
    AX_Write8((AX_REG_RX_PARAM_SET3 | AX_RX_AGCTARGET), 0x84);
    AX_Write8((AX_REG_RX_PARAM_SET3 | AX_RX_AGCAHYST), 0x00);
    AX_Write8((AX_REG_RX_PARAM_SET3 | AX_RX_AGCMINMAX), 0x00);
    AX_Write8((AX_REG_RX_PARAM_SET3 | AX_RX_TIMEGAIN), 0xF5);
    AX_Write8((AX_REG_RX_PARAM_SET3 | AX_RX_DRGAIN), 0xF0);
    AX_Write8((AX_REG_RX_PARAM_SET3 | AX_RX_PHASEGAIN), 0xC3);
    AX_Write8((AX_REG_RX_PARAM_SET3 | AX_RX_FREQUENCYGAINA), 0x0F);
    AX_Write8((AX_REG_RX_PARAM_SET3 | AX_RX_FREQUENCYGAINB), 0x1F);
    AX_Write8((AX_REG_RX_PARAM_SET3 | AX_RX_FREQUENCYGAINC), 0x0D);
    AX_Write8((AX_REG_RX_PARAM_SET3 | AX_RX_FREQUENCYGAIND), 0x0D);
    AX_Write8((AX_REG_RX_PARAM_SET3 | AX_RX_AMPLITUDEGAIN), 0x06);
    AX_Write16((AX_REG_RX_PARAM_SET3 | AX_RX_FREQDEV), 0x0043);
    AX_Write8((AX_REG_RX_PARAM_SET3 | AX_RX_FOURFSK), 0x16);
    AX_Write8((AX_REG_RX_PARAM_SET3 | AX_RX_BBOFFSRES), 0x00);
    AX_Write8(AX_REG_MODCFGF, 0x03);
    AX_Write24(AX_REG_FSKDEV, 0x00022F);
    AX_Write8(AX_REG_MODCFGA, 0x05);
    AX_Write24(AX_REG_TXRATE, 0x00068E);
    AX_Write16(AX_REG_TXPWRCOEFFB, 0x0FFF);
    AX_Write8(AX_REG_PLLVCOI, 0x99);
    AX_Write8(AX_REG_PLLRNGCLK, 0x05);
    AX_Write8(AX_REG_BBTUNE, 0x0F);
    AX_Write8(AX_REG_BBOFFSCAP, 0x77);
    AX_Write8(AX_REG_PKTADDRCFG, 0x01); // address bytes must start at position 1 in the buffer that is transmitted
    AX_Write8(AX_REG_PKTLENCFG, 0x80); // all bits in length byte are significant // does LEN POS of 0 mean pkt length must be in 0th pos of Tx buffer?
    AX_Write8(AX_REG_PKTLENOFFSET, 0x00);
    AX_Write8(AX_REG_PKTMAXLEN, 0xC8);
    if (Mode == AX_MODE_PTX) {
        AX_Write32(AX_REG_PKTADDR, 0x00003432);
    } else if (Mode == AX_MODE_PRX) {
        AX_Write32(AX_REG_PKTADDR, 0x00003433);
    }
    AX_Write32(AX_REG_PKTADDRMASK, 0x0000FFFF);
    AX_Write32(AX_REG_MATCH0PAT, 0xAACCAACC); // this is preamble0 that Rx uses to detect/accept an incoming packet
    AX_Write8(AX_REG_MATCH0LEN, 0x1F);
    AX_Write8(AX_REG_MATCH0MAX, 0x1F); // this says at least 31 bits of match0 pattern must match
    AX_Write16(AX_REG_MATCH1PAT, 0x5555); // this is preamble1 that Rx uses to detect/accept an incoming packet
//    AX_Write8(AX_REG_MATCH1LEN, 0x8A); // NOTE: changing this because it seems reasonable
    AX_Write8(AX_REG_MATCH1LEN, 0x0A);
    AX_Write8(AX_REG_MATCH1MAX, 0x0A); // this says at least 10 bits of match1 pattern must match
    AX_Write8(AX_REG_TMGTXBOOST, 0x5B);
    AX_Write8(AX_REG_TMGTXSETTLE, 0x3E);
    AX_Write8(AX_REG_TMGRXBOOST, 0x5B);
    AX_Write8(AX_REG_TMGRXSETTLE, 0x3E);
    AX_Write8(AX_REG_TMGRXOFFSACQ, 0x00);
    AX_Write8(AX_REG_TMGRXCOARSEAGC, 0x9C);
    AX_Write8(AX_REG_TMGRXRSSI, 0x03);
    AX_Write8(AX_REG_TMGRXPREAMBLE2, 0x35);
    AX_Write8(AX_REG_RSSIABSTHR, 0xE0);
    AX_Write8(AX_REG_BGNDRSSITHR, 0x00);
    AX_Write8(AX_REG_PKTCHUNKSIZE, 0x0D);
    AX_Write8(AX_REG_PKTSTOREFLAGS, 0x00/*0x15*/); // store timer value, RF frequency offset, RSSI
//    AX_Write8(AX_REG_PKTACCEPTFLAGS, 0x20); // accept packets that span multiple FIFO chunks
        AX_Write8(AX_REG_PKTACCEPTFLAGS, 0x3F); // accept any god dang packet
    AX_Write16(AX_REG_DACVALUE, 0x0000);
    AX_Write8(AX_REG_DACCONFIG, 0x00);
    AX_Write8(AX_REG_REF, 0x03);
    AX_Write8(AX_REG_0xF10, 0x04); // set to 0x04 for TCXO, 0x0D if frequency of TCXO or internal osc is > 43MHz, or 0x03 otherwise
    AX_Write8(AX_REG_0xF11, 0x00); // set to 0x00 for TCXO, 0x07 if crystal is used
    AX_Write8(AX_REG_0xF1C, 0x07);
    AX_Write8(AX_REG_0xF21, 0x68);
    AX_Write8(AX_REG_0xF22, 0xFF);
    AX_Write8(AX_REG_0xF23, 0x84);
    AX_Write8(AX_REG_0xF26, 0x98);
    AX_Write8(AX_REG_0xF34, 0x08);
    AX_Write8(AX_REG_0xF35, 0x11);
    AX_Write8(AX_REG_0xF44, 0x25);
}

void AX_InitTxRegisters(void) {
    AX_InitRegistersCommon();
    AX_Write8(AX_REG_PLLLOOP, 0x07);
    AX_Write8(AX_REG_PLLCPI, 0x12);
    AX_Write8(AX_REG_PLLVCODIV, 0x20);
    AX_Write8(AX_REG_XTALCAP, 0x00);
    AX_Write8(AX_REG_0xF00, 0x0F);
    AX_Write8(AX_REG_0xF18, 0x06);
}

void AX_InitRxRegisters(void) {
    AX_InitRegistersCommon();
    AX_Write8(AX_REG_PLLLOOP, 0x07);
    AX_Write8(AX_REG_PLLCPI, 0x08);
    AX_Write8(AX_REG_PLLVCODIV, 0x20);
    AX_Write8(AX_REG_XTALCAP, 0x00);
    AX_Write8(AX_REG_0xF00, 0x0F);
    AX_Write8(AX_REG_0xF18, 0x06);
    
    // rx on continuous
    AX_Write8(AX_REG_RSSIREFERENCE, 0x36); // set offset for the computed RSSI
    AX_Write8(AX_REG_TMGRXAGC, 0x00); // AGC settling time
    AX_Write8(AX_REG_TMGRXPREAMBLE1, 0x00); // preamble1 timeout - preamble 1 corresponds to MATCH1PAT // TODO: MAKE THESE NONZERO??
    AX_Write8(AX_REG_TMGRXPREAMBLE2, 0x00); // preamble2 timeout - preamble 2 corresponds to MATCH1PAT
    AX_Write8(AX_REG_TMGRXPREAMBLE3, 0x00); // preamble3 timeout - preamble 3 corresponds to MATCH0PAT
    AX_Write8(AX_REG_PKTMISCFLAGS, 0x00); // disable all miscellaneous packet flags
    
    AX_Write8(AX_REG_FIFOSTAT, AX_FIFOCMD_CLEAR_FIFO_DATA_AND_FLAGS); // clear FIFO
    AX_Write8(AX_REG_PWRMODE, AX_PWRMODE_XOEN | AX_PWRMODE_REFEN | AX_PWRMODE_FULLRX);
    AX_EnableOscillator();
    // TODO: enable FIFO not empty interrupt once those are used
}

uint8_t AX_InitRegistersCommon(void) {
    uint8_t rng = PllRangingA;
    if (rng & 0x20)
        return 0x06; // AXRADIO_ERR_RANGING;
    if (AX_Read8(AX_REG_PLLLOOP) & 0x80)
        AX_Write8(AX_REG_PLLRANGINGB, (rng & 0x0F));
    else
        AX_Write8(AX_REG_PLLRANGINGA, (rng & 0x0F));
    rng = ChannelVcoi;
    if (rng & 0x80)
        AX_Write8(AX_REG_PLLVCOI, rng);
    return 0x00; // AXRADIO_ERR_NOERROR;
}

uint16_t AX_GetStatus(void) {
    return AXStatus;
}

void AX_PrintStatus(void) {
    uint16_t status = AXStatus;
    uint8_t fifoStat = AX_Read8(AX_REG_FIFOSTAT);
    Print_EnqueueMsg("hwStatus - %u%u%u%u %u%u%u%u %u%u%u%u %u%u%u%u, fifoStatus - %u%u%u%u %u%u%u%u\n",
            ((status & (1 << 15)) >> 15), ((status & (1 << 14)) >> 14), ((status & (1 << 13)) >> 13), ((status & (1 << 12)) >> 12),
            ((status & (1 << 11)) >> 11), ((status & (1 << 10)) >> 10), ((status & (1 << 9)) >> 9),   ((status & (1 << 8)) >> 8),
            ((status & (1 << 7)) >> 7),   ((status & (1 << 6)) >> 6),   ((status & (1 << 5)) >> 5),   ((status & (1 << 4)) >> 4),
            ((status & (1 << 3)) >> 3),   ((status & (1 << 2)) >> 2),   ((status & (1 << 1)) >> 1),   ((status & (1 << 0)) >> 0),
            ((fifoStat & (1 << 7)) >> 7), ((fifoStat & (1 << 6)) >> 6), ((fifoStat & (1 << 5)) >> 5), ((fifoStat & (1 << 4)) >> 4),
            ((fifoStat & (1 << 3)) >> 3), ((fifoStat & (1 << 2)) >> 2), ((fifoStat & (1 << 1)) >> 1), ((fifoStat & (1 << 0)) >> 0));
}

void AX_ReceivePacket(uint8_t* rxPacket) {
    if ((AX_Read8(AX_REG_FIFOSTAT) & AX_FIFO_EMPTY) != AX_FIFO_EMPTY) {
        // fifo is not empty
        uint16_t fifoCnt = AX_Read16(AX_REG_FIFOCOUNT);
        LED1_Toggle();

        // empty fifo
        uint16_t cntRead = 0;
        uint8_t buf[PRINT_BUFFER_SIZE] = {0};
        do {
            buf[cntRead] = AX_Read8(AX_REG_FIFODATA);
            cntRead++;
        } while ((AX_Read8(AX_REG_FIFOSTAT) & AX_FIFO_EMPTY) != AX_FIFO_EMPTY);
        debug("RECEIVED, fifoCnt %u, cntRead %u\n", fifoCnt, cntRead);
        Print_EnqueueBuffer(buf, PRINT_BUFFER_SIZE);
    }
}

int AX_TransmitPacket(uint8_t* txPacket, uint8_t length) {
    uint8_t returnVal = 0;
    
    /* Clear FIFO */
    AX_Write8(AX_REG_FIFOSTAT, AX_FIFOCMD_CLEAR_FIFO_DATA_AND_FLAGS);

    /* Place chip in FULLTX mode with crystal oscillator and reference circuitry enabled */
    AX_Write8(AX_REG_PWRMODE, AX_PWRMODE_XOEN | AX_PWRMODE_REFEN | AX_PWRMODE_FULLTX);

    /* enable the oscillator */
    AX_EnableOscillator();
    
//    AX_Read8(AX_REG_POWSTICKYSTAT); // clear POWSTICKYSTAT register. I think this only needs to be done when using brownout gate interrupts??
    /* Wait for FIFO power ready */
    while ((AX_Read8(AX_REG_POWSTAT) & AX_POWSTAT_SVMODEM) != AX_POWSTAT_SVMODEM) {};

    /* write preamble and packet to FIFO */
    if (AX_WritePacketToFifo(txPacket, length)) {
        /* Wait for oscillator to start running  */
        // block for now, can config to be interrupt driven later
        while (AX_Read8(AX_REG_XTALSTATUS) != 0x01) {};
        
        /* wait for tx to complete */
        // block for now, can config to be interrupt driven later
        while (AX_Read8(AX_REG_RADIOSTATE) != 0x00) {};
        returnVal = 1;
    }

    /* disable the oscillator */
//    AX_DisableOscillator();

    /* Place chip in POWERDOWN mode */
//    AX_Write8(AX_REG_PWRMODE, AX_PWRMODE_POWERDOWN);

    return returnVal;
}

// **** MODULE FUNCTION IMPLEMENTATIONS ****
int AX_WritePacketToFifo(uint8_t* txPacket, uint8_t length) {
    if (length > AX_TX_PACKET_MAX_SIZE) {
        debug("tx packet is too large, aborting\n");
        return 0;
    }
        
    /* disable Rx interrupts */
    // TODO
    
    uint8_t fifoBytes[AX_TX_PACKET_MAX_SIZE], cnt = 0;

    /* write preamble 1/2, writing MATCHPAT1 bits */ 
    fifoBytes[cnt++] = AX_FIFO_CHUNK_REPEATDATA;
    fifoBytes[cnt++] = AX_FIFO_TXDATA_UNENC | AX_FIFO_TXDATA_RAW | AX_FIFO_TXDATA_NOCRC;
    fifoBytes[cnt++] = 4; // preamble length of 4 bytes
    fifoBytes[cnt++] = 0x55; // preamble value, sent preamble length times
    
    /* write preamble 3, writing MATCHPAT0 bits */
    fifoBytes[cnt++] = 0xA1;//AX_FIFO_CHUNK_DATA;
    fifoBytes[cnt++] = 0x18;//AX_FIFO_TXDATA_UNENC | AX_FIFO_TXDATA_RAW | AX_FIFO_TXDATA_NOCRC;
    fifoBytes[cnt++] = 0xCC;
    fifoBytes[cnt++] = 0xAA;
    fifoBytes[cnt++] = 0xCC;
    fifoBytes[cnt++] = 0xAA;

    /* write commands to fifo to prepare it for the incoming packet */
    fifoBytes[cnt++] = AX_FIFO_CHUNK_DATA; // fifo data command
    fifoBytes[cnt++] = length + 1; // length byte, +1 to include control field byte
    fifoBytes[cnt++] = AX_FIFO_TXDATA_PKTSTART | AX_FIFO_TXDATA_PKTEND; // control field byte
    
    memcpy(fifoBytes+cnt, txPacket, length);
    AX_WriteFifo(fifoBytes, cnt+length);

    /* commit packet to the FIFO for tx */
    AX_Write8(AX_REG_FIFOSTAT, AX_FIFOCMD_COMMIT);
    
    /* enable Tx interrupts */
    // TODO

    return 1;
}

void AX_WriteFifo(uint8_t* buffer, uint8_t length) {
    // One SPI transaction for each byte in the buffer, as per easyax example code
    uint8_t i;
    for (i = 0; i < length; i++) {
        AX_Write8(AX_REG_FIFODATA, buffer[i]);
    }
}

void AX_EnableOscillator(void) {
    AX_Write8(AX_REG_PINFUNCPWRAMP, 0x07); // 0x~0 for internal oscillator use, 0x~7 to enable TCXO
    AX_Write8(AX_REG_0xF10, 0x04); // set to 0x04 for TCXO, 0x0D if frequency of TCXO or internal osc is > 43MHz, or 0x03 otherwise
    AX_Write8(AX_REG_0xF11, 0x00); // set to 0x00 for TCXO, 0x07 if crystal is used
}

void AX_DisableOscillator(void) {
    AX_Write8(AX_REG_PINFUNCPWRAMP, 0x00); // what to do to disable? this single line doesn't quite do it
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

void AX_WriteLong16(uint16_t reg, uint16_t value) {
    uint8_t buf[4];
    
    buf[0] = (((uint8_t) (reg >> 8) & 0x007F) | 0xF0); // set top four bits for write operation and to follow along with programming manual protocol
    buf[1] = (uint8_t) (reg & 0x00FF);
    buf[2] = (uint8_t) (value >> 8);
    buf[3] = (uint8_t) (value >> 0);
    AX_SPI_Transfer(buf, 4);

    AXStatus = ((uint16_t) (buf[0] << 8)) | buf[1];
}

void AX_Write16(uint16_t reg, uint16_t value) {
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
}

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

void AX_Write24(uint16_t reg, uint32_t value) {
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
}

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

void AX_Write32(uint16_t reg, uint32_t value) {
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

bool IsSPIBusy(void) {
    // return true if the SPI SS line is active (LOW)
    return (SPI1_SS_Get() == false);
}

void AX_SPI_Transfer(unsigned char* data, uint8_t length) {
    static unsigned char dataToReceive[256];
    SPI1_WriteRead(data, length, dataToReceive, length);
    memcpy(data, dataToReceive, length);
    while (IsSPIBusy()) {};
}

// ported over from ax radio's crazy library.
uint8_t AX_AdjustVcoi(uint8_t rng) {
    uint8_t offs;
	uint8_t bestrng;
	uint16_t bestval = (uint16_t)~0;
	rng &= 0x7F;
	bestrng = rng;
	for (offs = 0; offs != 16; ++offs) {
		uint16_t val;
		if (!((uint8_t)(rng + offs) & 0xC0)) {
			AX_Write8(AX_REG_PLLVCOI, (0x80 | (rng + offs)));
			val = AX_TuneVoltage();
			if (val < bestval) {
				bestval = val;
				bestrng = rng + offs;
			}
		}
		if (!offs)
			continue;
		if (!((uint8_t)(rng - offs) & 0xC0)) {
			AX_Write8(AX_REG_PLLVCOI, (0x80 | (rng - offs)));
			val = AX_TuneVoltage();
			if (val < bestval) {
				bestval = val;
				bestrng = rng - offs;
			}
		}
	}
	// if we hit the lower rail, do not change anything
	if (bestval <= 0x0010)
		return rng | 0x80;
	return bestrng | 0x80;
}

// ported over from ax radio's crazy library.
int16_t AX_TuneVoltage(void) {
    int16_t r = 0;
    uint8_t cnt = 64;
    do {
        AX_Write8(AX_REG_GPADCCTRL, 0x84);
        do {
        } while (AX_Read8(AX_REG_GPADCCTRL) & 0x80);
    } while (--cnt);
    cnt = 32;
    do {
        AX_Write8(AX_REG_GPADCCTRL, 0x84);
        do {
        } while (AX_Read8(AX_REG_GPADCCTRL) & 0x80);
        {
            int16_t x = AX_Read8(AX_REG_GPADC13VALUE1) & 0x03;
            x <<= 8;
            x |= AX_Read8(AX_REG_GPADC13VALUE0);
            r += x;
        }
    } while (--cnt);
    return r;
}
// **** END MODULE FUNCTION IMPLEMENTATIONS ****
