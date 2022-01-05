/* 
 * File:   MH_AX.c
 * Author: Kodiak North
 * 
 * Description: Masterhaul's implementation of a library to interface with
 * OnSemiconductor's cumbersome AX-5043/5243 transceivers
 * 
 * Initialize one module as a Primary Transmitter (PTX), and the other as a
 * Primary Receiver (PRX).
 *
 * Created on October 28, 2021, 1:13 PM
 */

// **** MODULE INCLUDE DIRECTIVES ****
#include <string.h>

#include "AX.h"
#include "AX_FifoCommands.h"
#include "AX_PacketQueue.h"
#include "AX_Registers.h"
#include "AX_RegisterAPI.h"
#include "AX_RegisterValues.h"
#include "Print.h"

#include "peripheral/gpio/plib_gpio.h"
#include "Time.h"
#include "peripheral/evic/plib_evic.h"
// **** END MODULE INCLUDE DIRECTIVES ****
// -----------------------------------------------------------------------------
// **** MODULE MACROS ****
#define _DEBUG
#ifdef _DEBUG
#define debug Print_EnqueueMsg
#else
#define debug(...)
#endif

#define DEV_KIT_TX
// **** END MODULE MACROS ****
// -----------------------------------------------------------------------------
// **** MODULE TYPEDEFS ****
typedef enum {
    IDLE = 0,
    WAIT_FIFO_POWER_GOOD,
    WAIT_XTAL,
    WAIT_TX_COMPLETE,
    WAIT_RECEIVE,
} ax_tx_state_t;

typedef enum {
    PTX_TRANSMIT = 0x5458,
    PRX_ACK = 0xAC4B,
} ax_packet_identifier_t;

typedef struct {
    uint8_t Size; // size of packet metadata (Size, RxAddr, Identifier) + size of relevant bytes in Payload buffer
    uint32_t RxAddr;
    ax_packet_identifier_t Identifier;
    uint8_t Payload[AX_PAYLOAD_MAX_SIZE];
} ax_tx_packet_t;
// **** END MODULE TYPEDEFS ****
// -----------------------------------------------------------------------------
// **** MODULE GLOBAL VARIABLES ****
ax_mode_t G_Mode = AX_MODE_NONE;

// PTX
uint32_t G_PtxAckTimeoutStartTimeMs = 0;

// PRX

// common
volatile ax_tx_state_t G_CommState;
volatile bool G_FifoPowerGood = false;
volatile bool G_XtalReady = false;
volatile bool G_TxComplete = false;
volatile bool G_DataAvailable = false;

// parameters
uint8_t G_ChannelVcoi = 0; // access this variable when ax crazy library calls axradio_get_pllvcoi function
uint8_t G_PllRangingA = 0;
// **** END MODULE GLOBAL VARIABLES ****
// -----------------------------------------------------------------------------
// **** MODULE FUNCTION PROTOTYPES ****
// interface with chip
void AX_EnableOscillator(void);
void AX_WritePacketToFifo(uint8_t* txPacket, uint8_t length);

// IRQ handlers
void AX_InterruptHandler(GPIO_PIN pin, uintptr_t context);

// register initialization
void AX_InitConfigRegisters(void);
void AX_InitTxRegisters(void);
void AX_InitRxRegisters(void);
void AX_PrepareForModeSwitch(void);

// chip calibration / tuning
uint8_t AX_AdjustVcoi(uint8_t rng);
int16_t AX_TuneVoltage(void);
// **** END MODULE FUNCTION PROTOTYPES ****
// -----------------------------------------------------------------------------

uint16_t GetState(void) {
    return (uint16_t) G_CommState;
}

void AX_EnableTxRxDoneIRQ(void) {
    /* make sure bits in RADIOEVENTREQ are cleared */
    AX_Read8(AX_REG_RADIOEVENTREQ0);
    /* enable Tx complete interrupt */
    AX_Write8(AX_REG_RADIOEVENTMASK0, AX_REVMDONE);
    AX_Write16(AX_REG_IRQMASK, AX_IRQMRADIOCTRL);   
}

void AX_ClearTxRxDoneIRQ(void) {
    /* clear Tx done interrupt masks */
    AX_Read8(AX_REG_RADIOEVENTREQ0);
    AX_Write8(AX_REG_RADIOEVENTMASK0, 0x00);
    AX_Write16(AX_REG_IRQMASK, 0x00);
}

bool AX_Init(ax_mode_t _mode) {
    // reset the chip first
    AX_Write8(AX_REG_PWRMODE, AX_PWRMODE_RST);
    AX_Write8(AX_REG_PWRMODE, AX_PWRMODE_POWERDOWN);
    
    unsigned long start = Time_GetMs();
    while ((Time_GetMs() - start) < 10) {};
    
    G_Mode = _mode;
    uint8_t revision = 0, scratch = 0;
    uint8_t revision2 = 0, scratch2 = 0;
    revision = AX_Read8(AX_REG_SILICONREVISION);
    revision2 = AX_Read8(AX_REG_SILICONREVISION);
    scratch = AX_Read8(AX_REG_SCRATCH);
    scratch2 = AX_Read8(AX_REG_SCRATCH);
    if ((revision2 != AX_SILICONREVISION) || (scratch2 != AX_SCRATCH)) {
        // chip likely dead or not connected
        debug("ax chip not found... revision 0x%02X, scratch 0x%02X\n", revision, scratch);
        debug("ax chip not found... revision 0x%02X, scratch 0x%02X\n", revision2, scratch2);
        return false;
    }
        
    AX_InitConfigRegisters();
    AX_InitTxRegisters();
    
    AX_Write8(AX_REG_PLLLOOP, 0x09);
    AX_Write8(AX_REG_PLLCPI, 0x08);
    AX_Write8(AX_REG_PWRMODE, AX_PWRMODE_XOEN | AX_PWRMODE_REFEN | AX_PWRMODE_STANDBY); // added STANDBY in attempt to get internal crystal running
    AX_Write8(AX_REG_MODULATION, 0x08);
    AX_Write24(AX_REG_FSKDEV, 0x000000);
    /* Wait for oscillator to start running  */
    while (AX_Read8(AX_REG_XTALSTATUS) != 0x01) {};
    
    // write synthesizer frequency A
    AX_Write32(AX_REG_FREQA, 0x1216EEEF); // NOTE: this may need to be changed if crystal frequency is altered
    
    AX_Write8(AX_REG_PLLRANGINGA, 0x1A); // start the auto ranging process with VCO_A range of 10
    // wait for ranging to complete
    do {
        G_PllRangingA = AX_Read8(AX_REG_PLLRANGINGA);
    } while ((G_PllRangingA & AX_PLLRANGING_RNG_START) != 0x00);
    if ((G_PllRangingA & AX_PLLRANGING_RNGERR) == AX_PLLRANGING_RNGERR) {
        // range error, programmed frequency cannot be achieved
        debug("ax ranging failed, PLLRANGING = 0x%02X\n", G_PllRangingA);
        return false;
    }
    
    // VCOI calibration
    AX_InitTxRegisters(); // good idea to re-init Tx registers after ranging
    AX_Write8(AX_REG_PLLLOOP, 0x0D);
    AX_Write8(AX_REG_0xF35, 0x92);
    
    AX_Write8(AX_REG_PWRMODE, AX_PWRMODE_XOEN | AX_PWRMODE_REFEN | AX_PWRMODE_SYNTHTX);
    uint8_t vcoiSave = AX_Read8(AX_REG_PLLVCOI);
    AX_Write8(AX_REG_PLLRANGINGA, G_PllRangingA & 0x0F);
    
    uint8_t i;
    for (i = 0; i < 2; i++) {
        uint8_t x = 0x99;
        x += (G_PllRangingA & 0x0F) - 0x0A;
        G_ChannelVcoi = AX_AdjustVcoi(x);
    }
    
    AX_Write8(AX_REG_PLLVCOI, vcoiSave);
    AX_Write8(AX_REG_PWRMODE, AX_PWRMODE_XOEN | AX_PWRMODE_REFEN | AX_PWRMODE_POWERDOWN);
    AX_InitConfigRegisters(); // must re-init registers after VCOI calibration
    
    /* attach callback and enable interrupt on input IRQ pin */
    GPIO_PinInterruptCallbackRegister(AX_IRQ_PIN_PIN, AX_InterruptHandler, (uintptr_t) NULL);
    AX_IRQ_PIN_InterruptEnable();
    
    if (G_Mode == AX_MODE_PTX) {
        G_CommState = IDLE;
    } else if (G_Mode == AX_MODE_PRX) {
        G_CommState = WAIT_RECEIVE;
        AX_InitRxRegisters();
        /* enable FIFO not empty interrupt */
        AX_Write16(AX_REG_IRQMASK, AX_IRQMFIFONOTEMPTY);
    }
    
    /* init the tx packet queue */
    AX_PacketQueue_Init();
    
    return true;
}

uint32_t G_RxCount = 0;
/* interrupt driven communication */
void AX_CommTask(void) {
    EVIC_SourceDisable(INT_SOURCE_CHANGE_NOTICE_A);
    switch (G_CommState) {
        case IDLE:
            if (!AX_IsQueueEmpty()) {
                G_CommState = WAIT_FIFO_POWER_GOOD;
//                G_CommState = WAIT_XTAL; // now waiting for fifo pow
                
                /* Clear FIFO */
                AX_Write8(AX_REG_FIFOSTAT, AX_FIFOCMD_CLEAR_FIFO_DATA_AND_FLAGS);

                /* Place chip in FULLTX mode with crystal oscillator and reference circuitry enabled */
                AX_Write8(AX_REG_PWRMODE, AX_PWRMODE_XOEN | AX_PWRMODE_REFEN | AX_PWRMODE_FULLTX);

                /* enable the oscillator */
                AX_EnableOscillator();
                
                /* wait for fifo power to be ready */
                AX_Write8(AX_REG_POWIRQMASK, AX_POWIRQ_MPWRGOOD | AX_POWIRQ_MSVMODEM);
                AX_Write16(AX_REG_IRQMASK, AX_IRQMPOWER);
                
                /* enable crystal oscillator interrupt */
//                AX_Write16(AX_REG_IRQMASK, AX_IRQMXTALREADY); // now waiting for fifo pow instead
            }
            break;
        case WAIT_FIFO_POWER_GOOD:
            if (G_FifoPowerGood) {
                G_FifoPowerGood = false;
                
                /* clear fifo power good interrupt masks */
                AX_Write8(AX_REG_POWIRQMASK, 0x00);
                AX_Write16(AX_REG_IRQMASK, 0x00);
                
                SetDebugLEDs(1);
                while(GPIO_PinRead(AX_IRQ_PIN_PIN) != 0) {}; // wait for IRQ pin to deactivate
                
                G_CommState = WAIT_XTAL;
                
                /* enable crystal oscillator interrupt */
                AX_Write16(AX_REG_IRQMASK, AX_IRQMXTALREADY);
            }
            break;
        case WAIT_XTAL:
            if (G_XtalReady) {
                G_XtalReady = false;
                
                /* clear crystal oscillator interrupt mask */
                AX_Write16(AX_REG_IRQMASK, 0x00);
                
                /* write packet to the FIFO */
                uint8_t* pTxPacket = NULL, length = 0;
                AX_DequeuePacket(&pTxPacket, &length);
                if (pTxPacket) {
                    AX_WritePacketToFifo(pTxPacket, length);
                    G_CommState = WAIT_TX_COMPLETE;
                } else {
                    // error
                    debug("ERROR DEQUEUEING AX RF PACKET\n");
                }

                /* make sure bits in RADIOEVENTREQ are cleared */
                AX_Read8(AX_REG_RADIOEVENTREQ0);
                /* enable Tx complete interrupt */
                AX_Write8(AX_REG_RADIOEVENTMASK0, AX_REVMDONE);
                AX_Write16(AX_REG_IRQMASK, AX_IRQMRADIOCTRL);

                /* commit packet to the FIFO for tx */
                AX_Write8(AX_REG_FIFOSTAT, AX_FIFOCMD_COMMIT);
            }
            break;
        case WAIT_TX_COMPLETE:
            if (G_TxComplete) {
                G_TxComplete = false;
                
                /* clear Tx done interrupt masks */
                AX_Read8(AX_REG_RADIOEVENTREQ0);
                AX_Write8(AX_REG_RADIOEVENTMASK0, 0x00);
                AX_Write16(AX_REG_IRQMASK, 0x00);
                
                /* switch back into rx mode */
                G_CommState = WAIT_RECEIVE;
                AX_PrepareForModeSwitch();
                AX_InitRxRegisters();
                
                /* enable FIFO not empty interrupt */
                AX_Write16(AX_REG_IRQMASK, AX_IRQMFIFONOTEMPTY);

                if (G_Mode == AX_MODE_PTX) {
                    G_PtxAckTimeoutStartTimeMs = Time_GetMs();
                }
            }
            break;
        case WAIT_RECEIVE:
            if (G_DataAvailable) {
                G_DataAvailable = false;
                G_RxCount++;
                
                /* clear FIFO not empty interrupt mask */
                AX_Write16(AX_REG_IRQMASK, 0x00);
                
                LED1_Toggle();
                // empty fifo
                uint16_t cntRead = 0;
                uint8_t buf[PRINT_BUFFER_SIZE] = {0};
                do {
                    buf[cntRead] = AX_Read8(AX_REG_FIFODATA);
                    cntRead++;
                } while ((AX_Read8(AX_REG_FIFOSTAT) & AX_FIFO_EMPTY) != AX_FIFO_EMPTY);
                Print_EnqueueBuffer(buf, PRINT_BUFFER_SIZE);
                
                /* switch back to Tx mode */
                AX_PrepareForModeSwitch();
                AX_InitTxRegisters();
                G_CommState = IDLE;
                
                if (G_Mode == AX_MODE_PRX) {
                    /* enqueue an ack packet */
                    uint8_t packet[9] = {0x09, 0x32, 0x34, 0x00, 0x00, 0xAA, 0xBB, 0xCC, 0xDD};
                    static uint16_t counter = 0;
                    counter++;
                    packet[3] = (uint8_t) (counter & 0x00FF);
                    packet[4] = (uint8_t) ((counter & 0xFF00) >> 8);
                    AX_EnqueuePacket(packet, 9);
                }
                else {
                    // can remove later
//                    debug("tx complete, ack received, response time %ums\n", Time_GetMs() - G_PtxAckTimeoutStartTimeMs);
                }
            } else if (G_Mode == AX_MODE_PTX) { 
                if ((Time_GetMs() - G_PtxAckTimeoutStartTimeMs) >= AX_PTX_ACK_TIMEOUT_MS) {
                    /* switch back to Tx mode */
                    AX_PrepareForModeSwitch();
                    AX_InitTxRegisters();
                    G_CommState = IDLE;
//                    debug("timeout waiting for ack\n");
                    
                    // TODO: resend any commands and relevant data
                }
            }
            break;
    }
    EVIC_SourceEnable(INT_SOURCE_CHANGE_NOTICE_A);
}

void SetDebugLEDs(uint8_t val) {
    if (val > 15) {
        debug("SetDebugLEDs ERROR val passed is too large\n");
        return;
    }
    GPIO_PortWrite(GPIO_PORT_K, 0xF0, (val << 4));
}

/* blocking transmit */
int AX_TransmitPacket(uint8_t* txPacket, uint8_t length) {
    uint8_t returnVal = 0;

    /* Clear FIFO */
    AX_Write8(AX_REG_FIFOSTAT, AX_FIFOCMD_CLEAR_FIFO_DATA_AND_FLAGS);

    /* Place chip in FULLTX mode with crystal oscillator and reference circuitry enabled */
    AX_Write8(AX_REG_PWRMODE, AX_PWRMODE_XOEN | AX_PWRMODE_REFEN | AX_PWRMODE_FULLTX);

    /* enable the oscillator */
    AX_EnableOscillator(); // might not need this because we never power down

    /* Wait for FIFO power ready */
    while ((AX_Read8(AX_REG_POWSTAT) & AX_POWSTAT_SVMODEM) != AX_POWSTAT_SVMODEM) {};

    /* write preamble and packet to FIFO */
    AX_WritePacketToFifo(txPacket, length);
    /* Wait for oscillator to start running  */
    while (AX_Read8(AX_REG_XTALSTATUS) != 0x01) {};

    /* commit packet to the FIFO for tx */
    AX_Write8(AX_REG_FIFOSTAT, AX_FIFOCMD_COMMIT);

    /* wait for tx to complete */
    while (AX_Read8(AX_REG_RADIOSTATE) != 0x00) {};
    returnVal = 1;
    
    if (G_Mode == AX_MODE_PTX) {
        G_PtxAckTimeoutStartTimeMs = Time_GetMs();
        // switch into Rx mode
        AX_PrepareForModeSwitch();
        AX_InitRxRegisters();

        SetDebugLEDs(5);
        while (1) {
            uint8_t fifostat = AX_Read8(AX_REG_FIFOSTAT);
            if ((fifostat & AX_FIFO_EMPTY) == 0) {
                // fifo is not empty
                LED1_Toggle();
                // empty fifo
                uint16_t cntRead = 0;
                uint8_t buf[PRINT_BUFFER_SIZE] = {0};
                do {
                    buf[cntRead] = AX_Read8(AX_REG_FIFODATA);
                    cntRead++;
                } while ((AX_Read8(AX_REG_FIFOSTAT) & AX_FIFO_EMPTY) != AX_FIFO_EMPTY);
                Print_EnqueueBuffer(buf, PRINT_BUFFER_SIZE);
                
                static int i = 1;
                Print_EnqueueMsg("transmit success %u, response time %ums\n", i++, (Time_GetMs() - G_PtxAckTimeoutStartTimeMs));
                break;
            } else if (G_Mode == AX_MODE_PTX) {
                if ((Time_GetMs() - G_PtxAckTimeoutStartTimeMs) >= AX_PTX_ACK_TIMEOUT_MS) {
                    debug("timeout waiting for ack\n");
                    break;
                }
            }
        }

        // switch back into Tx mode
        AX_PrepareForModeSwitch();
        AX_InitTxRegisters();
        AX_Write8(AX_REG_PWRMODE, AX_PWRMODE_XOEN | AX_PWRMODE_REFEN | AX_PWRMODE_POWERDOWN);
    }
    
    return returnVal;
}

/* poll for data available, then blocking switch to Tx mode and transmit ACK */
void AX_Receive(void) {
    uint16_t stat;
    stat = AX_Read8(AX_REG_FIFOSTAT);
    if ((stat & 0x01) == 0) {
        // fifo is not empty
        LED1_Toggle();
        G_RxCount++;
        // empty fifo
        uint16_t cntRead = 0;
        
        uint8_t buf[PRINT_BUFFER_SIZE];
        SetDebugLEDs(1);        
        do {
            buf[cntRead] = AX_Read8(AX_REG_FIFODATA);
            cntRead++;
        } while (((AX_Read8(AX_REG_FIFOSTAT) & AX_FIFO_EMPTY) != AX_FIFO_EMPTY) || (cntRead >= PRINT_BUFFER_SIZE));
        Print_EnqueueBuffer(buf, PRINT_BUFFER_SIZE);
        
        if (cntRead > 12) {
            debug("EXTRA LARGE PACKET RECEIVED %u\n", cntRead);
        }
        
        SetDebugLEDs(2);
        // switch into Tx mode
        AX_PrepareForModeSwitch();
        AX_InitTxRegisters();

        // write an ack packet
        uint8_t packet[9] = {0x09, 0x32, 0x34, 0x00, 0x00, 0xAA, 0xBB, 0xCC, 0xDD};
        static uint16_t counter = 0;
        counter++;
        packet[3] = (uint8_t) (counter & 0x00FF);
        packet[4] = (uint8_t) ((counter & 0xFF00) >> 8);
        SetDebugLEDs(3);
        AX_TransmitPacket(packet, 9);

        SetDebugLEDs(4);
        // switch back into Rx mode
        AX_PrepareForModeSwitch();
        AX_InitRxRegisters();
    }
}

// **** MODULE FUNCTION IMPLEMENTATIONS ****
void AX_EnableOscillator(void) {
    AX_Write8(AX_REG_PINFUNCPWRAMP, PINFUNCPWRAMP_VAL);
    AX_Write8(AX_REG_0xF10, F10_VAL);
    AX_Write8(AX_REG_0xF11, F11_VAL);
}

void AX_WritePacketToFifo(uint8_t* txPacket, uint8_t length) { 
    uint8_t fifoBytes[AX_FIFO_MAX_SIZE] = {0}, cnt = 0;

    /* write preamble MATCHPAT1 bits */ 
    fifoBytes[cnt++] = AX_FIFO_CHUNK_REPEATDATA;
    fifoBytes[cnt++] = AX_FIFO_TXDATA_UNENC | AX_FIFO_TXDATA_RAW | AX_FIFO_TXDATA_NOCRC;
    fifoBytes[cnt++] = 4; // preamble length of 4 bytes
    fifoBytes[cnt++] = 0x55; // preamble value, sent preamble length times
    
    /* write preamble MATCHPAT0 bits */
    fifoBytes[cnt++] = AX_FIFO_CHUNK_DATA;
    fifoBytes[cnt++] = 5; // preamble length of 4 bytes + fifo command flag byte
    fifoBytes[cnt++] = AX_FIFO_TXDATA_UNENC | AX_FIFO_TXDATA_RAW | AX_FIFO_TXDATA_NOCRC;
#ifdef DEV_KIT_TX
    fifoBytes[cnt++] = 0xCC;
#else
    fifoBytes[cnt++] = 0x55;
#endif
    fifoBytes[cnt++] = 0xAA;
#ifdef DEV_KIT_TX
    fifoBytes[cnt++] = 0xCC;
#else
    fifoBytes[cnt++] = 0x55;
#endif
    fifoBytes[cnt++] = 0xAA;

    /* write commands to fifo to prepare it for the incoming packet */
    fifoBytes[cnt++] = AX_FIFO_CHUNK_DATA; // fifo data command
    fifoBytes[cnt++] = length + 1; // length byte, +1 to include control field byte
    fifoBytes[cnt++] = AX_FIFO_TXDATA_PKTSTART | AX_FIFO_TXDATA_PKTEND; // control field byte
    
    memcpy(fifoBytes+cnt, txPacket, length);
    AX_WriteFifo(fifoBytes, cnt+length);
}

void AX_InterruptHandler(GPIO_PIN pin, uintptr_t context) {
    if (GPIO_PinRead(pin) == 1) {
        switch (G_CommState) {
            case IDLE:
                break;
            case WAIT_FIFO_POWER_GOOD:
                G_FifoPowerGood = true;
                break;
            case WAIT_XTAL:
                G_XtalReady = true;
                break;
            case WAIT_TX_COMPLETE:
                G_TxComplete = true;
                break;
            case WAIT_RECEIVE:
                G_DataAvailable = true;
                break;
        }
    }
}

void AX_InitConfigRegisters(void) {
    AX_Write8(AX_REG_POWIRQMASK, 0x00); // power related interrupts
    AX_Write16(AX_REG_IRQMASK, 0x0000);
    AX_Write16(AX_REG_RADIOEVENTMASK, 0x00); // enable more interrupts with this register
    AX_Write8(AX_REG_MODULATION, 0x08); // FSK modulation
    AX_Write8(AX_REG_ENCODING, 0x00);
    AX_Write8(AX_REG_FRAMING, 0x26);
    AX_Write8(AX_REG_PINFUNCSYSCLK, 0x04); // set to output frequency of oscillator used (either internal or TCXO)
    AX_Write8(AX_REG_PINFUNCDCLK, 0x00); // set to output '0'
    AX_Write8(AX_REG_PINFUNCDATA, 0x00); // set to output '0'
    AX_Write8(AX_REG_PINFUNCIRQ, 0x03); // 0x00 to output '0', 0x03 to enable IRQ; no inversion, no pullup
    AX_Write8(AX_REG_PINFUNCANTSEL, 0x00); // set to output '0'
    AX_Write8(AX_REG_PINFUNCPWRAMP, PINFUNCPWRAMP_VAL);
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
    if (G_Mode == AX_MODE_PTX) {
        AX_Write8(AX_REG_PKTMAXLEN, PTX_PRX_TEST_PACKET_SIZE); // PKTMAXLEN includes 3 byte FIFODATA command + data packet length
        AX_Write32(AX_REG_PKTADDR, PTX_PKTADDR);
    } else if (G_Mode == AX_MODE_PRX) {
        AX_Write8(AX_REG_PKTMAXLEN, PTX_PRX_TEST_PACKET_SIZE); // PKTMAXLEN includes 3 byte FIFODATA command + data packet length
        AX_Write32(AX_REG_PKTADDR, PRX_PKTADDR);
    }
    AX_Write32(AX_REG_PKTADDRMASK, 0x0000FFFF);
#ifdef DEV_KIT_TX
    AX_Write32(AX_REG_MATCH0PAT, 0xAACCAACC); // this is preamble0 that Rx uses to detect/accept an incoming packet
#else
    AX_Write32(AX_REG_MATCH0PAT, 0xAA55AA55); // this is preamble0 that Rx uses to detect/accept an incoming packet
#endif
    AX_Write8(AX_REG_MATCH0LEN, 0x1F);
    AX_Write8(AX_REG_MATCH0MAX, 0x1F); // this says at least 31 bits of match0 pattern must match
    AX_Write16(AX_REG_MATCH1PAT, 0x5555); // this is preamble1 that Rx uses to detect/accept an incoming packet
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
    AX_Write8(AX_REG_PKTSTOREFLAGS, 0x00);
    AX_Write8(AX_REG_PKTACCEPTFLAGS, 0x20); // accept packets that span multiple FIFO chunks
    AX_Write16(AX_REG_DACVALUE, 0x0000);
    AX_Write8(AX_REG_DACCONFIG, 0x00);
    AX_Write8(AX_REG_REF, 0x03);
    AX_Write8(AX_REG_0xF10, F10_VAL);
    AX_Write8(AX_REG_0xF11, F11_VAL);
    AX_Write8(AX_REG_0xF1C, 0x07);
    AX_Write8(AX_REG_0xF21, 0x68);
    AX_Write8(AX_REG_0xF22, 0xFF);
    AX_Write8(AX_REG_0xF23, 0x84);
    AX_Write8(AX_REG_0xF26, 0x98);
    AX_Write8(AX_REG_0xF34, 0x08);
    AX_Write8(AX_REG_0xF35, F35_VAL);
    AX_Write8(AX_REG_0xF44, 0x25);
}

uint8_t AX_InitCommonRegisters(void) {
    uint8_t rng = G_PllRangingA;
    if (rng & 0x20)
        return 0x06; // AXRADIO_ERR_RANGING;
    if (AX_Read8(AX_REG_PLLLOOP) & 0x80)
        AX_Write8(AX_REG_PLLRANGINGB, (rng & 0x0F));
    else
        AX_Write8(AX_REG_PLLRANGINGA, (rng & 0x0F));
    rng = G_ChannelVcoi;
    if (rng & 0x80)
        AX_Write8(AX_REG_PLLVCOI, rng);
    return 0x00; // AXRADIO_ERR_NOERROR;
}

void AX_InitTxRegisters(void) {
    AX_Write8(AX_REG_PLLLOOP, 0x07);
    AX_Write8(AX_REG_PLLCPI, 0x12);
    AX_Write8(AX_REG_PLLVCODIV, PLLVCODIV_VAL);
    AX_Write8(AX_REG_XTALCAP, XTALCAP_VAL);
    AX_Write8(AX_REG_0xF00, 0x0F);
    AX_Write8(AX_REG_0xF18, 0x06);
    AX_InitCommonRegisters();
}

void AX_InitRxRegisters(void) {
    AX_Write8(AX_REG_PLLLOOP, 0x07);
    AX_Write8(AX_REG_PLLCPI, 0x08);
    AX_Write8(AX_REG_PLLVCODIV, PLLVCODIV_VAL);
    AX_Write8(AX_REG_XTALCAP, XTALCAP_VAL);
    AX_Write8(AX_REG_0xF00, 0x0F);
    AX_Write8(AX_REG_0xF18, 0x06);
    AX_InitCommonRegisters();

    // rx on continuous
    AX_Write8(AX_REG_RSSIREFERENCE, 0x36); // set offset for the computed RSSI
    AX_Write8(AX_REG_TMGRXAGC, 0x00); // AGC settling time
    AX_Write8(AX_REG_TMGRXPREAMBLE1, 0x00); // preamble1 timeout - preamble 1 corresponds to MATCH1PAT
    AX_Write8(AX_REG_TMGRXPREAMBLE2, 0x00); // preamble2 timeout - preamble 2 corresponds to MATCH1PAT
    AX_Write8(AX_REG_TMGRXPREAMBLE3, 0x00); // preamble3 timeout - preamble 3 corresponds to MATCH0PAT
    AX_Write8(AX_REG_PKTMISCFLAGS, 0x00); // disable all miscellaneous packet flags

    AX_Write8(AX_REG_FIFOSTAT, AX_FIFOCMD_CLEAR_FIFO_DATA_AND_FLAGS); // clear FIFO
    AX_Write8(AX_REG_PWRMODE, AX_PWRMODE_XOEN | AX_PWRMODE_REFEN | AX_PWRMODE_FULLRX); // set power mode to FULLRX
    AX_EnableOscillator();
}

void AX_PrepareForModeSwitch(void) {
    AX_Write8(AX_REG_FIFOSTAT, AX_FIFOCMD_CLEAR_FIFO_DATA_AND_FLAGS);
    AX_Write16(AX_REG_IRQMASK, 0x0000);
    AX_Write8(AX_REG_PWRMODE, AX_PWRMODE_XOEN | AX_PWRMODE_REFEN | AX_PWRMODE_STANDBY);
    SetDebugLEDs(0xA);
    while ((AX_Read8(AX_REG_POWSTAT) & AX_POWSTAT_SVMODEM) == AX_POWSTAT_SVMODEM) {}; // wait for svmodem bit to be cleared
    AX_Write8(AX_REG_PWRMODE, AX_PWRMODE_XOEN | AX_PWRMODE_REFEN | AX_PWRMODE_FIFOON);
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