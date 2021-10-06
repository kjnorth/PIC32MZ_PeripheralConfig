/* 
 * File:   IMU_HWT901B.c
 * Author: Kodiak North
 * 
 * Description:
 *
 * Created on September 13, 2021, 3:09 PM
 */

// **** MODULE INCLUDE DIRECTIVES ****
#include "IMU_HWT901B.h"

#include "peripheral/evic/plib_evic.h"
#include "peripheral/uart/plib_uart1.h"
#include "Time.h"
#include "UART_Common.h"
// **** END MODULE INCLUDE DIRECTIVES ****
// -----------------------------------------------------------------------------
// **** MODULE MACROS ****
#define DATA_START_BYTE (0x55u)
#define DATA_LENGTH (10u) // bytes
#define CONFIG_PACKET_LENGTH (5u) // bytes

// indices in RxBuffer
#define PACKET_ID_IDX (0u)

#define PACKET_X_LOW_IDX (1u)
#define PACKET_X_HIGH_IDX (2u)
#define PACKET_Y_LOW_IDX (3u)
#define PACKET_Y_HIGH_IDX (4u)
#define PACKET_Z_LOW_IDX (5u)
#define PACKET_Z_HIGH_IDX (6u)

#define PACKET_SUM_IDX (9u)
// **** END MODULE MACROS ****
// -----------------------------------------------------------------------------
// **** MODULE TYPEDEFS ****
typedef enum {
    DATA_TIME = 0x50,
    DATA_ACC,
    DATA_GYRO,
    DATA_ANGLE,
    DATA_MAG,
    DATA_BARO,
    DATA_QUAT,
} data_id_byte_t;

typedef enum {
    WAIT_START,
    WAIT_DATA,
} imu_state_t;

typedef struct {
    uint8_t Head1;
    uint8_t Head2;
    uint8_t Address;
    uint8_t DataL;
    uint8_t DataH;
} imu_config_packet_t;
// **** END MODULE TYPEDEFS ****
// -----------------------------------------------------------------------------
// **** MODULE GLOBAL VARIABLES ****
// data
static float AccData[3]; // in G's
static float GyroData[3]; // in deg/s
static float MagData[3]; // in amps/m [H]
static float AngleData[3]; // in deg

// uart
static imu_state_t ImuState;
static imu_config_packet_t ConfigPacket;
static uart_interrupt_flags_t Uart1;
static uint8_t TxBuffer[CONFIG_PACKET_LENGTH]; // only used for sending config cmds
static uint8_t RxBuffer[DATA_LENGTH];
// **** END MODULE GLOBAL VARIABLES ****
// -----------------------------------------------------------------------------
// **** MODULE FUNCTION PROTOTYPES ****
void IMU_ParseData(void);
void IMU_WriteUnlockPacket(void);
void IMU_WriteConfigPacket(uint8_t hexAddress, uint8_t hexDataL, uint8_t hexDataH);
void IMU_WriteSavePacket(void);
static void Delay250ms(void);
static void UART1_WriteCallback(uintptr_t context);
static void UART1_ReadCallback(uintptr_t context);
// **** MODULE FUNCTION PROTOTYPES ****
// -----------------------------------------------------------------------------

void IMU_Init(void) {
    // Head1/2 are the same for all config cmds
    ConfigPacket.Head1 = 0xFF;
    ConfigPacket.Head2 = 0xAA;
    
    Uart1.IsRxErrorDetected = false;
    Uart1.IsRxFinished = false;
    Uart1.IsTxFinished = false;
    Uart1.Errors = UART_ERROR_NONE;
    UART1_WriteCallbackRegister(UART1_WriteCallback, (uintptr_t) NULL);
    UART1_ReadCallbackRegister(UART1_ReadCallback, (uintptr_t) NULL);
    
    UART1_Read(RxBuffer, sizeof (uint8_t));
    ImuState = WAIT_START;
}

/*
 * What settings do I want configurable from the PIC?
 * calibrate for Gyro and Accel
 * install dir? NOPE need to negate angles based on install dir
 * gyro auto calibration
 * return rate
 * set x, y, z bias? seems like alg only cares about 1g on z-axis
 */
#include "Print.h"

void temp_printCmd(void) {
    uint8_t i;
    for (i = 0; i < CONFIG_PACKET_LENGTH; i++) {
        Print_EnqueueMsg("txBuf[%u]: 0x%02X ", i, TxBuffer[i]);
    }
    Print_EnqueueMsg("\n");
}

void IMU_Config(void /*reg to config?*/) {
    // cancel current read
    UART1_ReadAbort();
    
    // 1. unlock device
    IMU_WriteUnlockPacket();
    // 2. apply the setting
    // TEST RETURN DATA RATE NOW
    IMU_WriteConfigPacket(0x03, 0x09, 0x00);
    // 3. save
    IMU_WriteSavePacket();
    // restart communications
    Uart1.IsRxErrorDetected = false;
    Uart1.IsRxFinished = false;
    Uart1.IsTxFinished = false;
    Uart1.Errors = UART_ERROR_NONE;
    
    // restart communications
    UART1_Read(RxBuffer, sizeof (uint8_t));
    ImuState = WAIT_START;
}

void IMU_Config_Accel(void) {
    // config accel must delay 3-5s while accel config is completing, so this func is special
}

void IMU_SampleTask(void) {
    if (Uart1.IsRxErrorDetected) {
        Uart1.IsRxErrorDetected = false;
        Uart1.IsRxFinished = false;
        UART1_ReadAbort();
        UART1_Read(RxBuffer, sizeof (uint8_t));
        ImuState = WAIT_START;
    } else if (Uart1.IsRxFinished) {
        Uart1.IsRxFinished = false;
        
        switch (ImuState) {
            case WAIT_START:
                // enter state when 1 byte read into RxBuffer
                if (RxBuffer[0] == DATA_START_BYTE) {
                    UART1_Read(RxBuffer, DATA_LENGTH);
                    ImuState = WAIT_DATA;
                } else {
                    UART1_Read(RxBuffer, sizeof (uint8_t));
                }
                break;
            case WAIT_DATA: {
                // enter state when DATA_LENGTH of bytes are read
                numUpdates++;
                // validate checksum
                uint8_t checksum = DATA_START_BYTE;
                uint8_t i;
                for (i = 0; i < (DATA_LENGTH - 1); i++) {
                    checksum += RxBuffer[i];
                }
                if (checksum == RxBuffer[PACKET_SUM_IDX]) {
                    // data is valid
                    IMU_ParseData();
                }
                UART1_Read(RxBuffer, sizeof (uint8_t));
                ImuState = WAIT_START;
                break;
            }
        }
    } else if (Uart1.IsTxFinished) {
        Uart1.IsTxFinished = false;
    }
}

float IMU_RollGet(void) {
    return AngleData[0];
}

float IMU_PitchGet(void) {
    return AngleData[1];
}

// **** MODULE FUNCTIONS ****
void IMU_ParseData(void) {
    data_id_byte_t id = (data_id_byte_t) RxBuffer[PACKET_ID_IDX];
    switch (id) {
        case DATA_TIME: break;
        case DATA_ACC:
            AccData[0] = (float)((int16_t)(RxBuffer[PACKET_X_HIGH_IDX] << 8) | RxBuffer[PACKET_X_LOW_IDX]) / 32768.0 * 16.0;
            AccData[1] = (float)((int16_t)(RxBuffer[PACKET_Y_HIGH_IDX] << 8) | RxBuffer[PACKET_Y_LOW_IDX]) / 32768.0 * 16.0;
            AccData[2] = (float)((int16_t)(RxBuffer[PACKET_Z_HIGH_IDX] << 8) | RxBuffer[PACKET_Z_LOW_IDX]) / 32768.0 * 16.0;
            break;
        case DATA_GYRO:
            GyroData[0] = (float)((int16_t)(RxBuffer[PACKET_X_HIGH_IDX] << 8) | RxBuffer[PACKET_X_LOW_IDX]) / 32768.0 * 2000.0;
            GyroData[1] = (float)((int16_t)(RxBuffer[PACKET_Y_HIGH_IDX] << 8) | RxBuffer[PACKET_Y_LOW_IDX]) / 32768.0 * 2000.0;
            GyroData[2] = (float)((int16_t)(RxBuffer[PACKET_Z_HIGH_IDX] << 8) | RxBuffer[PACKET_Z_LOW_IDX]) / 32768.0 * 2000.0;
            break;
        case DATA_ANGLE:
            AngleData[0] = (float)((int16_t)(RxBuffer[PACKET_X_HIGH_IDX] << 8) | RxBuffer[PACKET_X_LOW_IDX]) / 32768.0 * 180.0;
            AngleData[1] = (float)((int16_t)(RxBuffer[PACKET_Y_HIGH_IDX] << 8) | RxBuffer[PACKET_Y_LOW_IDX]) / 32768.0 * 180.0;
            AngleData[2] = (float)((int16_t)(RxBuffer[PACKET_Z_HIGH_IDX] << 8) | RxBuffer[PACKET_Z_LOW_IDX]) / 32768.0 * 180.0;
            break;
        case DATA_MAG:
            MagData[0] = (float)((int16_t)(RxBuffer[PACKET_X_HIGH_IDX] << 8) | RxBuffer[PACKET_X_LOW_IDX]);
            MagData[1] = (float)((int16_t)(RxBuffer[PACKET_Y_HIGH_IDX] << 8) | RxBuffer[PACKET_Y_LOW_IDX]);
            MagData[2] = (float)((int16_t)(RxBuffer[PACKET_Z_HIGH_IDX] << 8) | RxBuffer[PACKET_Z_LOW_IDX]);
            break;
        case DATA_BARO: break;
        case DATA_QUAT: break;
    }
}

void IMU_WriteUnlockPacket(void) {
    ConfigPacket.Address = 0x69;
    ConfigPacket.DataL = 0x88;
    ConfigPacket.DataH = 0xB5;
    memcpy(TxBuffer, (uint8_t*) &ConfigPacket, CONFIG_PACKET_LENGTH);
    UART1_Write(TxBuffer, CONFIG_PACKET_LENGTH);
    Delay250ms();
}

void IMU_WriteConfigPacket(uint8_t hexAddress, uint8_t hexDataL, uint8_t hexDataH) {
    ConfigPacket.Address = hexAddress;
    ConfigPacket.DataL = hexDataL;
    ConfigPacket.DataH = hexDataH;
    memcpy(TxBuffer, (uint8_t*) &ConfigPacket, CONFIG_PACKET_LENGTH);
    UART1_Write(TxBuffer, CONFIG_PACKET_LENGTH);
    Delay250ms();
}

void IMU_WriteSavePacket(void) {
    ConfigPacket.Address = 0x00;
    ConfigPacket.DataL = 0x00;
    ConfigPacket.DataH = 0x00;
    memcpy(TxBuffer, (uint8_t*) &ConfigPacket, CONFIG_PACKET_LENGTH);
    UART1_Write(TxBuffer, CONFIG_PACKET_LENGTH);
    Delay250ms();
}

static void Delay250ms(void) {
    uint32_t curTime = Time_GetMs();
    uint32_t preTime = curTime;
    while (curTime - preTime <= 250) {
        curTime = Time_GetMs();
    }
}

/**
 * function called when UART1 finishes transmitting data
 */
void UART1_WriteCallback(uintptr_t context) {
    Uart1.IsTxFinished = true;
}

/**
 * function called when UART1 finishes reading data
 */
void UART1_ReadCallback(uintptr_t context) {
    Uart1.Errors = UART1_ErrorGet();
    if (Uart1.Errors != UART_ERROR_NONE) {
        /* ErrorGet clears errors, set error flag to notify console */
        Uart1.IsRxErrorDetected = true;
    } else {
        Uart1.IsRxFinished = true;
    }
}
// **** END MODULE FUNCTIONS ****