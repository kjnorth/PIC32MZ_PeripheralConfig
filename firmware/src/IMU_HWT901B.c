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
void IMU_Config(uint8_t hexAddress, uint8_t hexDataL, uint8_t hexDataH);
void IMU_Calibrate_AccelGyro(void);
void IMU_ParseData(void);
void IMU_WriteUnlockPacket(void);
void IMU_WriteConfigPacket(uint8_t hexAddress, uint8_t hexDataL, uint8_t hexDataH);
void IMU_WriteSavePacket(void);
static void DelayXms(uint16_t x_ms);
static void UART1_WriteCallback(uintptr_t context);
static void UART1_ReadCallback(uintptr_t context);
// **** MODULE FUNCTION PROTOTYPES ****
// -----------------------------------------------------------------------------

void IMU_Init(void) {
    Uart1.IsRxErrorDetected = false;
    Uart1.IsRxFinished = false;
    Uart1.IsTxFinished = false;
    Uart1.Errors = UART_ERROR_NONE;
    UART1_WriteCallbackRegister(UART1_WriteCallback, (uintptr_t) NULL);
    UART1_ReadCallbackRegister(UART1_ReadCallback, (uintptr_t) NULL);
    
    // Init config packet - Head1/2 are the same for all config cmds
    ConfigPacket.Head1 = 0xFF;
    ConfigPacket.Head2 = 0xAA;
    // Apply IMU config settings
    IMU_Config(0x03, 0x07, 0x00); // set 20Hz output data rate
    
    UART1_Read(RxBuffer, sizeof (uint8_t));
    ImuState = WAIT_START;
}

/*
 * What settings do I want configurable from the PIC?
 * calibrate for Gyro and Accel - done
 * gyro auto calibration - what does it do??
 * return rate - done
 * 
 * INSTALL DIRECTION NOTES
 */

/**
 * This function should only be called from IMU_Init after the UART registers
 * have been initialized
 * @param address - address of register to be configured
 * @param dataL - config packet low data byte from datasheet
 * @param dataH - config packet high data byte from datasheet
 */
void IMU_Config(uint8_t hexAddress, uint8_t hexDataL, uint8_t hexDataH) {
    // 1. unlock device
    IMU_WriteUnlockPacket();
    // 2. apply the setting
    IMU_WriteConfigPacket(hexAddress, hexDataL, hexDataH);
    // 3. save
    IMU_WriteSavePacket();
}

/**
 * Only calibrate accel and gyro when IMU is upright (white/blue label facing
 * up) on perfectly level surface with no external disturbances such as walking
 * or jumping nearby.
 * @note calibrate accel/gyro must delay 3-5s while calibration is completing,
 * so this function is unique from IMU_Config above
 */
void IMU_Calibrate_AccelGyro(void) {
    // 1. unlock device
    IMU_WriteUnlockPacket();
    // 2. apply the setting
    IMU_WriteConfigPacket(0x01, 0x01, 0x00);
    // 3. delay 3-5s
    DelayXms(5000);
    // 4. save
    IMU_WriteSavePacket(); 
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

#define PL_IMU

float IMU_RollGet(void) {
#if defined(PL_IMU)
    float returnVal = 0.0;
    float rollData = AngleData[0];
    if (rollData < 0.0) {
        returnVal = rollData + 180.0;
    } else {
        returnVal = rollData - 180.0;
    }
    return returnVal;
#elif defined(BR_IMU)
    return 0.0;
#else
    return 0.0;
#endif
}

float IMU_PitchGet(void) {
#if defined(PL_IMU)
    return -AngleData[1];
#elif defined(BR_IMU)
    return 0.0;
#else
    return 0.0;
#endif
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
    DelayXms(250);
}

void IMU_WriteConfigPacket(uint8_t hexAddress, uint8_t hexDataL, uint8_t hexDataH) {
    ConfigPacket.Address = hexAddress;
    ConfigPacket.DataL = hexDataL;
    ConfigPacket.DataH = hexDataH;
    memcpy(TxBuffer, (uint8_t*) &ConfigPacket, CONFIG_PACKET_LENGTH);
    UART1_Write(TxBuffer, CONFIG_PACKET_LENGTH);
    DelayXms(250);
}

void IMU_WriteSavePacket(void) {
    ConfigPacket.Address = 0x00;
    ConfigPacket.DataL = 0x00;
    ConfigPacket.DataH = 0x00;
    memcpy(TxBuffer, (uint8_t*) &ConfigPacket, CONFIG_PACKET_LENGTH);
    UART1_Write(TxBuffer, CONFIG_PACKET_LENGTH);
    DelayXms(250);
}

static void DelayXms(uint16_t x_ms) {
    uint32_t curTime = Time_GetMs();
    uint32_t preTime = curTime;
    while (curTime - preTime <= x_ms) {
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