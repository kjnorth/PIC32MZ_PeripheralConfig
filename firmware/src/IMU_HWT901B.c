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

#include "peripheral/uart/plib_uart1.h"
#include "UART_Comon.h"
// **** END MODULE INCLUDE DIRECTIVES ****
// -----------------------------------------------------------------------------
// **** MODULE MACROS ****
#define DATA_START_BYTE (0x55u)
#define DATA_LENGTH (10u) // bytes

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
static uart_interrupt_flags_t Uart1;
static uint8_t RxBuffer[DATA_LENGTH];
UART_ERROR Errors;
// **** END MODULE GLOBAL VARIABLES ****
// -----------------------------------------------------------------------------
// **** MODULE FUNCTION PROTOTYPES ****
void IMU_ParseData(void);
// **** MODULE FUNCTION PROTOTYPES ****
// -----------------------------------------------------------------------------

void IMU_Init(void) {
    Uart1.isRxErrorDetected = false;
    Uart1.isRxFinished = false;
    Uart1.isTxFinished = false;
    
    UART1_Read(RxBuffer, sizeof (uint8_t));
    ImuState = WAIT_START;
}

void IMU_Config(void /*reg to config?*/);

void IMU_SampleTask(void) {
    if (Uart1.isRxErrorDetected) {
        Uart1.isRxErrorDetected = false;
        UART1_ReadAbort();
        UART1_Read(RxBuffer, sizeof (uint8_t));
        ImuState = WAIT_START;
    } else if (Uart1.isRxFinished) {
        Uart1.isRxFinished = false;
        
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
                // validate checksum
                uint16_t checksum = DATA_START_BYTE;
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
        
    } else if (Uart1.isTxFinished) {
        Uart1.isTxFinished = false;
    }
}

/*
 static uint16_t AccData[3];
static uint16_t GyroData[3];
static uint16_t MagData[3];
static uint16_t AngleData[3];
 */
void IMU_ParseData(void) {
    data_id_byte_t id = (data_id_byte_t) RxBuffer[PACKET_ID_IDX];
    switch (id) {
        case DATA_TIME: break;
        case DATA_ACC:
            AccData[0] = ((RxBuffer[PACKET_X_HIGH_IDX] << 8) | RxBuffer[PACKET_X_LOW_IDX]) / 32768.0 * 16.0;
            AccData[1] = ((RxBuffer[PACKET_Y_HIGH_IDX] << 8) | RxBuffer[PACKET_Y_LOW_IDX]) / 32768.0 * 16.0;
            AccData[2] = ((RxBuffer[PACKET_Z_HIGH_IDX] << 8) | RxBuffer[PACKET_Z_LOW_IDX]) / 32768.0 * 16.0;
            break;
        case DATA_GYRO:
            GyroData[0] = ((RxBuffer[PACKET_X_HIGH_IDX] << 8) | RxBuffer[PACKET_X_LOW_IDX]) / 32768.0 * 2000.0;
            GyroData[1] = ((RxBuffer[PACKET_Y_HIGH_IDX] << 8) | RxBuffer[PACKET_Y_LOW_IDX]) / 32768.0 * 2000.0;
            GyroData[2] = ((RxBuffer[PACKET_Z_HIGH_IDX] << 8) | RxBuffer[PACKET_Z_LOW_IDX]) / 32768.0 * 2000.0;
            break;
        case DATA_ANGLE:
            AngleData[0] = ((RxBuffer[PACKET_X_HIGH_IDX] << 8) | RxBuffer[PACKET_X_LOW_IDX]) / 32768.0 * 180.0;
            AngleData[1] = ((RxBuffer[PACKET_Y_HIGH_IDX] << 8) | RxBuffer[PACKET_Y_LOW_IDX]) / 32768.0 * 180.0;
            AngleData[2] = ((RxBuffer[PACKET_Z_HIGH_IDX] << 8) | RxBuffer[PACKET_Z_LOW_IDX]) / 32768.0 * 180.0;
            break;
        case DATA_MAG:
            MagData[0] = ((RxBuffer[PACKET_X_HIGH_IDX] << 8) | RxBuffer[PACKET_X_LOW_IDX]);
            MagData[1] = ((RxBuffer[PACKET_Y_HIGH_IDX] << 8) | RxBuffer[PACKET_Y_LOW_IDX]);
            MagData[2] = ((RxBuffer[PACKET_Z_HIGH_IDX] << 8) | RxBuffer[PACKET_Z_LOW_IDX]);
            break;
        case DATA_BARO: break;
        case DATA_QUAT: break;
    }
}