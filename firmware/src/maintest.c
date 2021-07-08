/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file contains the "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/

/*******************************************************************************
 * Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
 *
 * Subject to your compliance with these terms, you may use Microchip software
 * and any derivatives exclusively with Microchip products. It is your
 * responsibility to comply with third party license terms applicable to your
 * use of third party software (including open source software) that may
 * accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
 * ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include <string.h>
#include "definitions.h"                // SYS function prototypes

#define LED_On()                        LED1_Set()
#define LED_Off()                       LED1_Clear()

char messageError[] = "**** UART error occurred ****\n";

bool isRxErrorDetected = false;
bool isTxFinished = false;
bool isRxFinished = true; // on init, device considered to be done receiving data

void APP_WriteCallback(uintptr_t context) {
    isTxFinished = true;
}

void APP_ReadCallback(uintptr_t context) {
    if (UART5_ErrorGet() != UART_ERROR_NONE) {
        /* ErrorGet clears errors, set error flag to notify console */
        isRxErrorDetected = true;
    } else {
        isRxFinished = true;
    }
}

#define START 0xA5u
#define ACK 0xF9u

//typedef enum {
//    SEND_START,
//    WAIT_ACK_START_SEND_MSG,
//    WAIT_ACK_MSG,
//} uart_comm_t;
//uart_comm_t commState = SEND_START;

typedef enum {
    SEND_START,
    SEND_MSG,
    SEND_START_VERIFY_ACK,
} uart_send_t;
uart_send_t sendState = SEND_START;

void UartComm(void) {
    static uint8_t response = 0;
    //    char msg[] = "Hello\n";
    //    uint8_t sizeMsg = sizeof (msg);

    /** SW breakdown:
     * 1: state = S_S,       isTxFinished = false, isRxFinished = true,  initiate send of data
     * 2: state = W_A_S_S_M, isTxFinished = true,  isRxFinished = false, initiate receive of ack
     * 2/ state = W_A_S_S_M, isTxFinished = false, isRxFinished = false, wait for receive of ack
     * 2//state = W_A_S_S_M, isTxFinished = false, isRxFinished = true, 
     * 3: state = W_A_S_S_M, isTxFinished = false, isRxFinished = true,  send data
     * 4: state = W_A_M,     isTxFinished = true,  isRxFinished = false, receive data
     * 5: state = W_A_M,     isTxFinished = false, isRxFinished = true,  send data
     */

    /*
    switch (commState) {
        case SEND_START:
            byte = START; // what would happen if byte was a local variable to the scope of this case? would the pointer point to null when scope is exited?
            nextWrite = (char*) &byte;
            sizeNextWrite = 1;
            commState = WAIT_ACK_START_SEND_MSG;
            break;
        case WAIT_ACK_START_SEND_MSG:
            if (response == ACK) {
                response = 0;
                nextWrite = (char*) &msg; // TRY USING THE MEMCPY FUNCTION HERE
                sizeNextWrite = sizeMsg;
                commState = WAIT_ACK_MSG;
            }
            break;
        case WAIT_ACK_MSG:
            if (response == ACK) {
                response = 0;
                byte = START;
                nextWrite = (char*) &byte;
                sizeNextWrite = 1;
                commState = WAIT_ACK_START_SEND_MSG;
            }
            break;
    } */

    if (isRxErrorDetected == true) {
        /* Send error message to console */
        isRxErrorDetected = false;
        UART5_Write(&messageError, sizeof (messageError));
    } else if (isRxFinished) {
        /* send start byte or msg or toggle LED if ack not received correctly */
        isRxFinished = false;

//        char filler[255] = {0};
        char msg[] = "Hello\n";
        char* nextWrite = (char*) &msg;//&filler;
        uint8_t sizeNextWrite = 0;
        uint8_t start = START; // once working, make this a local var and see what happens


        switch (sendState) {
            case SEND_START:
            {
                //                uint8_t start = START;
                nextWrite = (char*) &start;
                sizeNextWrite = 1;
                sendState = SEND_MSG;
                break;
            }
            case SEND_MSG:
                if (response == ACK) {
                    nextWrite = (char*) &msg;
                    sizeNextWrite = 7;//sizeof (msg);
                    sendState = SEND_START_VERIFY_ACK;
                } else {
                    LED_On();
//                    LED1_Toggle();
                }
                break;
            case SEND_START_VERIFY_ACK:
                if (response == ACK) {
                    nextWrite = (char*) &start;
                    sizeNextWrite = 1;
                    sendState = SEND_MSG;
                } else {
//                    LED1_Toggle();
                }
                break;
        }

        UART5_Write(nextWrite, sizeNextWrite);
    } else if (isTxFinished == true) {
        /* initiate read of ack */
        isTxFinished = false;
        UART5_Read(&response, 1);
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

int main(void) {
    /* Initialize all modules */
    SYS_Initialize(NULL);
    LED_Off();

    /* Register callback functions and send start message */
    UART5_WriteCallbackRegister(APP_WriteCallback, 0);
    UART5_ReadCallbackRegister(APP_ReadCallback, 0);

    //    static uint8_t byte = 0;
    //    char temp[] = "temp\n";
    //    char* nextWrite = NULL;
    //    static uint8_t sizeNextWrite = 0;
    //    isTxFinished = true;
    //    bool flip = true;

    while (true) {
        UartComm();

        /* this code works, it blasts the arduino with data
        if (flip) {
            byte = START;
            nextWrite = (char*) &byte;
            sizeNextWrite = 1;
        } else {
            nextWrite = (char*) &temp;
            sizeNextWrite = sizeof (temp);
        }

        flip = !flip;
        UART5_Write(nextWrite, sizeNextWrite);
         * */

    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE);
}


/*******************************************************************************
 End of File
 */

