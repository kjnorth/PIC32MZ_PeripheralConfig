/*******************************************************************************
  GPIO PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_gpio.h

  Summary:
    GPIO PLIB Header File

  Description:
    This library provides an interface to control and interact with Parallel
    Input/Output controller (GPIO) module.

*******************************************************************************/

/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
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

#ifndef PLIB_GPIO_H
#define PLIB_GPIO_H

#include <device.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Data types and constants
// *****************************************************************************
// *****************************************************************************


/*** Macros for M1_PH pin ***/
#define M1_PH_Set()               (LATGSET = (1<<15))
#define M1_PH_Clear()             (LATGCLR = (1<<15))
#define M1_PH_Toggle()            (LATGINV= (1<<15))
#define M1_PH_OutputEnable()      (TRISGCLR = (1<<15))
#define M1_PH_InputEnable()       (TRISGSET = (1<<15))
#define M1_PH_Get()               ((PORTG >> 15) & 0x1)
#define M1_PH_PIN                  GPIO_PIN_RG15

/*** Macros for M2_PH pin ***/
#define M2_PH_Set()               (LATASET = (1<<5))
#define M2_PH_Clear()             (LATACLR = (1<<5))
#define M2_PH_Toggle()            (LATAINV= (1<<5))
#define M2_PH_OutputEnable()      (TRISACLR = (1<<5))
#define M2_PH_InputEnable()       (TRISASET = (1<<5))
#define M2_PH_Get()               ((PORTA >> 5) & 0x1)
#define M2_PH_PIN                  GPIO_PIN_RA5

/*** Macros for PWM_M2L pin ***/
#define PWM_M2L_Get()               ((PORTE >> 5) & 0x1)
#define PWM_M2L_PIN                  GPIO_PIN_RE5

/*** Macros for M3_PH pin ***/
#define M3_PH_Set()               (LATESET = (1<<6))
#define M3_PH_Clear()             (LATECLR = (1<<6))
#define M3_PH_Toggle()            (LATEINV= (1<<6))
#define M3_PH_OutputEnable()      (TRISECLR = (1<<6))
#define M3_PH_InputEnable()       (TRISESET = (1<<6))
#define M3_PH_Get()               ((PORTE >> 6) & 0x1)
#define M3_PH_PIN                  GPIO_PIN_RE6

/*** Macros for M4_PH pin ***/
#define M4_PH_Set()               (LATESET = (1<<7))
#define M4_PH_Clear()             (LATECLR = (1<<7))
#define M4_PH_Toggle()            (LATEINV= (1<<7))
#define M4_PH_OutputEnable()      (TRISECLR = (1<<7))
#define M4_PH_InputEnable()       (TRISESET = (1<<7))
#define M4_PH_Get()               ((PORTE >> 7) & 0x1)
#define M4_PH_PIN                  GPIO_PIN_RE7

/*** Macros for PWM_M2H pin ***/
#define PWM_M2H_Get()               ((PORTC >> 1) & 0x1)
#define PWM_M2H_PIN                  GPIO_PIN_RC1

/*** Macros for M1_RST pin ***/
#define M1_RST_Set()               (LATJSET = (1<<8))
#define M1_RST_Clear()             (LATJCLR = (1<<8))
#define M1_RST_Toggle()            (LATJINV= (1<<8))
#define M1_RST_OutputEnable()      (TRISJCLR = (1<<8))
#define M1_RST_InputEnable()       (TRISJSET = (1<<8))
#define M1_RST_Get()               ((PORTJ >> 8) & 0x1)
#define M1_RST_PIN                  GPIO_PIN_RJ8

/*** Macros for M2_RST pin ***/
#define M2_RST_Set()               (LATJSET = (1<<9))
#define M2_RST_Clear()             (LATJCLR = (1<<9))
#define M2_RST_Toggle()            (LATJINV= (1<<9))
#define M2_RST_OutputEnable()      (TRISJCLR = (1<<9))
#define M2_RST_InputEnable()       (TRISJSET = (1<<9))
#define M2_RST_Get()               ((PORTJ >> 9) & 0x1)
#define M2_RST_PIN                  GPIO_PIN_RJ9

/*** Macros for PWM_M1H pin ***/
#define PWM_M1H_Get()               ((PORTC >> 2) & 0x1)
#define PWM_M1H_PIN                  GPIO_PIN_RC2

/*** Macros for PWM_M3H pin ***/
#define PWM_M3H_Get()               ((PORTC >> 3) & 0x1)
#define PWM_M3H_PIN                  GPIO_PIN_RC3

/*** Macros for PWM_M4H pin ***/
#define PWM_M4H_Get()               ((PORTC >> 4) & 0x1)
#define PWM_M4H_PIN                  GPIO_PIN_RC4

/*** Macros for PWM_M4L pin ***/
#define PWM_M4L_Get()               ((PORTG >> 6) & 0x1)
#define PWM_M4L_PIN                  GPIO_PIN_RG6

/*** Macros for M3_RST pin ***/
#define M3_RST_Set()               (LATGSET = (1<<7))
#define M3_RST_Clear()             (LATGCLR = (1<<7))
#define M3_RST_Toggle()            (LATGINV= (1<<7))
#define M3_RST_OutputEnable()      (TRISGCLR = (1<<7))
#define M3_RST_InputEnable()       (TRISGSET = (1<<7))
#define M3_RST_Get()               ((PORTG >> 7) & 0x1)
#define M3_RST_PIN                  GPIO_PIN_RG7

/*** Macros for PWM_M3L pin ***/
#define PWM_M3L_Get()               ((PORTG >> 8) & 0x1)
#define PWM_M3L_PIN                  GPIO_PIN_RG8

/*** Macros for PWM_M1L pin ***/
#define PWM_M1L_Get()               ((PORTG >> 9) & 0x1)
#define PWM_M1L_PIN                  GPIO_PIN_RG9

/*** Macros for M4_RST pin ***/
#define M4_RST_Set()               (LATASET = (1<<0))
#define M4_RST_Clear()             (LATACLR = (1<<0))
#define M4_RST_Toggle()            (LATAINV= (1<<0))
#define M4_RST_OutputEnable()      (TRISACLR = (1<<0))
#define M4_RST_InputEnable()       (TRISASET = (1<<0))
#define M4_RST_Get()               ((PORTA >> 0) & 0x1)
#define M4_RST_PIN                  GPIO_PIN_RA0

/*** Macros for ENC1_A pin ***/
#define ENC1_A_Set()               (LATESET = (1<<9))
#define ENC1_A_Clear()             (LATECLR = (1<<9))
#define ENC1_A_Toggle()            (LATEINV= (1<<9))
#define ENC1_A_OutputEnable()      (TRISECLR = (1<<9))
#define ENC1_A_InputEnable()       (TRISESET = (1<<9))
#define ENC1_A_Get()               ((PORTE >> 9) & 0x1)
#define ENC1_A_PIN                  GPIO_PIN_RE9

/*** Macros for ENC1_B pin ***/
#define ENC1_B_Set()               (LATBSET = (1<<5))
#define ENC1_B_Clear()             (LATBCLR = (1<<5))
#define ENC1_B_Toggle()            (LATBINV= (1<<5))
#define ENC1_B_OutputEnable()      (TRISBCLR = (1<<5))
#define ENC1_B_InputEnable()       (TRISBSET = (1<<5))
#define ENC1_B_Get()               ((PORTB >> 5) & 0x1)
#define ENC1_B_PIN                  GPIO_PIN_RB5

/*** Macros for ENC1_0 pin ***/
#define ENC1_0_Set()               (LATBSET = (1<<4))
#define ENC1_0_Clear()             (LATBCLR = (1<<4))
#define ENC1_0_Toggle()            (LATBINV= (1<<4))
#define ENC1_0_OutputEnable()      (TRISBCLR = (1<<4))
#define ENC1_0_InputEnable()       (TRISBSET = (1<<4))
#define ENC1_0_Get()               ((PORTB >> 4) & 0x1)
#define ENC1_0_PIN                  GPIO_PIN_RB4

/*** Macros for ENC_A_TEST pin ***/
#define ENC_A_TEST_Set()               (LATJSET = (1<<13))
#define ENC_A_TEST_Clear()             (LATJCLR = (1<<13))
#define ENC_A_TEST_Toggle()            (LATJINV= (1<<13))
#define ENC_A_TEST_OutputEnable()      (TRISJCLR = (1<<13))
#define ENC_A_TEST_InputEnable()       (TRISJSET = (1<<13))
#define ENC_A_TEST_Get()               ((PORTJ >> 13) & 0x1)
#define ENC_A_TEST_PIN                  GPIO_PIN_RJ13
#define ENC_A_TEST_InterruptEnable()   (CNENJSET = (1<<13))
#define ENC_A_TEST_InterruptDisable()  (CNENJCLR = (1<<13))

/*** Macros for ENC_B_TEST pin ***/
#define ENC_B_TEST_Set()               (LATJSET = (1<<14))
#define ENC_B_TEST_Clear()             (LATJCLR = (1<<14))
#define ENC_B_TEST_Toggle()            (LATJINV= (1<<14))
#define ENC_B_TEST_OutputEnable()      (TRISJCLR = (1<<14))
#define ENC_B_TEST_InputEnable()       (TRISJSET = (1<<14))
#define ENC_B_TEST_Get()               ((PORTJ >> 14) & 0x1)
#define ENC_B_TEST_PIN                  GPIO_PIN_RJ14
#define ENC_B_TEST_InterruptEnable()   (CNENJSET = (1<<14))
#define ENC_B_TEST_InterruptDisable()  (CNENJCLR = (1<<14))

/*** Macros for ENC2_A pin ***/
#define ENC2_A_Set()               (LATBSET = (1<<3))
#define ENC2_A_Clear()             (LATBCLR = (1<<3))
#define ENC2_A_Toggle()            (LATBINV= (1<<3))
#define ENC2_A_OutputEnable()      (TRISBCLR = (1<<3))
#define ENC2_A_InputEnable()       (TRISBSET = (1<<3))
#define ENC2_A_Get()               ((PORTB >> 3) & 0x1)
#define ENC2_A_PIN                  GPIO_PIN_RB3

/*** Macros for ENC2_B pin ***/
#define ENC2_B_Set()               (LATBSET = (1<<2))
#define ENC2_B_Clear()             (LATBCLR = (1<<2))
#define ENC2_B_Toggle()            (LATBINV= (1<<2))
#define ENC2_B_OutputEnable()      (TRISBCLR = (1<<2))
#define ENC2_B_InputEnable()       (TRISBSET = (1<<2))
#define ENC2_B_Get()               ((PORTB >> 2) & 0x1)
#define ENC2_B_PIN                  GPIO_PIN_RB2

/*** Macros for ENC2_0 pin ***/
#define ENC2_0_Set()               (LATBSET = (1<<1))
#define ENC2_0_Clear()             (LATBCLR = (1<<1))
#define ENC2_0_Toggle()            (LATBINV= (1<<1))
#define ENC2_0_OutputEnable()      (TRISBCLR = (1<<1))
#define ENC2_0_InputEnable()       (TRISBSET = (1<<1))
#define ENC2_0_Get()               ((PORTB >> 1) & 0x1)
#define ENC2_0_PIN                  GPIO_PIN_RB1

/*** Macros for ENC3_A pin ***/
#define ENC3_A_Set()               (LATBSET = (1<<0))
#define ENC3_A_Clear()             (LATBCLR = (1<<0))
#define ENC3_A_Toggle()            (LATBINV= (1<<0))
#define ENC3_A_OutputEnable()      (TRISBCLR = (1<<0))
#define ENC3_A_InputEnable()       (TRISBSET = (1<<0))
#define ENC3_A_Get()               ((PORTB >> 0) & 0x1)
#define ENC3_A_PIN                  GPIO_PIN_RB0

/*** Macros for ENC3_B pin ***/
#define ENC3_B_Set()               (LATBSET = (1<<6))
#define ENC3_B_Clear()             (LATBCLR = (1<<6))
#define ENC3_B_Toggle()            (LATBINV= (1<<6))
#define ENC3_B_OutputEnable()      (TRISBCLR = (1<<6))
#define ENC3_B_InputEnable()       (TRISBSET = (1<<6))
#define ENC3_B_Get()               ((PORTB >> 6) & 0x1)
#define ENC3_B_PIN                  GPIO_PIN_RB6

/*** Macros for ENC3_0 pin ***/
#define ENC3_0_Set()               (LATBSET = (1<<7))
#define ENC3_0_Clear()             (LATBCLR = (1<<7))
#define ENC3_0_Toggle()            (LATBINV= (1<<7))
#define ENC3_0_OutputEnable()      (TRISBCLR = (1<<7))
#define ENC3_0_InputEnable()       (TRISBSET = (1<<7))
#define ENC3_0_Get()               ((PORTB >> 7) & 0x1)
#define ENC3_0_PIN                  GPIO_PIN_RB7

/*** Macros for ENC4_A pin ***/
#define ENC4_A_Set()               (LATASET = (1<<9))
#define ENC4_A_Clear()             (LATACLR = (1<<9))
#define ENC4_A_Toggle()            (LATAINV= (1<<9))
#define ENC4_A_OutputEnable()      (TRISACLR = (1<<9))
#define ENC4_A_InputEnable()       (TRISASET = (1<<9))
#define ENC4_A_Get()               ((PORTA >> 9) & 0x1)
#define ENC4_A_PIN                  GPIO_PIN_RA9

/*** Macros for ENC4_B pin ***/
#define ENC4_B_Set()               (LATASET = (1<<10))
#define ENC4_B_Clear()             (LATACLR = (1<<10))
#define ENC4_B_Toggle()            (LATAINV= (1<<10))
#define ENC4_B_OutputEnable()      (TRISACLR = (1<<10))
#define ENC4_B_InputEnable()       (TRISASET = (1<<10))
#define ENC4_B_Get()               ((PORTA >> 10) & 0x1)
#define ENC4_B_PIN                  GPIO_PIN_RA10

/*** Macros for ENC4_0 pin ***/
#define ENC4_0_Set()               (LATHSET = (1<<0))
#define ENC4_0_Clear()             (LATHCLR = (1<<0))
#define ENC4_0_Toggle()            (LATHINV= (1<<0))
#define ENC4_0_OutputEnable()      (TRISHCLR = (1<<0))
#define ENC4_0_InputEnable()       (TRISHSET = (1<<0))
#define ENC4_0_Get()               ((PORTH >> 0) & 0x1)
#define ENC4_0_PIN                  GPIO_PIN_RH0

/*** Macros for LED1 pin ***/
#define LED1_Set()               (LATHSET = (1<<2))
#define LED1_Clear()             (LATHCLR = (1<<2))
#define LED1_Toggle()            (LATHINV= (1<<2))
#define LED1_OutputEnable()      (TRISHCLR = (1<<2))
#define LED1_InputEnable()       (TRISHSET = (1<<2))
#define LED1_Get()               ((PORTH >> 2) & 0x1)
#define LED1_PIN                  GPIO_PIN_RH2

/*** Macros for ENC5_A pin ***/
#define ENC5_A_Set()               (LATBSET = (1<<8))
#define ENC5_A_Clear()             (LATBCLR = (1<<8))
#define ENC5_A_Toggle()            (LATBINV= (1<<8))
#define ENC5_A_OutputEnable()      (TRISBCLR = (1<<8))
#define ENC5_A_InputEnable()       (TRISBSET = (1<<8))
#define ENC5_A_Get()               ((PORTB >> 8) & 0x1)
#define ENC5_A_PIN                  GPIO_PIN_RB8

/*** Macros for ENC6_A pin ***/
#define ENC6_A_Set()               (LATASET = (1<<1))
#define ENC6_A_Clear()             (LATACLR = (1<<1))
#define ENC6_A_Toggle()            (LATAINV= (1<<1))
#define ENC6_A_OutputEnable()      (TRISACLR = (1<<1))
#define ENC6_A_InputEnable()       (TRISASET = (1<<1))
#define ENC6_A_Get()               ((PORTA >> 1) & 0x1)
#define ENC6_A_PIN                  GPIO_PIN_RA1

/*** Macros for ENC6_B pin ***/
#define ENC6_B_Set()               (LATFSET = (1<<13))
#define ENC6_B_Clear()             (LATFCLR = (1<<13))
#define ENC6_B_Toggle()            (LATFINV= (1<<13))
#define ENC6_B_OutputEnable()      (TRISFCLR = (1<<13))
#define ENC6_B_InputEnable()       (TRISFSET = (1<<13))
#define ENC6_B_Get()               ((PORTF >> 13) & 0x1)
#define ENC6_B_PIN                  GPIO_PIN_RF13

/*** Macros for ENC6_0 pin ***/
#define ENC6_0_Set()               (LATFSET = (1<<12))
#define ENC6_0_Clear()             (LATFCLR = (1<<12))
#define ENC6_0_Toggle()            (LATFINV= (1<<12))
#define ENC6_0_OutputEnable()      (TRISFCLR = (1<<12))
#define ENC6_0_InputEnable()       (TRISFSET = (1<<12))
#define ENC6_0_Get()               ((PORTF >> 12) & 0x1)
#define ENC6_0_PIN                  GPIO_PIN_RF12

/*** Macros for ENC7_A pin ***/
#define ENC7_A_Set()               (LATBSET = (1<<12))
#define ENC7_A_Clear()             (LATBCLR = (1<<12))
#define ENC7_A_Toggle()            (LATBINV= (1<<12))
#define ENC7_A_OutputEnable()      (TRISBCLR = (1<<12))
#define ENC7_A_InputEnable()       (TRISBSET = (1<<12))
#define ENC7_A_Get()               ((PORTB >> 12) & 0x1)
#define ENC7_A_PIN                  GPIO_PIN_RB12

/*** Macros for ENC7_B pin ***/
#define ENC7_B_Set()               (LATBSET = (1<<13))
#define ENC7_B_Clear()             (LATBCLR = (1<<13))
#define ENC7_B_Toggle()            (LATBINV= (1<<13))
#define ENC7_B_OutputEnable()      (TRISBCLR = (1<<13))
#define ENC7_B_InputEnable()       (TRISBSET = (1<<13))
#define ENC7_B_Get()               ((PORTB >> 13) & 0x1)
#define ENC7_B_PIN                  GPIO_PIN_RB13

/*** Macros for ENC7_0 pin ***/
#define ENC7_0_Set()               (LATBSET = (1<<14))
#define ENC7_0_Clear()             (LATBCLR = (1<<14))
#define ENC7_0_Toggle()            (LATBINV= (1<<14))
#define ENC7_0_OutputEnable()      (TRISBCLR = (1<<14))
#define ENC7_0_InputEnable()       (TRISBSET = (1<<14))
#define ENC7_0_Get()               ((PORTB >> 14) & 0x1)
#define ENC7_0_PIN                  GPIO_PIN_RB14

/*** Macros for ENC8_A pin ***/
#define ENC8_A_Set()               (LATBSET = (1<<15))
#define ENC8_A_Clear()             (LATBCLR = (1<<15))
#define ENC8_A_Toggle()            (LATBINV= (1<<15))
#define ENC8_A_OutputEnable()      (TRISBCLR = (1<<15))
#define ENC8_A_InputEnable()       (TRISBSET = (1<<15))
#define ENC8_A_Get()               ((PORTB >> 15) & 0x1)
#define ENC8_A_PIN                  GPIO_PIN_RB15

/*** Macros for ENC8_B pin ***/
#define ENC8_B_Set()               (LATHSET = (1<<4))
#define ENC8_B_Clear()             (LATHCLR = (1<<4))
#define ENC8_B_Toggle()            (LATHINV= (1<<4))
#define ENC8_B_OutputEnable()      (TRISHCLR = (1<<4))
#define ENC8_B_InputEnable()       (TRISHSET = (1<<4))
#define ENC8_B_Get()               ((PORTH >> 4) & 0x1)
#define ENC8_B_PIN                  GPIO_PIN_RH4

/*** Macros for ENC8_0 pin ***/
#define ENC8_0_Set()               (LATHSET = (1<<5))
#define ENC8_0_Clear()             (LATHCLR = (1<<5))
#define ENC8_0_Toggle()            (LATHINV= (1<<5))
#define ENC8_0_OutputEnable()      (TRISHCLR = (1<<5))
#define ENC8_0_InputEnable()       (TRISHSET = (1<<5))
#define ENC8_0_Get()               ((PORTH >> 5) & 0x1)
#define ENC8_0_PIN                  GPIO_PIN_RH5

/*** Macros for PLLC1 pin ***/
#define PLLC1_Get()               ((PORTH >> 6) & 0x1)
#define PLLC1_PIN                  GPIO_PIN_RH6

/*** Macros for PLLC2 pin ***/
#define PLLC2_Get()               ((PORTD >> 14) & 0x1)
#define PLLC2_PIN                  GPIO_PIN_RD14

/*** Macros for PLLC3 pin ***/
#define PLLC3_Get()               ((PORTD >> 15) & 0x1)
#define PLLC3_PIN                  GPIO_PIN_RD15

/*** Macros for CAN2_TX pin ***/
#define CAN2_TX_Get()               ((PORTF >> 3) & 0x1)
#define CAN2_TX_PIN                  GPIO_PIN_RF3

/*** Macros for CAN2_RX pin ***/
#define CAN2_RX_Get()               ((PORTF >> 8) & 0x1)
#define CAN2_RX_PIN                  GPIO_PIN_RF8

/*** Macros for SCL1_TEST pin ***/
#define SCL1_TEST_Get()               ((PORTA >> 14) & 0x1)
#define SCL1_TEST_PIN                  GPIO_PIN_RA14

/*** Macros for SDA1_TEST pin ***/
#define SDA1_TEST_Get()               ((PORTA >> 15) & 0x1)
#define SDA1_TEST_PIN                  GPIO_PIN_RA15

/*** Macros for SS4_OUT_TEST pin ***/
#define SS4_OUT_TEST_Get()               ((PORTD >> 9) & 0x1)
#define SS4_OUT_TEST_PIN                  GPIO_PIN_RD9

/*** Macros for SCK4_TEST pin ***/
#define SCK4_TEST_Get()               ((PORTD >> 10) & 0x1)
#define SCK4_TEST_PIN                  GPIO_PIN_RD10

/*** Macros for SDI4_MISO_TEST pin ***/
#define SDI4_MISO_TEST_Get()               ((PORTD >> 11) & 0x1)
#define SDI4_MISO_TEST_PIN                  GPIO_PIN_RD11

/*** Macros for SDO4_MOSI_TEST pin ***/
#define SDO4_MOSI_TEST_Get()               ((PORTD >> 0) & 0x1)
#define SDO4_MOSI_TEST_PIN                  GPIO_PIN_RD0

/*** Macros for CAN1_RX pin ***/
#define CAN1_RX_Get()               ((PORTC >> 13) & 0x1)
#define CAN1_RX_PIN                  GPIO_PIN_RC13

/*** Macros for CAN1_TX pin ***/
#define CAN1_TX_Get()               ((PORTC >> 14) & 0x1)
#define CAN1_TX_PIN                  GPIO_PIN_RC14

/*** Macros for SOL5_EN pin ***/
#define SOL5_EN_Set()               (LATDSET = (1<<1))
#define SOL5_EN_Clear()             (LATDCLR = (1<<1))
#define SOL5_EN_Toggle()            (LATDINV= (1<<1))
#define SOL5_EN_OutputEnable()      (TRISDCLR = (1<<1))
#define SOL5_EN_InputEnable()       (TRISDSET = (1<<1))
#define SOL5_EN_Get()               ((PORTD >> 1) & 0x1)
#define SOL5_EN_PIN                  GPIO_PIN_RD1

/*** Macros for SOL5_EXT pin ***/
#define SOL5_EXT_Set()               (LATDSET = (1<<2))
#define SOL5_EXT_Clear()             (LATDCLR = (1<<2))
#define SOL5_EXT_Toggle()            (LATDINV= (1<<2))
#define SOL5_EXT_OutputEnable()      (TRISDCLR = (1<<2))
#define SOL5_EXT_InputEnable()       (TRISDSET = (1<<2))
#define SOL5_EXT_Get()               ((PORTD >> 2) & 0x1)
#define SOL5_EXT_PIN                  GPIO_PIN_RD2

/*** Macros for SOL5_RET pin ***/
#define SOL5_RET_Set()               (LATDSET = (1<<3))
#define SOL5_RET_Clear()             (LATDCLR = (1<<3))
#define SOL5_RET_Toggle()            (LATDINV= (1<<3))
#define SOL5_RET_OutputEnable()      (TRISDCLR = (1<<3))
#define SOL5_RET_InputEnable()       (TRISDSET = (1<<3))
#define SOL5_RET_Get()               ((PORTD >> 3) & 0x1)
#define SOL5_RET_PIN                  GPIO_PIN_RD3

/*** Macros for SOL6_EN pin ***/
#define SOL6_EN_Set()               (LATDSET = (1<<12))
#define SOL6_EN_Clear()             (LATDCLR = (1<<12))
#define SOL6_EN_Toggle()            (LATDINV= (1<<12))
#define SOL6_EN_OutputEnable()      (TRISDCLR = (1<<12))
#define SOL6_EN_InputEnable()       (TRISDSET = (1<<12))
#define SOL6_EN_Get()               ((PORTD >> 12) & 0x1)
#define SOL6_EN_PIN                  GPIO_PIN_RD12

/*** Macros for SOL6_EXT pin ***/
#define SOL6_EXT_Set()               (LATDSET = (1<<13))
#define SOL6_EXT_Clear()             (LATDCLR = (1<<13))
#define SOL6_EXT_Toggle()            (LATDINV= (1<<13))
#define SOL6_EXT_OutputEnable()      (TRISDCLR = (1<<13))
#define SOL6_EXT_InputEnable()       (TRISDSET = (1<<13))
#define SOL6_EXT_Get()               ((PORTD >> 13) & 0x1)
#define SOL6_EXT_PIN                  GPIO_PIN_RD13

/*** Macros for SOL6_RET pin ***/
#define SOL6_RET_Set()               (LATJSET = (1<<0))
#define SOL6_RET_Clear()             (LATJCLR = (1<<0))
#define SOL6_RET_Toggle()            (LATJINV= (1<<0))
#define SOL6_RET_OutputEnable()      (TRISJCLR = (1<<0))
#define SOL6_RET_InputEnable()       (TRISJSET = (1<<0))
#define SOL6_RET_Get()               ((PORTJ >> 0) & 0x1)
#define SOL6_RET_PIN                  GPIO_PIN_RJ0

/*** Macros for U5TX_TEST pin ***/
#define U5TX_TEST_Get()               ((PORTF >> 0) & 0x1)
#define U5TX_TEST_PIN                  GPIO_PIN_RF0

/*** Macros for U5RX_TEST pin ***/
#define U5RX_TEST_Get()               ((PORTF >> 1) & 0x1)
#define U5RX_TEST_PIN                  GPIO_PIN_RF1

/*** Macros for LS1 pin ***/
#define LS1_Set()               (LATASET = (1<<7))
#define LS1_Clear()             (LATACLR = (1<<7))
#define LS1_Toggle()            (LATAINV= (1<<7))
#define LS1_OutputEnable()      (TRISACLR = (1<<7))
#define LS1_InputEnable()       (TRISASET = (1<<7))
#define LS1_Get()               ((PORTA >> 7) & 0x1)
#define LS1_PIN                  GPIO_PIN_RA7

/*** Macros for LS2 pin ***/
#define LS2_Set()               (LATJSET = (1<<4))
#define LS2_Clear()             (LATJCLR = (1<<4))
#define LS2_Toggle()            (LATJINV= (1<<4))
#define LS2_OutputEnable()      (TRISJCLR = (1<<4))
#define LS2_InputEnable()       (TRISJSET = (1<<4))
#define LS2_Get()               ((PORTJ >> 4) & 0x1)
#define LS2_PIN                  GPIO_PIN_RJ4

/*** Macros for LS3 pin ***/
#define LS3_Set()               (LATJSET = (1<<5))
#define LS3_Clear()             (LATJCLR = (1<<5))
#define LS3_Toggle()            (LATJINV= (1<<5))
#define LS3_OutputEnable()      (TRISJCLR = (1<<5))
#define LS3_InputEnable()       (TRISJSET = (1<<5))
#define LS3_Get()               ((PORTJ >> 5) & 0x1)
#define LS3_PIN                  GPIO_PIN_RJ5

/*** Macros for LS4 pin ***/
#define LS4_Set()               (LATJSET = (1<<6))
#define LS4_Clear()             (LATJCLR = (1<<6))
#define LS4_Toggle()            (LATJINV= (1<<6))
#define LS4_OutputEnable()      (TRISJCLR = (1<<6))
#define LS4_InputEnable()       (TRISJSET = (1<<6))
#define LS4_Get()               ((PORTJ >> 6) & 0x1)
#define LS4_PIN                  GPIO_PIN_RJ6

/*** Macros for LS5 pin ***/
#define LS5_Set()               (LATJSET = (1<<7))
#define LS5_Clear()             (LATJCLR = (1<<7))
#define LS5_Toggle()            (LATJINV= (1<<7))
#define LS5_OutputEnable()      (TRISJCLR = (1<<7))
#define LS5_InputEnable()       (TRISJSET = (1<<7))
#define LS5_Get()               ((PORTJ >> 7) & 0x1)
#define LS5_PIN                  GPIO_PIN_RJ7

/*** Macros for LS6 pin ***/
#define LS6_Set()               (LATESET = (1<<0))
#define LS6_Clear()             (LATECLR = (1<<0))
#define LS6_Toggle()            (LATEINV= (1<<0))
#define LS6_OutputEnable()      (TRISECLR = (1<<0))
#define LS6_InputEnable()       (TRISESET = (1<<0))
#define LS6_Get()               ((PORTE >> 0) & 0x1)
#define LS6_PIN                  GPIO_PIN_RE0

/*** Macros for LS7 pin ***/
#define LS7_Set()               (LATESET = (1<<1))
#define LS7_Clear()             (LATECLR = (1<<1))
#define LS7_Toggle()            (LATEINV= (1<<1))
#define LS7_OutputEnable()      (TRISECLR = (1<<1))
#define LS7_InputEnable()       (TRISESET = (1<<1))
#define LS7_Get()               ((PORTE >> 1) & 0x1)
#define LS7_PIN                  GPIO_PIN_RE1

/*** Macros for LS8 pin ***/
#define LS8_Set()               (LATGSET = (1<<14))
#define LS8_Clear()             (LATGCLR = (1<<14))
#define LS8_Toggle()            (LATGINV= (1<<14))
#define LS8_OutputEnable()      (TRISGCLR = (1<<14))
#define LS8_InputEnable()       (TRISGSET = (1<<14))
#define LS8_Get()               ((PORTG >> 14) & 0x1)
#define LS8_PIN                  GPIO_PIN_RG14

/*** Macros for LS15 pin ***/
#define LS15_Set()               (LATGSET = (1<<12))
#define LS15_Clear()             (LATGCLR = (1<<12))
#define LS15_Toggle()            (LATGINV= (1<<12))
#define LS15_OutputEnable()      (TRISGCLR = (1<<12))
#define LS15_InputEnable()       (TRISGSET = (1<<12))
#define LS15_Get()               ((PORTG >> 12) & 0x1)
#define LS15_PIN                  GPIO_PIN_RG12

/*** Macros for LS16 pin ***/
#define LS16_Set()               (LATGSET = (1<<13))
#define LS16_Clear()             (LATGCLR = (1<<13))
#define LS16_Toggle()            (LATGINV= (1<<13))
#define LS16_OutputEnable()      (TRISGCLR = (1<<13))
#define LS16_InputEnable()       (TRISGSET = (1<<13))
#define LS16_Get()               ((PORTG >> 13) & 0x1)
#define LS16_PIN                  GPIO_PIN_RG13

/*** Macros for LS17 pin ***/
#define LS17_Set()               (LATESET = (1<<2))
#define LS17_Clear()             (LATECLR = (1<<2))
#define LS17_Toggle()            (LATEINV= (1<<2))
#define LS17_OutputEnable()      (TRISECLR = (1<<2))
#define LS17_InputEnable()       (TRISESET = (1<<2))
#define LS17_Get()               ((PORTE >> 2) & 0x1)
#define LS17_PIN                  GPIO_PIN_RE2

/*** Macros for LS18 pin ***/
#define LS18_Set()               (LATESET = (1<<3))
#define LS18_Clear()             (LATECLR = (1<<3))
#define LS18_Toggle()            (LATEINV= (1<<3))
#define LS18_OutputEnable()      (TRISECLR = (1<<3))
#define LS18_InputEnable()       (TRISESET = (1<<3))
#define LS18_Get()               ((PORTE >> 3) & 0x1)
#define LS18_PIN                  GPIO_PIN_RE3

/*** Macros for PLLC4 pin ***/
#define PLLC4_Get()               ((PORTE >> 4) & 0x1)
#define PLLC4_PIN                  GPIO_PIN_RE4


// *****************************************************************************
/* GPIO Port

  Summary:
    Identifies the available GPIO Ports.

  Description:
    This enumeration identifies the available GPIO Ports.

  Remarks:
    The caller should not rely on the specific numbers assigned to any of
    these values as they may change from one processor to the next.

    Not all ports are available on all devices.  Refer to the specific
    device data sheet to determine which ports are supported.
*/

typedef enum
{
    GPIO_PORT_A = 0,
    GPIO_PORT_B = 1,
    GPIO_PORT_C = 2,
    GPIO_PORT_D = 3,
    GPIO_PORT_E = 4,
    GPIO_PORT_F = 5,
    GPIO_PORT_G = 6,
    GPIO_PORT_H = 7,
    GPIO_PORT_J = 8,
    GPIO_PORT_K = 9,
} GPIO_PORT;

// *****************************************************************************
/* GPIO Port Pins

  Summary:
    Identifies the available GPIO port pins.

  Description:
    This enumeration identifies the available GPIO port pins.

  Remarks:
    The caller should not rely on the specific numbers assigned to any of
    these values as they may change from one processor to the next.

    Not all pins are available on all devices.  Refer to the specific
    device data sheet to determine which pins are supported.
*/

typedef enum
{
    GPIO_PIN_RA0 = 0,
    GPIO_PIN_RA1 = 1,
    GPIO_PIN_RA2 = 2,
    GPIO_PIN_RA3 = 3,
    GPIO_PIN_RA4 = 4,
    GPIO_PIN_RA5 = 5,
    GPIO_PIN_RA6 = 6,
    GPIO_PIN_RA7 = 7,
    GPIO_PIN_RA9 = 9,
    GPIO_PIN_RA10 = 10,
    GPIO_PIN_RA14 = 14,
    GPIO_PIN_RA15 = 15,
    GPIO_PIN_RB0 = 16,
    GPIO_PIN_RB1 = 17,
    GPIO_PIN_RB2 = 18,
    GPIO_PIN_RB3 = 19,
    GPIO_PIN_RB4 = 20,
    GPIO_PIN_RB5 = 21,
    GPIO_PIN_RB6 = 22,
    GPIO_PIN_RB7 = 23,
    GPIO_PIN_RB8 = 24,
    GPIO_PIN_RB9 = 25,
    GPIO_PIN_RB10 = 26,
    GPIO_PIN_RB11 = 27,
    GPIO_PIN_RB12 = 28,
    GPIO_PIN_RB13 = 29,
    GPIO_PIN_RB14 = 30,
    GPIO_PIN_RB15 = 31,
    GPIO_PIN_RC1 = 33,
    GPIO_PIN_RC2 = 34,
    GPIO_PIN_RC3 = 35,
    GPIO_PIN_RC4 = 36,
    GPIO_PIN_RC12 = 44,
    GPIO_PIN_RC13 = 45,
    GPIO_PIN_RC14 = 46,
    GPIO_PIN_RC15 = 47,
    GPIO_PIN_RD0 = 48,
    GPIO_PIN_RD1 = 49,
    GPIO_PIN_RD2 = 50,
    GPIO_PIN_RD3 = 51,
    GPIO_PIN_RD4 = 52,
    GPIO_PIN_RD5 = 53,
    GPIO_PIN_RD6 = 54,
    GPIO_PIN_RD7 = 55,
    GPIO_PIN_RD9 = 57,
    GPIO_PIN_RD10 = 58,
    GPIO_PIN_RD11 = 59,
    GPIO_PIN_RD12 = 60,
    GPIO_PIN_RD13 = 61,
    GPIO_PIN_RD14 = 62,
    GPIO_PIN_RD15 = 63,
    GPIO_PIN_RE0 = 64,
    GPIO_PIN_RE1 = 65,
    GPIO_PIN_RE2 = 66,
    GPIO_PIN_RE3 = 67,
    GPIO_PIN_RE4 = 68,
    GPIO_PIN_RE5 = 69,
    GPIO_PIN_RE6 = 70,
    GPIO_PIN_RE7 = 71,
    GPIO_PIN_RE8 = 72,
    GPIO_PIN_RE9 = 73,
    GPIO_PIN_RF0 = 80,
    GPIO_PIN_RF1 = 81,
    GPIO_PIN_RF2 = 82,
    GPIO_PIN_RF3 = 83,
    GPIO_PIN_RF4 = 84,
    GPIO_PIN_RF5 = 85,
    GPIO_PIN_RF8 = 88,
    GPIO_PIN_RF12 = 92,
    GPIO_PIN_RF13 = 93,
    GPIO_PIN_RG0 = 96,
    GPIO_PIN_RG1 = 97,
    GPIO_PIN_RG6 = 102,
    GPIO_PIN_RG7 = 103,
    GPIO_PIN_RG8 = 104,
    GPIO_PIN_RG9 = 105,
    GPIO_PIN_RG12 = 108,
    GPIO_PIN_RG13 = 109,
    GPIO_PIN_RG14 = 110,
    GPIO_PIN_RG15 = 111,
    GPIO_PIN_RH0 = 112,
    GPIO_PIN_RH1 = 113,
    GPIO_PIN_RH2 = 114,
    GPIO_PIN_RH3 = 115,
    GPIO_PIN_RH4 = 116,
    GPIO_PIN_RH5 = 117,
    GPIO_PIN_RH6 = 118,
    GPIO_PIN_RH7 = 119,
    GPIO_PIN_RH8 = 120,
    GPIO_PIN_RH9 = 121,
    GPIO_PIN_RH10 = 122,
    GPIO_PIN_RH11 = 123,
    GPIO_PIN_RH12 = 124,
    GPIO_PIN_RH13 = 125,
    GPIO_PIN_RH14 = 126,
    GPIO_PIN_RH15 = 127,
    GPIO_PIN_RJ0 = 128,
    GPIO_PIN_RJ1 = 129,
    GPIO_PIN_RJ2 = 130,
    GPIO_PIN_RJ3 = 131,
    GPIO_PIN_RJ4 = 132,
    GPIO_PIN_RJ5 = 133,
    GPIO_PIN_RJ6 = 134,
    GPIO_PIN_RJ7 = 135,
    GPIO_PIN_RJ8 = 136,
    GPIO_PIN_RJ9 = 137,
    GPIO_PIN_RJ10 = 138,
    GPIO_PIN_RJ11 = 139,
    GPIO_PIN_RJ12 = 140,
    GPIO_PIN_RJ13 = 141,
    GPIO_PIN_RJ14 = 142,
    GPIO_PIN_RJ15 = 143,
    GPIO_PIN_RK0 = 144,
    GPIO_PIN_RK1 = 145,
    GPIO_PIN_RK2 = 146,
    GPIO_PIN_RK3 = 147,
    GPIO_PIN_RK4 = 148,
    GPIO_PIN_RK5 = 149,
    GPIO_PIN_RK6 = 150,
    GPIO_PIN_RK7 = 151,

    /* This element should not be used in any of the GPIO APIs.
       It will be used by other modules or application to denote that none of the GPIO Pin is used */
    GPIO_PIN_NONE = -1

} GPIO_PIN;

typedef  void (*GPIO_PIN_CALLBACK) ( GPIO_PIN pin, uintptr_t context);

void GPIO_Initialize(void);

// *****************************************************************************
// *****************************************************************************
// Section: GPIO Functions which operates on multiple pins of a port
// *****************************************************************************
// *****************************************************************************

uint32_t GPIO_PortRead(GPIO_PORT port);

void GPIO_PortWrite(GPIO_PORT port, uint32_t mask, uint32_t value);

uint32_t GPIO_PortLatchRead ( GPIO_PORT port );

void GPIO_PortSet(GPIO_PORT port, uint32_t mask);

void GPIO_PortClear(GPIO_PORT port, uint32_t mask);

void GPIO_PortToggle(GPIO_PORT port, uint32_t mask);

void GPIO_PortInputEnable(GPIO_PORT port, uint32_t mask);

void GPIO_PortOutputEnable(GPIO_PORT port, uint32_t mask);

void GPIO_PortInterruptEnable(GPIO_PORT port, uint32_t mask);

void GPIO_PortInterruptDisable(GPIO_PORT port, uint32_t mask);

// *****************************************************************************
// *****************************************************************************
// Section: Local Data types and Prototypes
// *****************************************************************************
// *****************************************************************************

typedef struct {

    /* target pin */
    GPIO_PIN                 pin;

    /* Callback for event on target pin*/
    GPIO_PIN_CALLBACK        callback;

    /* Callback Context */
    uintptr_t               context;

} GPIO_PIN_CALLBACK_OBJ;

// *****************************************************************************
// *****************************************************************************
// Section: GPIO Functions which operates on one pin at a time
// *****************************************************************************
// *****************************************************************************

static inline void GPIO_PinWrite(GPIO_PIN pin, bool value)
{
    GPIO_PortWrite((GPIO_PORT)(pin>>4), (uint32_t)(0x1) << (pin & 0xF), (uint32_t)(value) << (pin & 0xF));
}

static inline bool GPIO_PinRead(GPIO_PIN pin)
{
    return (bool)(((GPIO_PortRead((GPIO_PORT)(pin>>4))) >> (pin & 0xF)) & 0x1);
}

static inline bool GPIO_PinLatchRead(GPIO_PIN pin)
{
    return (bool)((GPIO_PortLatchRead((GPIO_PORT)(pin>>4)) >> (pin & 0xF)) & 0x1);
}

static inline void GPIO_PinToggle(GPIO_PIN pin)
{
    GPIO_PortToggle((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

static inline void GPIO_PinSet(GPIO_PIN pin)
{
    GPIO_PortSet((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

static inline void GPIO_PinClear(GPIO_PIN pin)
{
    GPIO_PortClear((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

static inline void GPIO_PinInputEnable(GPIO_PIN pin)
{
    GPIO_PortInputEnable((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

static inline void GPIO_PinOutputEnable(GPIO_PIN pin)
{
    GPIO_PortOutputEnable((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

static inline void GPIO_PinInterruptEnable(GPIO_PIN pin)
{
    GPIO_PortInterruptEnable((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

static inline void GPIO_PinInterruptDisable(GPIO_PIN pin)
{
    GPIO_PortInterruptDisable((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

bool GPIO_PinInterruptCallbackRegister(
    GPIO_PIN pin,
    const   GPIO_PIN_CALLBACK callBack,
    uintptr_t context
);

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif
// DOM-IGNORE-END
#endif // PLIB_GPIO_H
