/** 
 * @Author: Kodiak North 
 * @Date: 2021-06-28 12:49:17 
 * @Desc: configure the device as a SPI slave to communicate with a PIC32MZ microcontroller 
 */

#include <Arduino.h>
#include <SPI.h>
#include "DataLog.h"

// spi
#define DUMMY_SPI_DATA 0x7Eu
volatile uint8_t rec, send, isDataUpdated;

// uart
#define PIC32_SERIAL Serial2
#define PRINT_STR_MAX_LEN 256u
#define START 0xA5u
#define ACK 0xF9u
#define PRINT_READ_TIMEOUT_MS 10u
typedef enum {
  WAIT_START,
  SEND_ACK_START,
  WAIT_MSG,
  SEND_ACK_MSG,
} uart_comm_t;
uart_comm_t commState = WAIT_START;
uart_comm_t preState = commState;

void setup() {
  Serial.begin(115200);
  PIC32_SERIAL.begin(115200);
  LogInfo("Arduino print receiver begins\n");

  uint8_t sreg = SREG; // save the current status register (SREG) configuration
  // init the SPI
  rec = 0;
  send = DUMMY_SPI_DATA;
  isDataUpdated = 0;
  SPDR = send;
  pinMode(SS, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(MISO, OUTPUT);
  pinMode(SCK, INPUT);
  SPCR = 0xC0; // SPI interrupts enabled, SPI peripheral enabled, MSB transmitted first, slave mode,
               // SCK idle low, sample on rising edge, clk bits [1:0] meaningless in slave mode
  SREG = sreg;
}

void UartComm(void) {
  uint8_t ack = ACK;
  int size = PIC32_SERIAL.available();
  static uint32_t waitMsgStartTime = 0;

  switch (commState) {
    case WAIT_START: {
      if (size > 0) {
        uint8_t byteRec = PIC32_SERIAL.read();
        if (byteRec == START) {
          commState = SEND_ACK_START;
        }
      }
      break;
    }
    case SEND_ACK_START:
      PIC32_SERIAL.write(&ack, 1);
      waitMsgStartTime = millis();
      commState = WAIT_MSG;
      break;
    case WAIT_MSG:
      if (PIC32_SERIAL.available()) {
        char buf[PRINT_STR_MAX_LEN];
        int bytesRead = PIC32_SERIAL.readBytesUntil('\0', buf, PRINT_STR_MAX_LEN);
        for (int i = 0; i < bytesRead; i++) {
          Serial.write(buf[i]);
        }
        commState = SEND_ACK_MSG;
      }
      else {
        uint32_t curtime = millis();
        if (curtime - waitMsgStartTime > PRINT_READ_TIMEOUT_MS) {
          // sending unit is not sending a message, abort
          commState = WAIT_START;
        }
      }
      break;
    case SEND_ACK_MSG:
      PIC32_SERIAL.write(&ack, 1);
      commState = WAIT_START;
      break;
  }
}

void loop() {
  if (isDataUpdated) {
    isDataUpdated = 0;
    LogInfo("PIC print state = 0x%02X\n", rec);
  }
  
  UartComm();
}

ISR (SPI_STC_vect) {
  rec = SPDR;
  send = DUMMY_SPI_DATA;
  isDataUpdated = 1;
  SPDR = send;
}