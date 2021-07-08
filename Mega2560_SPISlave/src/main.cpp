/** 
 * @Author: Kodiak North 
 * @Date: 2021-06-28 12:49:17 
 * @Desc: configure the device as a SPI slave to communicate with a PIC32MZ microcontroller 
 */

#include <Arduino.h>
#include <SPI.h>
#include "DataLog.h"

// spi
#define DUMMY_SPI_DATA 0xFFu
#define PRINT_STR_MAX_LEN 255u
#define PRINT_EXPECTED_ACK 0xACu
volatile uint8_t rec, send, isDataUpdated;

// uart
#define PIC32_SERIAL Serial2
#define START 0xA5u
#define ACK 0xF9u
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
  PIC32_SERIAL.begin(9600);
  PIC32_SERIAL.setTimeout(10); // set 10 ms timeout, decrease as SW works
  LogInfo("SPI Slave project begins\n");

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

  LogInfo("comm state is %u\n", commState);
}

void UartComm(void) {
  if (commState != preState) {
    preState = commState;
    // LogInfo("comm state is %u\n", commState);
  }
  int size = PIC32_SERIAL.available();
  switch (commState) {
    case WAIT_START: {
      // int size = PIC32_SERIAL.available();
      if (size > 0) {
        uint8_t byteRec = PIC32_SERIAL.read();
        if (byteRec == START) {
          commState = SEND_ACK_START;
        }
      }
      break;
    }
    case SEND_ACK_START:
    // LogInfo("ack start\n");
      PIC32_SERIAL.write((const char*)ACK);
      commState = WAIT_MSG;
      break;
    case WAIT_MSG: {
      // LogInfo("size %u\n", size);
      if (size > 0) {
        uint8_t length = 7; // h, e, l, l, o, \n, \0
        char buf[length];
        int bytesRead = PIC32_SERIAL.readBytesUntil('\0', buf, length);
        for (int i = 0; i < bytesRead; i++) {
          // Serial.write((int)buf[i]);
          LogInfo(F("buf[%d] = %u\n"), i, (uint8_t)buf[i]);
        }
        if (bytesRead < size) {
          // empty serial buffer
          for (int i = 0; i < (size-bytesRead); i++) {
            PIC32_SERIAL.read();
          }
        }
        commState = SEND_ACK_MSG;
      }
      break;
    }
    case SEND_ACK_MSG:
    // LogInfo("ack msg\n");
      PIC32_SERIAL.write((const char*)ACK);
      commState = WAIT_START;
      break;
  }
}

void loop() {
  UartComm();
  if (isDataUpdated) {
    isDataUpdated = 0;
    LogInfo("ack received by PIC 0x%02X\n", rec);
  }
}

ISR (SPI_STC_vect) {
  rec = SPDR;
  send = DUMMY_SPI_DATA;
  isDataUpdated = 1;
  SPDR = send;
}