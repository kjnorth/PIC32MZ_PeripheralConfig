/** 
 * @Author: Kodiak North 
 * @Date: 2021-06-28 12:49:17 
 * @Desc: configure the device as a SPI slave to communicate with a PIC32MZ microcontroller 
 */

#include <Arduino.h>
#include <SPI.h>
#include "DataLog.h"

#define PIC32_SERIAL Serial1
#define DUMMY_SPI_DATA 0xFF
#define PRINT_STR_MAX_LEN 255u
#define PRINT_EXPECTED_ACK 0xACu

volatile uint8_t rec, send;

void setup() {
  Serial.begin(115200);
  PIC32_SERIAL.begin(57600);
  LogInfo("SPI Slave project begins\n");

  uint8_t sreg = SREG; // save the current status register (SREG) configuration
  // init the SPI
  rec = 0;
  send = DUMMY_SPI_DATA;
  SPDR = send;
  pinMode(SS, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(MISO, OUTPUT);
  pinMode(SCK, INPUT);
  SPCR = 0xC0; // SPI interrupts enabled, SPI peripheral enabled, MSB transmitted first, slave mode,
               // SCK idle low, sample on rising edge, clk bits [1:0] meaningless in slave mode
  SREG = sreg;
}

void loop() {
  static uint8_t preRec = rec;
  if (rec != preRec) {
    LogInfo(F("received byte: 0x%02X, available uart rx bytes %d\n"), rec, PIC32_SERIAL.available());
    preRec = rec;
  }
  if (PIC32_SERIAL.available()) {
    char message[PRINT_STR_MAX_LEN];
    int size = PIC32_SERIAL.readBytesUntil('\0', message, PRINT_STR_MAX_LEN);
    for (int i = 0; i < size; i++) {
      Serial.print(message[i]);
    }
    // PIC32_SERIAL.readBytes(message, 44);
    // Serial.print(message);
    // Serial.print("\nmessage ended \n");
    PIC32_SERIAL.write(PRINT_EXPECTED_ACK);
  }
}

ISR (SPI_STC_vect) {
  rec = SPDR;
  send++;
  SPDR = send;
}