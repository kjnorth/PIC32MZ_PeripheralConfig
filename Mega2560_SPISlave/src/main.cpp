/** 
 * @Author: Kodiak North 
 * @Date: 2021-06-28 12:49:17 
 * @Desc: configure the device as a SPI slave to communicate with a PIC32MZ microcontroller 
 */

#include <Arduino.h>
#include <SPI.h>
#include "DataLog.h"

#define DUMMY_DATA 0xFF

volatile uint8_t rec, send;

void setup() {
  Serial.begin(115200);
  LogInfo("SPI Slave project begins\n");

  uint8_t sreg = SREG; // save the current status register (SREG) configuration
  // init the SPI
  rec = 0;
  send = DUMMY_DATA;
  SPDR = send;
  pinMode(SS, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(MISO, OUTPUT);
  pinMode(SCK, INPUT);
  SPCR = 0xC0; // interrupts enabled, SPI peripheral enabled, MSB transmitted first, slave, SCK idle low, sample on rising edge, clk bits [1:0] meaningless in slave mode
  SREG = sreg;
}

void loop() {
  static uint8_t preRec = rec;
  if (rec != preRec) {
    LogInfo(F("received byte: 0x%X\n"), rec);
    preRec = rec;
  }
}

ISR (SPI_STC_vect) {
  rec = SPDR;
  send = DUMMY_DATA;
  SPDR = send;
}