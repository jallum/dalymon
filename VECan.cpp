#include <Arduino.h>
#include <FlexCAN.h>
#include "VECan.h"

namespace Victron {

void encode0x351(CAN_message_t* m, uint16_t batteryHighVoltage_100mV, uint16_t chargeCurrentLimit_100mA, uint16_t dischargeCurrentLimit_100mA, uint16_t batteryLowVoltage_100mV) {
  m->id  = 0x351;
  m->len = 8;
  /**/  
  m->buf[0] = batteryHighVoltage_100mV & 0xFF;
  m->buf[1] = batteryHighVoltage_100mV >> 0x08;
  m->buf[2] = chargeCurrentLimit_100mA & 0xFF;
  m->buf[3] = chargeCurrentLimit_100mA >> 0x08;
  m->buf[4] = dischargeCurrentLimit_100mA & 0xFF;
  m->buf[5] = dischargeCurrentLimit_100mA >> 0x08;
  m->buf[6] = batteryLowVoltage_100mV & 0xFF;
  m->buf[7] = batteryLowVoltage_100mV >> 0x08;
}

void encode0x355(CAN_message_t* m, uint8_t stateOfCharge, uint8_t stateOfHealth) {
  m->id  = 0x355;
  m->len = 8;
  /**/  
  m->buf[0] = stateOfCharge;
  m->buf[1] = 0;
  m->buf[2] = stateOfHealth;
  m->buf[3] = 0;
  m->buf[4] = 0;
  m->buf[5] = 0;
  m->buf[6] = 0;
  m->buf[7] = 0;
}

void encode0x356(CAN_message_t* m, uint16_t batteryVoltage_10mV, uint16_t current_100mA, int16_t temperature_100mC) {
  m->id  = 0x356;
  m->len = 8;
  /**/  
  m->buf[0] = batteryVoltage_10mV & 0xFF;
  m->buf[1] = batteryVoltage_10mV >> 0x08;
  m->buf[2] = current_100mA & 0xFF;
  m->buf[3] = current_100mA >> 0x08;
  m->buf[4] = temperature_100mC & 0xFF;
  m->buf[5] = temperature_100mC >> 0x08;
  m->buf[6] = 0;
  m->buf[7] = 0;
}

void encode0x35A(CAN_message_t* m) {
  m->id  = 0x35A;
  m->len = 8;
  /**/  
  m->buf[0] = 0;
  m->buf[1] = 0;
  m->buf[2] = 0;
  m->buf[3] = 0;
  m->buf[4] = 0;
  m->buf[5] = 0;
  m->buf[6] = 0;
  m->buf[7] = 0;
}

void encode0x35E(CAN_message_t* m, char manufacturerName[8]) {
  m->id  = 0x35E;
  m->len = 8;
  /**/  
  memcpy(m->buf, manufacturerName, 8);
}

void encode0x370(CAN_message_t* m, char bmsName[8]) {
  m->id  = 0x370;
  m->len = 8;
  /**/  
  memcpy(m->buf, bmsName, 8);
}

void encode0x372(CAN_message_t* m, uint16_t numberOfModules) {
  m->id  = 0x372;
  m->len = 2;
  /**/  
  m->buf[0] = numberOfModules & 0xFF;
  m->buf[1] = numberOfModules >> 0x08;
}

void encode0x379(CAN_message_t* m, uint16_t batteryCapacity_Ah) {
  m->id  = 0x379;
  m->len = 2;
  /**/  
  m->buf[0] = batteryCapacity_Ah & 0xFF;
  m->buf[1] = batteryCapacity_Ah >> 0x08;
}
 
};
