#include <Arduino.h>
#include "Daly.h"


typedef struct Module {
  /**/
  uint16_t packVoltage;
  int16_t current;
  uint16_t stateOfCharge;

  /**/
  uint16_t cellMaxVoltage;
  uint16_t cellWithMaxVoltage;
  uint16_t cellMinVoltage;
  uint16_t cellWithMinVoltage;

  /**/
  int8_t cellMaxTemp;
  uint8_t cellWithMaxTemp;
  int8_t cellMinTemp;
  uint8_t cellWithMinTemp;

  /**/
  enum ChargeDischargeStatus : uint8_t {
    Neither = 0,
    Charging = 1,
    Discharging = 2
  } chargeDischargeStatus;
  uint8_t chargeFETsEnabled;
  uint8_t dischargeFETsEnabled;
  uint8_t bmsLife;
  uint32_t remainingCapacityInMAh;

  /**/
  uint8_t numberOfCells;
  uint8_t numberOfTempSensors;
  uint8_t chargerEnabled;
  uint8_t loadEnabled;
  uint8_t stateOfDIDO;
  uint16_t numberOfCycles;

  /**/
  uint16_t voltageByCell[0x30];

  /**/
  uint8_t tempBySensor[0x30];

  /**/
  uint8_t balancerStatusByCell[0x30 / 8];

  /**/
  struct {
    /* 0x00 */
    uint8_t levelOneCellVoltageTooHigh : 1;
    uint8_t levelTwoCellVoltageTooHigh : 1;
    uint8_t levelOneCellVoltageTooLow : 1;
    uint8_t levelTwoCellVoltageTooLow : 1;
    uint8_t levelOnePackVoltageTooHigh : 1;
    uint8_t levelTwoPackVoltageTooHigh : 1;
    uint8_t levelOnePackVoltageTooLow : 1;
    uint8_t levelTwoPackVoltageTooLow : 1;

    /* 0x01 */
    uint8_t levelOneChargeTempTooHigh : 1;
    uint8_t levelTwoChargeTempTooHigh : 1;
    uint8_t levelOneChargeTempTooLow : 1;
    uint8_t levelTwoChargeTempTooLow : 1;
    uint8_t levelOneDischargeTempTooHigh : 1;
    uint8_t levelTwoDischargeTempTooHigh : 1;
    uint8_t levelOneDischargeTempTooLow : 1;
    uint8_t levelTwoDischargeTempTooLow : 1;

    /* 0x02 */
    uint8_t levelOneChargeCurrentTooHigh : 1;
    uint8_t levelTwoChargeCurrentTooHigh : 1;
    uint8_t levelOneDischargeCurrentTooHigh : 1;
    uint8_t levelTwoDischargeCurrentTooHigh : 1;
    uint8_t levelOneModuleOfChargeTooHigh : 1;
    uint8_t levelTwoModuleOfChargeTooHigh : 1;
    uint8_t levelOneModuleOfChargeTooLow : 1;
    uint8_t levelTwoModuleOfChargeTooLow : 1;

    /* 0x03 */
    uint8_t levelOneCellVoltageDifferenceTooHigh : 1;
    uint8_t levelTwoCellVoltageDifferenceTooHigh : 1;
    uint8_t levelOneTempSensorDifferenceTooHigh : 1;
    uint8_t levelTwoTempSensorDifferenceTooHigh : 1;
    uint8_t _reserved1 : 4;

    /* 0x04 */
    uint8_t chargeFETTemperatureTooHigh : 1;
    uint8_t dischargeFETTemperatureTooHigh : 1;
    uint8_t failureOfChargeFETTemperatureSensor : 1;
    uint8_t failureOfDischargeFETTemperatureSensor : 1;
    uint8_t failureOfChargeFETAdhesion : 1;
    uint8_t failureOfDischargeFETAdhesion : 1;
    uint8_t failureOfChargeFETTBreaker : 1;
    uint8_t failureOfDischargeFETBreaker : 1;

    /* 0x05 */
    uint8_t failureOfAFEAcquisitionModule : 1;
    uint8_t failureOfVoltageSensorModule : 1;
    uint8_t failureOfTemperatureSensorModule : 1;
    uint8_t failureOfEEPROMStorageModule : 1;
    uint8_t failureOfRealtimeClockModule : 1;
    uint8_t failureOfPrechargeModule : 1;
    uint8_t failureOfVehicleCommunicationModule : 1;
    uint8_t failureOfIntranetCommunicationModule : 1;

    /* 0x06 */            
    uint8_t failureOfCurrentSensorModule : 1;
    uint8_t failureOfMainVoltageSensorModule : 1;
    uint8_t failureOfShortCircuitProtection : 1;
    uint8_t failureOfLowVoltageNoCharging : 1;
    uint8_t _reserved2 : 4;

    /* 0x07 */
    uint8_t faultCode;
  } failureFlags;
} Module;



#define MONITOR  Serial
#define BMS_UART Serial1

typedef struct Task {
  Daly::Frame::Command commandToRun;
  uint16_t intervalInMillis;
  uint32_t executeAt;
  uint32_t lastRunAt;
  uint32_t lastReplyAt;
} Task;

Task tasks[] = {
  { Daly::Frame::Command::VoltagesByCell,    1000 },
  { Daly::Frame::Command::VoltageAndCurrent, 1000 },
  { Daly::Frame::Command::BasicStatus,       2000 },
  { Daly::Frame::Command::FailureFlags,      5000 },
  { /* EMPTY */ }
};

static Daly::Protocol protocol;
static Daly::UARTPort port(&BMS_UART, &protocol);
static Module module;

void setup() {
  MONITOR.begin(115200);
  BMS_UART.begin(9600);

  protocol.onFrameReceived = [](Daly::Frame* frame, Daly::Protocol* source) {
    uint32_t now = millis();

    Task* t = tasks;
    while (t->commandToRun) {
      if (t->commandToRun == frame->command) {
        t->lastReplyAt = now;
        break;
      }
      t++;
    }

    switch (frame->command) {
      case Daly::Frame::Command::VoltageAndCurrent: {
        module.packVoltage   = ((uint16_t)frame->data.as_bytes[0] << 0x08) | ((uint16_t)frame->data.as_bytes[1]);
        module.current       = (((int16_t)frame->data.as_bytes[4] << 0x08) | (int16_t)frame->data.as_bytes[5]) - 30000;
        module.stateOfCharge = ((uint16_t)frame->data.as_bytes[6] << 0x08) | ((uint16_t)frame->data.as_bytes[7]);
        break;
      }

      case Daly::Frame::Command::MinMaxVoltage: {
        module.cellMaxVoltage      = ((uint16_t)frame->data.as_bytes[0] << 0x08) | ((uint16_t)frame->data.as_bytes[1]);
        module.cellWithMaxVoltage  = frame->data.as_bytes[2];
        module.cellMinVoltage      = ((uint16_t)frame->data.as_bytes[3] << 0x08) | ((uint16_t)frame->data.as_bytes[4]);
        module.cellWithMinVoltage  = frame->data.as_bytes[5];
        break;
      }

      case Daly::Frame::Command::MinMaxTemperature: {
        module.cellMaxTemp       = (int16_t)frame->data.as_bytes[0] - 40;
        module.cellWithMaxTemp   = frame->data.as_bytes[1];
        module.cellMinTemp       = (int16_t)frame->data.as_bytes[2] - 40;
        module.cellWithMinTemp   = frame->data.as_bytes[3];
        break;
      }

      case Daly::Frame::Command::ChargeDischargeStatus: {
        module.chargeDischargeStatus   = (Module::ChargeDischargeStatus)frame->data.as_bytes[0];
        module.chargeFETsEnabled       = frame->data.as_bytes[1];
        module.dischargeFETsEnabled    = frame->data.as_bytes[2];
        module.bmsLife                 = frame->data.as_bytes[3];
        module.remainingCapacityInMAh  = ((uint32_t)frame->data.as_bytes[4] << 0x18) | ((uint32_t)frame->data.as_bytes[5] << 0x10) | ((uint32_t)frame->data.as_bytes[6] << 0x08) | (uint32_t)frame->data.as_bytes[7];
        break;
      }

      case Daly::Frame::Command::BasicStatus: {
        module.numberOfCells       = frame->data.as_bytes[0];
        module.numberOfTempSensors = frame->data.as_bytes[1];
        module.chargerEnabled      = frame->data.as_bytes[2];
        module.loadEnabled         = frame->data.as_bytes[3];
        module.stateOfDIDO         = frame->data.as_bytes[4];
        module.numberOfCycles      = ((uint16_t)frame->data.as_bytes[5] << 0x08) | (uint16_t)frame->data.as_bytes[6];
        break;
      }

      case Daly::Frame::Command::VoltagesByCell: {
        uint8_t frameNumber = frame->data.as_bytes[0];
        if (frameNumber == 0xFF) {
          break;
        }
        uint8_t firstCell = frameNumber * 3;
        for (int i = 0; i < 3; i++) {
          module.voltageByCell[firstCell + i] = ((uint16_t)frame->data.as_bytes[1 + (i << 1)] << 0x08) | (uint16_t)frame->data.as_bytes[2 + (i << 1)];
        }
        break;
      }

      case Daly::Frame::Command::TempsBySensor: {
        uint8_t frameNumber = frame->data.as_bytes[0];
        if (frameNumber == 0xFF) {
          break;
        }
        uint8_t firstCell = frameNumber * 7;
        for (int i = 0; i < 7; i++) {
          module.tempBySensor[firstCell + i] = (int8_t)frame->data.as_bytes[i] - 40;
        }
        break;
      }

      case Daly::Frame::Command::BalancerStatus: {
        for (int i = 0; i < 6; i++) {
          module.balancerStatusByCell[i] = frame->data.as_bytes[i];
        }
        break;
      }

      case Daly::Frame::Command::FailureFlags: {
        memcpy(&module.failureFlags, frame->data.as_bytes, sizeof(module.failureFlags));
        break;
      }
    }

    MONITOR.print("<- ");
    frame->printToStream(&MONITOR);
    MONITOR.print("-- ");
    if (t->commandToRun) {
      MONITOR.print(" RA: ");
      MONITOR.print(t->lastRunAt);
      MONITOR.print(", RTT: ");
      MONITOR.print(t->lastReplyAt - t->lastRunAt);
      MONITOR.print("ms");
    }
    MONITOR.println(); 
    MONITOR.flush();
  };
}

void loop() {
  uint32_t now = millis();
  if (now < 5000) {
    return;
  }
  
  for (Task* t = tasks; t->commandToRun; t++) {
    if (t->executeAt <= now) {
      port.sendCommand(t->commandToRun);
      t->lastRunAt = now;
      t->executeAt = now + t->intervalInMillis;
      break;
    }
  }
  
  port.receiveAvailableBytes();
}
