#include <Arduino.h>
#include <FlexCAN.h>
#include <EEPROM.h>
#include "Daly.h"
#include "VECan.h"


typedef struct Module {
  /**/
  uint32_t nominalCapacity_Ah;
  int16_t nominalCellVoltage_mV;

  uint16_t level1AlarmThresholdForHighPackVoltage_100mV;
  uint16_t level2AlarmThresholdForHighPackVoltage_100mV;
  uint16_t level1AlarmThresholdForLowPackVoltage_100mV;
  uint16_t level2AlarmThresholdForLowPackVoltage_100mV;

  int16_t level1AlarmThresholdForHighChargeCurrent_100mA;
  int16_t level2AlarmThresholdForHighChargeCurrent_100mA;
  int16_t level1AlarmThresholdForHighDischargeCurrent_100mA;
  int16_t level2AlarmThresholdForHighDischargeCurrent_100mA;

  /**/
  uint16_t voltage_100mA;
  int16_t current_100mA;
  uint16_t stateOfCharge_10thsOfPct;

  /**/
  uint16_t cellMaxVoltage_mV;
  uint16_t cellWithMaxVoltage_mV;
  uint16_t cellMinVoltage_mV;
  uint16_t cellWithMinVoltage_mV;

  /**/
  int8_t cellMaxTemp_C;
  uint8_t cellWithMaxTemp_C;
  int8_t cellMinTemp_C;
  uint8_t cellWithMinTemp_C;

  /**/
  enum ChargeDischargeStatus : uint8_t {
    Neither = 0,
    Charging = 1,
    Discharging = 2
  } chargeDischargeStatus;
  uint8_t chargeFETsEnabled;
  uint8_t dischargeFETsEnabled;
  uint8_t bmsLife;
  uint32_t remainingCapacity_mAh;

  /**/
  uint8_t numberOfCells;
  uint8_t numberOfTempSensors;
  uint8_t chargerEnabled;
  uint8_t loadEnabled;
  uint8_t stateOfDIDO;
  uint16_t numberOfCycles;

  /**/
  uint16_t voltageByCell_mV[0x30];

  /**/
  uint8_t tempBySensor_100mC[0x30];

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
    uint8_t levelOneStateOfChargeTooHigh : 1;
    uint8_t levelTwoStateOfChargeTooHigh : 1;
    uint8_t levelOneStateOfChargeTooLow : 1;
    uint8_t levelTwoStateOfChargeTooLow : 1;

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


#define LED_PIN 13

#define MONITOR  Serial
#define BMS_UART Serial1

static Daly::Protocol protocol;
static Daly::UARTPort port(&BMS_UART, &protocol);
static Module module;

typedef void Runnable();
typedef struct Task {
  Runnable* runnable;
  uint16_t intervalInMillis;
  uint32_t executeAt;
  uint32_t lastRunAt;
} Task;


void updateVECan() {
  digitalWrite(LED_PIN, HIGH);

  CAN_message_t msg;

  if (module.level2AlarmThresholdForHighPackVoltage_100mV
      && module.level2AlarmThresholdForLowPackVoltage_100mV
      && module.level2AlarmThresholdForHighChargeCurrent_100mA
      && module.level2AlarmThresholdForHighDischargeCurrent_100mA
     ) {
    Victron::encode0x351(
      &msg,
      module.level2AlarmThresholdForHighPackVoltage_100mV,
      module.level2AlarmThresholdForHighChargeCurrent_100mA,
      module.level2AlarmThresholdForHighDischargeCurrent_100mA,
      module.level2AlarmThresholdForLowPackVoltage_100mV
    );
    Can0.write(msg);
    delay(2);
  }

  Victron::encode0x355(&msg, module.stateOfCharge_10thsOfPct / 10, 100 /* State Of Health */);
  Can0.write(msg);
  delay(2);

  if (module.voltage_100mA && module.tempBySensor_100mC[0]) {
    Victron::encode0x356(
      &msg,
      module.voltage_100mA * 10,
      module.current_100mA,
      module.tempBySensor_100mC[0]
    );
    Can0.write(msg);
    delay(2);
  }

  Victron::encode0x35A(&msg);
  Can0.write(msg);
  delay(2);

  char manufacturerName[8] = "DALY";
  Victron::encode0x35E(&msg, manufacturerName);
  Can0.write(msg);
  delay(2);

  char bmsName[8] = "BMS";
  Victron::encode0x370(&msg, bmsName);
  Can0.write(msg);
  delay(2);

  Victron::encode0x372(&msg, module.numberOfCells);
  Can0.write(msg);
  delay(2);

  if (module.nominalCapacity_Ah) {
    Victron::encode0x379(&msg, module.nominalCapacity_Ah);
    Can0.write(msg);
    delay(2);
  }

  digitalWrite(LED_PIN, LOW);
}

void printReport() {
  if (!module.numberOfCells) {
    return;
  }

  uint32_t sum = 0;
  uint8_t numberOfNonZeroCells = 0;
  for (int i = 0; i < module.numberOfCells; i++) {
    uint16_t voltage = module.voltageByCell_mV[i];
    sum += voltage;
    if (voltage) {
      numberOfNonZeroCells++;
    }
  }
  int16_t average = sum / numberOfNonZeroCells;

  Serial.print("\r");
  char buffer[0x10];

  sprintf(buffer, "%+2.1f", (float)module.current_100mA * 0.1);
  Serial.print(buffer);
  Serial.print("a, ");

  sprintf(buffer, "%2.3f", (float)sum * 0.001);
  Serial.print(buffer);
  Serial.print("v, ");

  sprintf(buffer, "%1.3f", (float)average * 0.001);
  Serial.print(buffer);
  Serial.print("v: ");

  for (int i = 0; i < module.numberOfCells; i++) {
    uint16_t voltage = module.voltageByCell_mV[i];
    if (voltage) {
      char buffer[0x10];
      sprintf(buffer, "%+4dmv", voltage - (int16_t)average);
      Serial.print(" ");
      Serial.print(buffer);
    } else {
      Serial.print("    N/A");
    }
  }
}

Task tasks[] = {
  { [](){ port.sendCommand(Daly::Frame::Command::FailureFlags); },                  5000 },
  { [](){ port.sendCommand(Daly::Frame::Command::BasicStatus); },                   2000 },
  { [](){ port.sendCommand(Daly::Frame::Command::VoltageAndCurrent); },             1000 },
  { [](){ port.sendCommand(Daly::Frame::Command::VoltagesByCell); },                1000 },
  { [](){ port.sendCommand(Daly::Frame::Command::TempsBySensor); },                 2000 },
  { [](){ port.sendCommand(Daly::Frame::Command::NominalCapacityAndCellVoltage); }, 5000 },
  { [](){ port.sendCommand(Daly::Frame::Command::AlarmThresholdsForPackVoltage); }, 5000 },
  { [](){ port.sendCommand(Daly::Frame::Command::AlarmThresholdsForPackCurrent); }, 5000 },
  { &printReport,                                                                   2000 },
  { &updateVECan,                                                                   500 },
  { NULL }
};

struct Task* taskToRunNext(uint32_t now) {
  for (Task* t = tasks; t->runnable; t++) {
    if (t->executeAt <= now) {
      return t;
    }
  }
  return NULL;
}

#define EEPROM_VERSION 1
typedef struct __attribute__((packed)) Settings {
  uint8_t version;
  uint32_t canSpeed;
} Settings;

void configureDefaultSettings(struct Settings* s) {
  s->version = EEPROM_VERSION;
  s->canSpeed = 500000;
}

Settings settings;

void setup() {
  pinMode(LED_PIN, OUTPUT);

  MONITOR.begin(115200);
  BMS_UART.begin(9600);

  EEPROM.get(0, settings);
  if (settings.version != EEPROM_VERSION) {
    configureDefaultSettings(&settings);
    EEPROM.put(0, settings);
  }

  Can0.begin(settings.canSpeed);
  CAN_filter_t allPassFilter = { 0, 1, 0 };
  for (int filterNum = 4; filterNum < 16; filterNum++) {
    Can0.setFilter(allPassFilter, filterNum);
  }

  protocol.onFrameReceived = [](Daly::Frame * frame, Daly::Protocol * source) {
    switch (frame->command) {
      case Daly::Frame::Command::NominalCapacityAndCellVoltage: {
        module.nominalCapacity_Ah = ((uint32_t)frame->data.as_bytes[0] << 0x18) | ((uint32_t)frame->data.as_bytes[1] << 0x10) | ((uint32_t)frame->data.as_bytes[2] << 0x08) | (uint32_t)frame->data.as_bytes[3];
        module.nominalCellVoltage_mV = ((uint16_t)frame->data.as_bytes[6] << 0x08) | ((uint16_t)frame->data.as_bytes[7]);
        break;
      }

      case Daly::Frame::Command::AlarmThresholdsForPackVoltage: {
        module.level1AlarmThresholdForHighPackVoltage_100mV = ((uint16_t)frame->data.as_bytes[0] << 0x08) | ((uint16_t)frame->data.as_bytes[1]);
        module.level2AlarmThresholdForHighPackVoltage_100mV = ((uint16_t)frame->data.as_bytes[2] << 0x08) | ((uint16_t)frame->data.as_bytes[3]);
        module.level1AlarmThresholdForLowPackVoltage_100mV = ((uint16_t)frame->data.as_bytes[4] << 0x08) | ((uint16_t)frame->data.as_bytes[5]);
        module.level2AlarmThresholdForLowPackVoltage_100mV = ((uint16_t)frame->data.as_bytes[6] << 0x08) | ((uint16_t)frame->data.as_bytes[7]);
        break;
      }

      case Daly::Frame::Command::AlarmThresholdsForPackCurrent: {
        module.level1AlarmThresholdForHighChargeCurrent_100mA = 30000 - (((int16_t)frame->data.as_bytes[0] << 0x08) | ((int16_t)frame->data.as_bytes[1]));
        module.level2AlarmThresholdForHighChargeCurrent_100mA = 30000 - (((int16_t)frame->data.as_bytes[2] << 0x08) | ((int16_t)frame->data.as_bytes[3]));
        module.level1AlarmThresholdForHighDischargeCurrent_100mA = (((int16_t)frame->data.as_bytes[4] << 0x08) | ((int16_t)frame->data.as_bytes[5])) - 30000;
        module.level2AlarmThresholdForHighDischargeCurrent_100mA = (((int16_t)frame->data.as_bytes[6] << 0x08) | ((int16_t)frame->data.as_bytes[7])) - 30000;
        break;
      }

      case Daly::Frame::Command::VoltageAndCurrent: {
        module.voltage_100mA = ((uint16_t)frame->data.as_bytes[0] << 0x08) | ((uint16_t)frame->data.as_bytes[1]);
        module.current_100mA = (((int16_t)frame->data.as_bytes[4] << 0x08) | (int16_t)frame->data.as_bytes[5]) - 30000;
        module.stateOfCharge_10thsOfPct = ((uint16_t)frame->data.as_bytes[6] << 0x08) | ((uint16_t)frame->data.as_bytes[7]);
        break;
      }

      case Daly::Frame::Command::MinMaxVoltage: {
        module.cellMaxVoltage_mV     = ((uint16_t)frame->data.as_bytes[0] << 0x08) | ((uint16_t)frame->data.as_bytes[1]);
        module.cellWithMaxVoltage_mV = frame->data.as_bytes[2];
        module.cellMinVoltage_mV     = ((uint16_t)frame->data.as_bytes[3] << 0x08) | ((uint16_t)frame->data.as_bytes[4]);
        module.cellWithMinVoltage_mV = frame->data.as_bytes[5];
        break;
      }

      case Daly::Frame::Command::MinMaxTemperature: {
        module.cellMaxTemp_C      = (int16_t)frame->data.as_bytes[0] - 40;
        module.cellWithMaxTemp_C  = frame->data.as_bytes[1];
        module.cellMinTemp_C      = (int16_t)frame->data.as_bytes[2] - 40;
        module.cellWithMinTemp_C  = frame->data.as_bytes[3];
        break;
      }

      case Daly::Frame::Command::ChargeDischargeStatus: {
        module.chargeDischargeStatus  = (Module::ChargeDischargeStatus)frame->data.as_bytes[0];
        module.chargeFETsEnabled      = frame->data.as_bytes[1];
        module.dischargeFETsEnabled   = frame->data.as_bytes[2];
        module.bmsLife                = frame->data.as_bytes[3];
        module.remainingCapacity_mAh  = ((uint32_t)frame->data.as_bytes[4] << 0x18) | ((uint32_t)frame->data.as_bytes[5] << 0x10) | ((uint32_t)frame->data.as_bytes[6] << 0x08) | (uint32_t)frame->data.as_bytes[7];
        break;
      }

      case Daly::Frame::Command::BasicStatus: {
        module.numberOfCells        = frame->data.as_bytes[0];
        module.numberOfTempSensors  = frame->data.as_bytes[1];
        module.chargerEnabled       = frame->data.as_bytes[2];
        module.loadEnabled          = frame->data.as_bytes[3];
        module.stateOfDIDO          = frame->data.as_bytes[4];
        module.numberOfCycles       = ((uint16_t)frame->data.as_bytes[5] << 0x08) | (uint16_t)frame->data.as_bytes[6];
        break;
      }

      case Daly::Frame::Command::VoltagesByCell: {
        uint8_t frameNumber = frame->data.as_bytes[0];
        if (frameNumber == 0xFF) {
          break;
        }
        uint8_t firstCell = (frameNumber - 1) * 3;
        for (int i = 0; i < 3; i++) {
          module.voltageByCell_mV[firstCell + i] = ((uint16_t)frame->data.as_bytes[1 + (i << 1)] << 0x08) | (uint16_t)frame->data.as_bytes[2 + (i << 1)];
        }
        break;
      }

      case Daly::Frame::Command::TempsBySensor: {
        uint8_t frameNumber = frame->data.as_bytes[0];
        if (frameNumber == 0xFF) {
          break;
        }
        uint8_t firstCell = (frameNumber - 1) * 7;
        for (int i = 0; i < 7; i++) {
          module.tempBySensor_100mC[firstCell + i] = (int8_t)frame->data.as_bytes[i] - 40;
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
  };
}


void loop() {
  uint32_t now = millis();
  if (now < 5000) {
    return;
  }

  Task* t = taskToRunNext(now);
  if (t && port.availableToSend()) {
    t->runnable();
    t->lastRunAt = now;
    t->executeAt = now + t->intervalInMillis;
  }

  port.sendPendingBytes();
  port.receiveAvailableBytes();
}
