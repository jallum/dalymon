

namespace Daly {

typedef struct __attribute__((packed)) Frame {
  uint8_t magic;
  enum Address : uint8_t {
    BMS   = 0x01,
    HOST  = 0x40,
  } address;
  enum Command : uint8_t {
    VoltageAndCurrent     = 0x90,
    MinMaxVoltage         = 0x91,
    MinMaxTemperature     = 0x92,
    ChargeDischargeStatus = 0x93,
    BasicStatus           = 0x94,
    VoltagesByCell        = 0x95,
    TempsBySensor         = 0x96,
    BalancerStatus        = 0x97,
    FailureFlags          = 0x98,
  } command;
  uint8_t data_length;
  union {
    uint8_t as_bytes[0x08];
  } data;
  uint8_t checksum;

  Frame() {
    this->magic = 0xA5;
    this->data_length = 0x08;
  }

  Frame(Address address, Command command) : Frame() {
    this->address = address;
    this->command = command;
  }

  void clear() {
    bzero(this, sizeof(Frame));
  }

  static uint8_t compute_checksum(uint8_t* buffer, size_t length) {
    uint8_t* p = buffer;
    uint8_t* pe = buffer + length;
    uint8_t checksum = 0;
    while (p < pe) {
      checksum += *p++;
    }
    return checksum;
  }

  uint8_t computeChecksum() {
    // Compute the checksum for every byte of the frame except the last one, the checksum byte itself.
    return compute_checksum((uint8_t*)this, sizeof(Frame) - 1);
  }
  
  void applyChecksum() {
    this->checksum = computeChecksum();
  }

  void printToStream(Stream* stream) {
    for (size_t i = 0; i < sizeof(Frame); i++) {
      char buffer[0x10];
      
      sprintf(buffer, "%02x", *(((char*)this) + i));
      stream->print(buffer);
      stream->print(" ");
    }
  }

} Frame;

};



typedef struct DalyBMS {
  typedef void DalyFrameReceiver(Daly::Frame* frame, struct DalyBMS* fromBMS);
  Daly::Frame frame;
  enum ReceiveState : uint8_t {
    WaitingForFrameToStart,
    WaitingForAddress,
    WaitingForCommand,
    WaitingForDataLength,
    ReadingData,
    WaitingForChecksum
  } receiveState;
  enum ReceiveError : uint8_t {
    None = 0,
    UnsupportedAddress,
    UnsupportedCommand,
    InvalidDataLength,
    DataOverflow,
    InvalidChecksum
  } receiveError;
  uint8_t expectedDataBytes;
  DalyFrameReceiver* onFrameReceived;
} DalyBMS;

void DalyBMS_init(DalyBMS* self) {
  bzero(self, sizeof(*self));
}

void DalyBMS_receivetheByte(DalyBMS* self, const uint8_t theByte) {
  switch (self->receiveState) {
    case DalyBMS::WaitingForFrameToStart: {
      if (0xA5 != theByte) {
        break;
      }
      
      self->receiveState = DalyBMS::WaitingForAddress;
      self->receiveError = DalyBMS::None;
      self->frame.clear();
      self->frame.magic = theByte;
      break;
    }

    case DalyBMS::WaitingForAddress: {
      switch (theByte) {
        case Daly::Frame::Address::BMS:
        case Daly::Frame::Address::HOST: {
          /* Valid */
          break;
        }

        default: {
          self->receiveError = DalyBMS::UnsupportedAddress;
          break;
        }
      }
      
      self->frame.address = (Daly::Frame::Address)theByte;
      self->receiveState = DalyBMS::WaitingForCommand;
      break;
    }

    case DalyBMS::WaitingForCommand: {
      switch (theByte) {
        case Daly::Frame::Command::VoltageAndCurrent:
        case Daly::Frame::Command::MinMaxVoltage:
        case Daly::Frame::Command::MinMaxTemperature:
        case Daly::Frame::Command::ChargeDischargeStatus:
        case Daly::Frame::Command::BasicStatus:
        case Daly::Frame::Command::VoltagesByCell:
        case Daly::Frame::Command::TempsBySensor:
        case Daly::Frame::Command::BalancerStatus:
        case Daly::Frame::Command::FailureFlags:
          /* Valid */
          break;
          
        default: {
          self->receiveError = DalyBMS::UnsupportedCommand;
          break;
        }
      }

      self->frame.command = (Daly::Frame::Command)theByte;
      self->receiveState = DalyBMS::WaitingForDataLength;
      break;
    }

    case DalyBMS::WaitingForDataLength: {
      if (theByte != sizeof(Daly::Frame::data)) {
        self->frame.data_length = theByte;
        self->receiveError = DalyBMS::InvalidDataLength;
        self->receiveState = DalyBMS::WaitingForFrameToStart;
        break;
      }

      self->expectedDataBytes = theByte;
      self->frame.data_length = 0;
      self->receiveState = DalyBMS::ReadingData;
      break;
    }

    case DalyBMS::ReadingData: {
      if (self->frame.data_length < sizeof(self->frame.data)) {
        self->frame.data.as_bytes[self->frame.data_length++] = theByte;
      } else {
        self->receiveError = DalyBMS::DataOverflow;
      }

      if (self->frame.data_length == self->expectedDataBytes) {
        self->receiveState = DalyBMS::WaitingForChecksum;
      }
      break;
    }

    case DalyBMS::WaitingForChecksum: {
      if (DalyBMS::None == self->receiveError) {
        self->frame.checksum = theByte;
        if (theByte != self->frame.computeChecksum()) {
          self->receiveError = DalyBMS::InvalidChecksum;
        } else {
          if (self->onFrameReceived) {
            self->onFrameReceived(&self->frame, self);
          }
        }
      }
      self->receiveState = DalyBMS::WaitingForFrameToStart;
      break;
    }
  }
}

void DalyBMS_readAvailableBytesFromStream(DalyBMS* self, Stream* stream) {
  while (stream->available()) {
    DalyBMS_receivetheByte(self, stream->read());
  }
}

void DalyBMS_sendCommandToStream(DalyBMS* self, Daly::Frame::Command command, Stream* stream) {
  Daly::Frame frame(Daly::Frame::Address::HOST, command);
  frame.applyChecksum();
  stream->write((byte*)&frame, sizeof(frame));
  stream->flush();
}


typedef struct State {
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
      Still = 0,
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
  uint8_t balancerStateByCell[0x30 / 8];

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
} State;



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

static DalyBMS bms;
static State state;

void setup() {
  MONITOR.begin(115200);
  BMS_UART.begin(9600);

  DalyBMS_init(&bms);
  bms.onFrameReceived = [](Daly::Frame* frame, DalyBMS* fromBMS) {
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
        state.packVoltage   = ((uint16_t)frame->data.as_bytes[0] << 0x08) | ((uint16_t)frame->data.as_bytes[1]);
        state.current       = (((int16_t)frame->data.as_bytes[4] << 0x08) | (int16_t)frame->data.as_bytes[5]) - 30000;
        state.stateOfCharge = ((uint16_t)frame->data.as_bytes[6] << 0x08) | ((uint16_t)frame->data.as_bytes[7]);
        break;
      }

      case Daly::Frame::Command::MinMaxVoltage: {
        state.cellMaxVoltage      = ((uint16_t)frame->data.as_bytes[0] << 0x08) | ((uint16_t)frame->data.as_bytes[1]);
        state.cellWithMaxVoltage  = frame->data.as_bytes[2];
        state.cellMinVoltage      = ((uint16_t)frame->data.as_bytes[3] << 0x08) | ((uint16_t)frame->data.as_bytes[4]);
        state.cellWithMinVoltage  = frame->data.as_bytes[5];
        break;
      }

      case Daly::Frame::Command::MinMaxTemperature: {
        state.cellMaxTemp       = (int16_t)frame->data.as_bytes[0] - 40;
        state.cellWithMaxTemp   = frame->data.as_bytes[1];
        state.cellMinTemp       = (int16_t)frame->data.as_bytes[2] - 40;
        state.cellWithMinTemp   = frame->data.as_bytes[3];
        break;
      }

      case Daly::Frame::Command::ChargeDischargeStatus: {
        state.chargeDischargeStatus   = (State::ChargeDischargeStatus)frame->data.as_bytes[0];
        state.chargeFETsEnabled       = frame->data.as_bytes[1];
        state.dischargeFETsEnabled    = frame->data.as_bytes[2];
        state.bmsLife                 = frame->data.as_bytes[3];
        state.remainingCapacityInMAh  = ((uint32_t)frame->data.as_bytes[4] << 0x18) | ((uint32_t)frame->data.as_bytes[5] << 0x10) | ((uint32_t)frame->data.as_bytes[6] << 0x08) | (uint32_t)frame->data.as_bytes[7];
        break;
      }

      case Daly::Frame::Command::BasicStatus: {
        state.numberOfCells       = frame->data.as_bytes[0];
        state.numberOfTempSensors = frame->data.as_bytes[1];
        state.chargerEnabled      = frame->data.as_bytes[2];
        state.loadEnabled         = frame->data.as_bytes[3];
        state.stateOfDIDO         = frame->data.as_bytes[4];
        state.numberOfCycles      = ((uint16_t)frame->data.as_bytes[5] << 0x08) | (uint16_t)frame->data.as_bytes[6];
        break;
      }

      case Daly::Frame::Command::VoltagesByCell: {
        uint8_t frameNumber = frame->data.as_bytes[0];
        if (frameNumber == 0xFF) {
          break;
        }
        uint8_t firstCell = frameNumber * 3;
        for (int i = 0; i < 3; i++) {
          state.voltageByCell[firstCell + i] = ((uint16_t)frame->data.as_bytes[1 + (i << 1)] << 0x08) | (uint16_t)frame->data.as_bytes[2 + (i << 1)];
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
          state.tempBySensor[firstCell + i] = (int8_t)frame->data.as_bytes[i] - 40;
        }
        break;
      }

      case Daly::Frame::Command::BalancerStatus: {
        for (int i = 0; i < 6; i++) {
          state.balancerStateByCell[i] = frame->data.as_bytes[i];
        }
        break;
      }

      case Daly::Frame::Command::FailureFlags: {
        memcpy(&state.failureFlags, frame->data.as_bytes, sizeof(state.failureFlags));
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
      DalyBMS_sendCommandToStream(&bms, t->commandToRun, &BMS_UART);
      t->lastRunAt = now;
      t->executeAt = now + t->intervalInMillis;
      break;
    }
  }
  
  DalyBMS_readAvailableBytesFromStream(&bms, &BMS_UART);
}
