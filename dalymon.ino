

typedef struct __attribute__((packed)) {
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
    Faults                = 0x77, // FIXME
  } command;
  uint8_t data_length;
  union {
    uint8_t as_bytes[0x08];
  } data;
  uint8_t checksum;
} Packet;

uint8_t compute_checksum(uint8_t* buffer, size_t length) {
  uint8_t* p = buffer;
  uint8_t* pe = buffer + length;
  uint8_t checksum = 0;
  while (p < pe) {
    checksum += *p++;
  }
  return checksum;
}

void Packet_clear(Packet* p) {
  bzero(p, sizeof(Packet));
}

void Packet_reset(Packet* p, Packet::Address address, Packet::Command command) {
  Packet_clear(p);
  p->magic = 0xA5;
  p->address = address;
  p->command = command;
  p->data_length = 0x08;
}

uint8_t Packet_computeChecksum(Packet* self) {
  // Compute the checksum for every byte of the packet except the last one, the checksum byte itself.
  return compute_checksum((uint8_t*)self, sizeof(Packet) - 1);
}

void Packet_applyChecksum(Packet* self) {
  self->checksum = Packet_computeChecksum(self);
}

void Packet_printToStream(Packet* self, Stream* stream) {
  for (size_t i = 0; i < sizeof(Packet); i++) {
    char buffer[0x10];
    
    sprintf(buffer, "%02x", *(((char*)self) + i));
    stream->print(buffer);
    stream->print(" ");
  }
}


typedef struct DalyBMS {
  typedef void PacketReceiver(Packet* packet, struct DalyBMS* fromBMS);
  Packet packet;
  enum ReceiveState : uint8_t {
    WaitingForPacketToStart,
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
  PacketReceiver* onPacketReceived;
} DalyBMS;

void DalyBMS_init(DalyBMS* self) {
  bzero(self, sizeof(*self));
}

void DalyBMS_receivetheByte(DalyBMS* self, const uint8_t theByte) {
  switch (self->receiveState) {
    case DalyBMS::WaitingForPacketToStart: {
      if (0xA5 != theByte) {
        break;
      }
      
      self->receiveState = DalyBMS::WaitingForAddress;
      self->receiveError = DalyBMS::None;
      Packet_clear(&self->packet);
      self->packet.magic = theByte;
      break;
    }

    case DalyBMS::WaitingForAddress: {
      switch (theByte) {
        case Packet::Address::BMS:
        case Packet::Address::HOST: {
          /* Valid */
          break;
        }

        default: {
          self->receiveError = DalyBMS::UnsupportedAddress;
          break;
        }
      }
      
      self->packet.address = (Packet::Address)theByte;
      self->receiveState = DalyBMS::WaitingForCommand;
      break;
    }

    case DalyBMS::WaitingForCommand: {
      switch (theByte) {
        case Packet::Command::VoltageAndCurrent:
        case Packet::Command::MinMaxVoltage:
        case Packet::Command::MinMaxTemperature:
        case Packet::Command::ChargeDischargeStatus:
        case Packet::Command::BasicStatus:
        case Packet::Command::VoltagesByCell:
        case Packet::Command::TempsBySensor:
        case Packet::Command::BalancerStatus:
        case Packet::Command::Faults:
          /* Valid */
          break;
          
        default: {
          self->receiveError = DalyBMS::UnsupportedCommand;
          break;
        }
      }

      self->packet.command = (Packet::Command)theByte;
      self->receiveState = DalyBMS::WaitingForDataLength;
      break;
    }

    case DalyBMS::WaitingForDataLength: {
      if (theByte != sizeof(Packet::data)) {
        self->packet.data_length = theByte;
        self->receiveError = DalyBMS::InvalidDataLength;
        self->receiveState = DalyBMS::WaitingForPacketToStart;
        break;
      }

      self->expectedDataBytes = theByte;
      self->packet.data_length = 0;
      self->receiveState = DalyBMS::ReadingData;
      break;
    }

    case DalyBMS::ReadingData: {
      if (self->packet.data_length < sizeof(self->packet.data)) {
        self->packet.data.as_bytes[self->packet.data_length++] = theByte;
      } else {
        self->receiveError = DalyBMS::DataOverflow;
      }

      if (self->packet.data_length == self->expectedDataBytes) {
        self->receiveState = DalyBMS::WaitingForChecksum;
      }
      break;
    }

    case DalyBMS::WaitingForChecksum: {
      if (DalyBMS::None == self->receiveError) {
        self->packet.checksum = theByte;
        if (theByte != Packet_computeChecksum(&self->packet)) {
          self->receiveError = DalyBMS::InvalidChecksum;
        } else {
          if (self->onPacketReceived) {
            self->onPacketReceived(&self->packet, self);
          }
        }
      }
      self->receiveState = DalyBMS::WaitingForPacketToStart;
      break;
    }
  }
}

void DalyBMS_readAvailableBytesFromStream(DalyBMS* self, Stream* stream) {
  while (stream->available()) {
    DalyBMS_receivetheByte(self, stream->read());
  }
}

void DalyBMS_sendCommandToStream(DalyBMS* self, Packet::Command command, Stream* stream) {
  Packet packet;
  Packet_reset(&packet, Packet::Address::HOST, command);
  Packet_applyChecksum(&packet);
  stream->write((byte*)&packet, sizeof(packet));
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
  
} State;



#define MONITOR  Serial
#define BMS_UART Serial1

typedef struct Task {
  Packet::Command commandToRun;
  uint16_t intervalInMillis;
  uint32_t executeAt;
  uint32_t lastRunAt;
  uint32_t lastReplyAt;
} Task;

Task tasks[] = {
  { Packet::Command::VoltagesByCell,    1000 },
  { Packet::Command::VoltageAndCurrent, 1000 },
  { Packet::Command::BasicStatus,       2000 },
//  { Packet::Command::Faults,            200  },
  { /* EMPTY */ }
};

static DalyBMS bms;
static State state;

void setup() {
  MONITOR.begin(115200);
  BMS_UART.begin(9600);

  DalyBMS_init(&bms);
  bms.onPacketReceived = [](Packet* packet, DalyBMS* fromBMS) {
    uint32_t now = millis();

    Task* t = tasks;
    while (t->commandToRun) {
      if (t->commandToRun == packet->command) {
        t->lastReplyAt = now;
        break;
      }
      t++;
    }

    switch (packet->command) {
      case Packet::Command::VoltageAndCurrent: {
        state.packVoltage   = ((uint16_t)packet->data.as_bytes[0] << 0x08) | ((uint16_t)packet->data.as_bytes[1]);
        state.current       = (((int16_t)packet->data.as_bytes[4] << 0x08) | (int16_t)packet->data.as_bytes[5]) - 30000;
        state.stateOfCharge = ((uint16_t)packet->data.as_bytes[6] << 0x08) | ((uint16_t)packet->data.as_bytes[7]);
        break;
      }

      case Packet::Command::MinMaxVoltage: {
        state.cellMaxVoltage      = ((uint16_t)packet->data.as_bytes[0] << 0x08) | ((uint16_t)packet->data.as_bytes[1]);
        state.cellWithMaxVoltage  = packet->data.as_bytes[2];
        state.cellMinVoltage      = ((uint16_t)packet->data.as_bytes[3] << 0x08) | ((uint16_t)packet->data.as_bytes[4]);
        state.cellWithMinVoltage  = packet->data.as_bytes[5];
        break;
      }

      case Packet::Command::MinMaxTemperature: {
        state.cellMaxTemp       = (int16_t)packet->data.as_bytes[0] - 40;
        state.cellWithMaxTemp   = packet->data.as_bytes[1];
        state.cellMinTemp       = (int16_t)packet->data.as_bytes[2] - 40;
        state.cellWithMinTemp   = packet->data.as_bytes[3];
        break;
      }

      case Packet::Command::ChargeDischargeStatus: {
        state.chargeDischargeStatus   = (State::ChargeDischargeStatus)packet->data.as_bytes[0];
        state.chargeFETsEnabled       = packet->data.as_bytes[1];
        state.dischargeFETsEnabled    = packet->data.as_bytes[2];
        state.bmsLife                 = packet->data.as_bytes[3];
        state.remainingCapacityInMAh  = ((uint32_t)packet->data.as_bytes[4] << 0x18) | ((uint32_t)packet->data.as_bytes[5] << 0x10) | ((uint32_t)packet->data.as_bytes[6] << 0x08) | (uint32_t)packet->data.as_bytes[7];
        break;
      }

      case Packet::Command::BasicStatus: {
        state.numberOfCells       = packet->data.as_bytes[0];
        state.numberOfTempSensors = packet->data.as_bytes[1];
        state.chargerEnabled      = packet->data.as_bytes[2];
        state.loadEnabled         = packet->data.as_bytes[3];
        state.stateOfDIDO         = packet->data.as_bytes[4];
        state.numberOfCycles      = ((uint16_t)packet->data.as_bytes[5] << 0x08) | (uint16_t)packet->data.as_bytes[6];
        break;
      }

      case Packet::Command::VoltagesByCell: {
        uint8_t frameNumber = packet->data.as_bytes[0];
        if (frameNumber == 0xFF) {
          break;
        }
        uint8_t firstCell = frameNumber * 3;
        for (int i = 0; i < 3; i++) {
          state.voltageByCell[firstCell + i] = ((uint16_t)packet->data.as_bytes[1 + (i << 1)] << 0x08) | (uint16_t)packet->data.as_bytes[2 + (i << 1)];
        }
        break;
      }

      case Packet::Command::TempsBySensor: {
        uint8_t frameNumber = packet->data.as_bytes[0];
        if (frameNumber == 0xFF) {
          break;
        }
        uint8_t firstCell = frameNumber * 7;
        for (int i = 0; i < 7; i++) {
          state.tempBySensor[firstCell + i] = (int8_t)packet->data.as_bytes[i] - 40;
        }
        break;
      }

      case Packet::Command::BalancerStatus: {
        for (int i = 0; i < 6; i++) {
          state.balancerStateByCell[i] = packet->data.as_bytes[i];
        }
        break;
      }

      case Packet::Command::Faults: {
        // TODO
        break;
      }
    }

    MONITOR.print("<- ");
    Packet_printToStream(packet, &MONITOR);
    MONITOR.print("-- ");
    if (t->commandToRun) {
      MONITOR.print("RA: ");
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
