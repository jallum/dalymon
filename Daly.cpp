#include <Arduino.h>
#include "Daly.h"

namespace Daly {

static uint8_t compute_checksum(uint8_t* buffer, size_t length) {
  uint8_t* p = buffer;
  uint8_t* pe = buffer + length;
  uint8_t checksum = 0;
  while (p < pe) {
    checksum += *p++;
  }
  return checksum;
}

Frame::Frame() {
  this->magic = 0xA5;
  this->dataLength = 0x08;
}

Frame::Frame(Address address, Command command) : Frame() {
  this->address = address;
  this->command = command;
}

void Frame::clear() {
  bzero(this, sizeof(Frame));
}

void Frame::reset(Address address, Command command) {
  clear();
  this->magic = 0xA5;
  this->address = address;
  this->command = command;
  this->dataLength = 0x08;
}

uint8_t Frame::computeChecksum() {
  // Compute the checksum for every byte of the frame except the last one, the checksum byte itself.
  return compute_checksum((uint8_t*)this, sizeof(Frame) - 1);
}

void Frame::applyChecksum() {
  this->checksum = computeChecksum();
}

void Frame::printToStream(Stream* stream) {
  for (size_t i = 0; i < sizeof(Frame); i++) {
    char buffer[0x10];
    
    sprintf(buffer, "%02x", *(((char*)this) + i));
    stream->print(buffer);
    stream->print(" ");
  }
}

/**/

void Protocol::receiveByte(const uint8_t theByte) {
  switch (this->receiveState) {
    case ReceiveState::WaitingForFrameToStart: {
      if (0xA5 != theByte) {
        break;
      }
      
      this->receiveState = ReceiveState::WaitingForAddress;
      this->receiveError = Error::None;
      this->frame.clear();
      this->frame.magic = theByte;
      break;
    }

    case ReceiveState::WaitingForAddress: {
      switch (theByte) {
        case Frame::Address::BMS:
        case Frame::Address::HOST: {
          /* Valid */
          break;
        }

        default: {
          this->receiveError = Error::UnsupportedAddress;
          break;
        }
      }
      
      this->frame.address = (Frame::Address)theByte;
      this->receiveState = ReceiveState::WaitingForCommand;
      break;
    }

    case ReceiveState::WaitingForCommand: {
      switch (theByte) {
        case Frame::Command::NominalCapacityAndCellVoltage:
        case Frame::Command::AlarmThresholdsForPackVoltage:
        case Frame::Command::AlarmThresholdsForPackCurrent:
        case Frame::Command::VoltageAndCurrent:
        case Frame::Command::MinMaxVoltage:
        case Frame::Command::MinMaxTemperature:
        case Frame::Command::ChargeDischargeStatus:
        case Frame::Command::BasicStatus:
        case Frame::Command::VoltagesByCell:
        case Frame::Command::TempsBySensor:
        case Frame::Command::BalancerStatus:
        case Frame::Command::FailureFlags:
          /* Valid */
          break;
          
        default: {
          this->receiveError = Error::UnsupportedCommand;
          break;
        }
      }

      this->frame.command = (Frame::Command)theByte;
      this->receiveState = ReceiveState::WaitingForDataLength;
      break;
    }

    case ReceiveState::WaitingForDataLength: {
      if (theByte != sizeof(Frame::data)) {
        this->frame.dataLength = theByte;
        this->receiveError = Error::InvalidDataLength;
        this->receiveState = WaitingForFrameToStart;
        break;
      }

      this->expectedDataBytes = theByte;
      this->frame.dataLength = 0;
      this->receiveState = ReceiveState::ReadingData;
      break;
    }

    case ReceiveState::ReadingData: {
      if (this->frame.dataLength < sizeof(this->frame.data)) {
        this->frame.data.as_bytes[this->frame.dataLength++] = theByte;
      } else {
        this->receiveError = Error::DataOverflow;
      }

      if (this->frame.dataLength == this->expectedDataBytes) {
        this->receiveState = ReceiveState::WaitingForChecksum;
      }
      break;
    }

    case ReceiveState::WaitingForChecksum: {
      if (Error::None == this->receiveError) {
        this->frame.checksum = theByte;
        if (theByte != this->frame.computeChecksum()) {
          this->receiveError = Error::InvalidChecksum;
        } else {
          if (this->onFrameReceived) {
            this->onFrameReceived(&this->frame, this);
          }
        }
      }
      this->receiveState = ReceiveState::WaitingForFrameToStart;
      break;
    }
  }
}

/**/

UARTPort::UARTPort(Stream* uart, Protocol* protocol) {
  this->uart = uart;
  this->protocol = protocol;
}

void UARTPort::sendCommand(Daly::Frame::Command command) {
  this->frame.reset(Frame::Address::HOST, command);
  this->frame.applyChecksum();
  this->bytesLeftToSend = sizeof(Frame);
  this->sendPendingBytes();
}

void UARTPort::receiveAvailableBytes() {
  while (this->uart->available()) {
    protocol->receiveByte(this->uart->read());
  }
}

size_t UARTPort::sendPendingBytes() {
  if (this->bytesLeftToSend == 0) {
    return 0;
  }
  
  int offset = sizeof(Frame) - this->bytesLeftToSend;
  int count = uart->write(((byte*)&frame) + offset, min(this->bytesLeftToSend, this->uart->availableForWrite()));
  this->bytesLeftToSend -= count;

  return count;
}

bool UARTPort::availableToSend() {
  return (this->bytesLeftToSend == 0) && this->uart->availableForWrite();
}

};
