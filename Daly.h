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
  uint8_t dataLength;
  union {
    uint8_t as_bytes[0x08];
  } data;
  uint8_t checksum;

public:
  Frame();
  Frame(Address address, Command command);

  void clear();
  void reset(Address address, Command command);
  uint8_t computeChecksum();
  void applyChecksum();
  void printToStream(Stream* stream);
} Frame;


class Protocol {
  Frame frame;
  enum ReceiveState : uint8_t {
    WaitingForFrameToStart,
    WaitingForAddress,
    WaitingForCommand,
    WaitingForDataLength,
    ReadingData,
    WaitingForChecksum
  } receiveState;
  enum Error : uint8_t {
    None = 0,
    UnsupportedAddress,
    UnsupportedCommand,
    InvalidDataLength,
    DataOverflow,
    InvalidChecksum
  } receiveError;
  uint8_t expectedDataBytes;

public:
  typedef void FrameReceiver(Frame* frame, Protocol* source);
  FrameReceiver* onFrameReceived;

  void receiveByte(const uint8_t theByte);
};


class UARTPort {
  Stream* uart;
  Protocol* protocol;
  Frame frame;
  uint8_t bytesLeftToSend;
  
public:
  UARTPort(Stream* uart, Protocol* protocol);
  size_t sendPendingBytes();
  bool availableToSend();
  void sendCommand(Frame::Command command);
  void receiveAvailableBytes();
};


};
