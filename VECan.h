namespace Victron {

extern void encode0x351(CAN_message_t* m, uint16_t batteryHighVoltage_100mV, uint16_t chargeCurrentLimit_100mA, uint16_t dischargeCurrentLimit_100mA, uint16_t batteryLowVoltage_100mV);
extern void encode0x355(CAN_message_t* m, uint8_t stateOfCharge, uint8_t stateOfHealth);
extern void encode0x356(CAN_message_t* m, uint16_t batteryVoltage_10mV, uint16_t current_100mA, int16_t temperature_100mC);
extern void encode0x35A(CAN_message_t* m);
extern void encode0x35E(CAN_message_t* m, char manufacturerName[8]);
extern void encode0x370(CAN_message_t* m, char bmsName[8]);
extern void encode0x372(CAN_message_t* m, uint16_t numberOfModules);
extern void encode0x379(CAN_message_t* m, uint16_t batteryCapacity_Ah);

};
