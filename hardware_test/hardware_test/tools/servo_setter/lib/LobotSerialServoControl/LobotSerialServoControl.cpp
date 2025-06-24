#include "LobotSerialServoControl.h"

LobotSerialServoControl::LobotSerialServoControl(HardwareSerial &serial, int dePin, int rePin) {
  _serial = &serial;
  _dePin = dePin;
  _rePin = rePin;
}

void LobotSerialServoControl::OnInit() {
  pinMode(_dePin, OUTPUT);
  pinMode(_rePin, OUTPUT);
  digitalWrite(_dePin, LOW);
  digitalWrite(_rePin, LOW);
}

void LobotSerialServoControl::SendBuf(uint8_t* buf, int len) {
  digitalWrite(_rePin, HIGH);
  digitalWrite(_dePin, HIGH);
  delayMicroseconds(10);
  _serial->write(buf, len);
  _serial->flush();
  delayMicroseconds(10);
  digitalWrite(_dePin, LOW);
  digitalWrite(_rePin, LOW);
}

uint8_t LobotSerialServoControl::CheckSum(uint8_t* buf, int len) {
  uint16_t sum = 0;
  for (int i = 2; i < len - 1; i++) {
    sum += buf[i];
  }
  return (~sum) & 0xFF;
}

void LobotSerialServoControl::LobotSerialServoMove(int ID, int Position, int Time) {
  uint8_t buf[10];
  buf[0] = 0x55;
  buf[1] = 0x55;
  buf[2] = ID;
  buf[3] = 7;
  buf[4] = 1;
  buf[5] = Position & 0xFF;
  buf[6] = (Position >> 8) & 0xFF;
  buf[7] = Time & 0xFF;
  buf[8] = (Time >> 8) & 0xFF;
  buf[9] = CheckSum(buf, 10);
  SendBuf(buf, 10);
}

void LobotSerialServoControl::LobotSerialServoSetID(int oldID, int newID) {
  uint8_t buf[7];
  buf[0] = 0x55;
  buf[1] = 0x55;
  buf[2] = oldID;
  buf[3] = 4;
  buf[4] = 13;
  buf[5] = newID;
  buf[6] = CheckSum(buf, 7);
  SendBuf(buf, 7);
}

int LobotSerialServoControl::LobotSerialServoReadID(int ID) {
  // Basic version: send ID read request (implementation may vary)
  uint8_t buf[6];
  buf[0] = 0x55;
  buf[1] = 0x55;
  buf[2] = ID;
  buf[3] = 3;
  buf[4] = 14;
  buf[5] = CheckSum(buf, 6);
  SendBuf(buf, 6);
  // Read not implemented (you can extend it)
  return -1;
}
