#ifndef LOBOT_SERIAL_SERVO_CONTROL_H
#define LOBOT_SERIAL_SERVO_CONTROL_H

#include "Arduino.h"

class LobotSerialServoControl {
  public:
    LobotSerialServoControl(HardwareSerial &serial, int dePin, int rePin);
    void OnInit();
    void LobotSerialServoMove(int ID, int Position, int Time);
    void LobotSerialServoSetID(int oldID, int newID);
    int  LobotSerialServoReadID(int ID);

  private:
    HardwareSerial* _serial;
    int _dePin;
    int _rePin;
    void SendBuf(uint8_t* buf, int len);
    uint8_t CheckSum(uint8_t* buf, int len);
};

#endif
