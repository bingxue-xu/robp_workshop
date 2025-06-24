#include <Arduino.h>
#include <LobotSerialServoControl.h>

#define SERVO_TX_PIN 33        // Only TX
#define SERVO_RX_PIN -1        // Not used
#define RECEIVE_ENABLE_PIN 13  // For RE (if used)
#define TRANSMIT_ENABLE_PIN 14 // For DE (if used)

HardwareSerial mySerial(2);  // UART2
LobotSerialServoControl BusServo(mySerial, RECEIVE_ENABLE_PIN, TRANSMIT_ENABLE_PIN);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("启动：设置舵机 ID");

  // Start UART2, TX only
  mySerial.begin(115200, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);

  BusServo.OnInit();
  delay(500);

  int oldID = 254;  // Broadcast ID (factory default)

  // Step 1: Move servo using old ID (broadcast)
  Serial.println("使用旧 ID (254) 移动舵机");
  BusServo.LobotSerialServoMove(oldID, 500, 1000);
  delay(1500);

  // Step 2: Change ID
  int newID = 3;
  Serial.print("发送更改 ID 命令：254 -> ");
  Serial.println(newID);
  BusServo.LobotSerialServoSetID(oldID, newID);
  delay(1000);

  // Step 3: Move using new ID
  Serial.println("使用新 ID (3) 移动舵机");
  BusServo.LobotSerialServoMove(newID, 250, 1000);
  delay(1500);

  Serial.println("设置 ID 完成");
}

void loop() {
  // 可选添加循环控制逻辑
}
