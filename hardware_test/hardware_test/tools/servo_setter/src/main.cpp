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
  Serial.println("Startup: Setting servo ID");

  // Start UART2, TX only
  mySerial.begin(115200, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);

  BusServo.OnInit();
  delay(500);

  int oldID = 254;  // Broadcast ID (factory default)

  // Step 1: Move servo using old ID (broadcast)
  Serial.println("Moving servo with old ID (254)");
  BusServo.LobotSerialServoMove(oldID, 500, 1000);
  delay(1500);

  // Step 2: Change ID
  int newID = 3;
  Serial.print("Sending change ID command: 254 -> 3 ");
  Serial.println(newID);
  BusServo.LobotSerialServoSetID(oldID, newID);
  delay(1000);

  // Step 3: Move using new ID
  Serial.println("Moving servo with new ID (3)");
  BusServo.LobotSerialServoMove(newID, 250, 1000);
  delay(1500);

  Serial.println("Done setting ID");
}

void loop() {
}
