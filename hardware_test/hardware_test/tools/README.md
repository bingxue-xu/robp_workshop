
## Tools

### Reset servo ID 
This tool allows you to use an ESP32 to reset servo IDs manually when an official debugging board isn't available.
## 🔧  XL-15D Servo ID Setter (PlatformIO + ESP32)

This tool uses an ESP32 board to set the ID of XL-15D serial bus servos. It flashes a small Arduino-based firmware to send serial commands over UART and configure the servo ID.

---


### 1. Install PlatformIO (if you haven't)

    sudo apt install python3-venv
    curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
    python3 get-platformio.py


### 2. Flash the firmware

    cd ~/dd2419/workshop_ws/src/hardware_test/hardware_test/tools/servo_setter
    ~/.platformio/penv/bin/platformio run --target upload 


### 3. Execute on the servo 

    ~/.platformio/penv/bin/platformio device monitor


Press the **EN** button to the left of the micro-USB connector

<img src="image.png" alt="ESP32 EN button" width="300"/>


Once see output like this, it is done 

    Startup: Setting servo ID
    Moving servo with old ID (254)
    Sending change ID command: 254 -> 3
    Moving servo with new ID (3)
    Done setting ID


### 👉 Change the newID number as needed in
 `hardware_test/hardware_test/tools/servo_setter/src/main.cpp`

**README:** [Reset servo ID with ESP32](hardware_test/tools/servo_setter/README.md)


### Reset the arm initial position
Reset the servo positions before flashing EPS32 in Hiwonder-xArm-ROS2 lib
(https://github.com/KTH-CAS-UAV/robp_arm/blob/217d123722b3326094f3bd3a047b823c8c211145/Hiwonder_xArm_ROS2/src/main.cpp#L415)
```bash
  servo5.move_time(16000, 1500);
  delay(500);
  servo4.move_time(20000, 1500);
  delay(500);
  servo3.move_time(3000, 1500);
  delay(500);
```

**Demo Video:**

<img src="test_results/videos/reset_arm_position.gif" alt="Reset Arm Position Demo" width="320"/> 



