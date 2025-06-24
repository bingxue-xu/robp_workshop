## 🔧  XL-15D Servo ID Setter (PlatformIO + ESP32)

This tool uses an ESP32 board to set the ID of XL-15D serial bus servos. It flashes a small Arduino-based firmware to send serial commands over UART and configure the servo ID.

---


### 1. Install PlatformIO (if you haven't)

    cd ~/Downloads
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