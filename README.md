# 🧬 Aero-Sanitize AI

**An Industrial-Grade, Failsafe UV-C Sterilization Controller using Sensor Fusion.**
*Built for the APC Project Competition.*

---

## ⚠️ The Problem
Standard UV-C sterilization systems in hospitals use basic passive infrared (PIR) motion sensors. These sensors frequently fail to detect anesthetized or motionless patients, leading to severe radiation burns. 

## 💡 The Innovation: Sensor Fusion
Aero-Sanitize AI eliminates false negatives by requiring **dual-confirmation** before activating the 220V UV-C relay. We combine:
1. **Micro-Vibration Tracking (CDM324 Doppler Radar):** Detects breathing and heartbeats through surgical blankets.
2. **Thermal Blob Analysis (AMG8833 Thermal Camera):** Scans an 8x8 matrix for human body heat signatures.
3. **Environmental Checks:** LDR (Light Sensor) and MC-38 (Magnetic Door Switches) ensure the room is unlit and physically sealed.

*The UV-C lamp activates ONLY when all four systems return a 100% CLEAR confidence score.*

---

## 🛠️ Hardware Architecture
This system is powered by an **ESP32-S3** microcontroller and features a custom analog signal processing chain.

| Component | Function | Operating Voltage |    
| :--- | :--- | :--- |
| **ESP32-S3 WROOM** | Core processing & SoftAP Dashboard | 3.3V |
| **CDM324 5.8GHz** | Doppler Radar (IF out) | 5V |
| **LM358 (x2)** | 2-Stage Signal Amplification & Comparator | 5V |
| **AMG8833** | I2C Thermal Camera | 3.3V |
| **MC-38** | Magnetic Door Reed Switches | 3.3V |
| **5V Relay (10A)** | 220V Mains Control (Fake-out LED for demo) | 5V |

> **Safety Note:** The 5V analog signal from the LM358 chain is stepped down via a resistor voltage divider to protect the 3.3V ESP32 ADC pins.

---

## 💻 Software Architecture
* **State Machine:** Non-blocking C++ architecture utilizing `millis()` instead of `delay()` to ensure the safety loop never freezes.
* **SoftAP Dashboard:** The ESP32 hosts its own localized WiFi network (192.168.4.1), allowing medical staff to monitor the room safely from their phones without requiring hospital internet infrastructure.
* **Digital Twin:** Validated in **Webots R2023b** via Python controller simulation prior to physical hardware deployment.

---

## 🚀 How to Run the Code
1. Clone this repository.
2. Open `AeroSanitize_Main.ino` in Arduino IDE 2.x.
3. Ensure the **ESP32S3 Dev Module** board is selected.
4. Install required libraries: `Adafruit_AMG88xx`, `DHT sensor library`, `RTClib`.
5. Compile and upload. 
6. Open Serial Monitor at `115200` baud rate to view the real-time sensor fusion confidence matrix.

---
*Created by **IES PES ENET'Com SBJC** for the APC Project Competition.*