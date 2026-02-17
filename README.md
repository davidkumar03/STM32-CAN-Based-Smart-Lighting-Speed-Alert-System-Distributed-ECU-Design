# 🚗 STM32 CAN-Based Smart Lighting & Speed Alert System

Distributed Embedded System using CAN + I2C + ADC  
Built on STM32F405RGT6

---

## 📌 Overview

This project implements a distributed automotive-style embedded system using two STM32F405RGT6 boards communicating over CAN bus at 125 kbps.

- **Board A (Transmitter ECU)** → Sensor Node  
- **Board B (Receiver ECU)** → Actuator Node  

The system simulates a simplified Body Control Module (BCM) architecture.

---

## 🏗 System Architecture

### 🔹 Transmitter ECU
- Ambient light sensing via I2C (100 kHz)
- Potentiometer speed input via 12-bit ADC
- Real-time decision logic
- CAN transmission (Std ID: 0x123)

### 🔹 Receiver ECU
- CAN frame decoding with hardware filter
- Automatic Low/High beam control
- Speed-based buzzer alerts:
  - Speed ≥ 80 km/h → Single beep
  - Speed ≥ 120 km/h → Continuous buzzer

---

## ⚙️ Technical Configuration

| Parameter | Value |
|------------|--------|
| MCU | STM32F405RGT6 |
| System Clock | 16 MHz (HSI) |
| CAN Bitrate | 125 kbps |
| Prescaler | 8 |
| BS1 | 13 TQ |
| BS2 | 2 TQ |
| I2C Speed | 100 kHz |
| ADC Resolution | 12-bit |

---

## 📦 CAN Frame Structure

| Byte | Description |
|------|------------|
| 0 | Low Beam (0/1) |
| 1 | High Beam (0/1) |
| 2 | Speed (0–200 km/h) |
| 3–7 | Reserved |

---

## 🌙 Light Logic

| Lux Range | Action |
|-----------|--------|
| > 500 | Lights OFF |
| 250–500 | Low Beam ON |
| ≤ 250 | Low Beam ON + High Beam conditional |

High beam activates at night when speed > 90 km/h.

---

## 🚨 Speed Warning Logic

- ≥120 km/h → Continuous buzzer  
- 80–119 km/h → Single beep (edge detected)  
- <80 km/h → Buzzer OFF  

---

## 📂 Repository Structure

STM32-CAN-Smart-Lighting/
│
├── Transmitter_ECU/
│ ├── Core/
│ ├── Drivers/
│ ├── .ioc
│ └── STM32F405RGTX_FLASH.ld
│
├── Receiver_ECU/
│ ├── Core/
│ ├── Drivers/
│ ├── .ioc
│ └── STM32F405RGTX_FLASH.ld
│
└── README.md

## 🧠 Concepts Demonstrated

- CAN bit timing configuration
- Hardware CAN filtering (Mask mode)
- Interrupt-driven reception
- I2C sensor interfacing
- ADC scaling and mapping
- Distributed embedded architecture
- Edge-triggered event logic

---

## 🚀 Future Improvements

- Replace blocking delays with timer-based state machine
- Add rolling counter in CAN payload
- Implement CAN bus-off recovery
- Add hysteresis to light thresholds

---

## 👨‍💻 Author

Davidkumar  
Embedded Systems | STM32 | CAN | Automotive Firmware
