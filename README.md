🚗 STM32 CAN-Based Smart Lighting & Speed Alert System

Distributed Embedded System using CAN + I2C + ADC
Built on STM32F405RGT6

📌 Project Overview

This project implements a distributed automotive-style embedded system using two STM32F405RGT6 boards communicating over CAN bus (125 kbps).

Board A (Transmitter ECU) → Sensor Node

Board B (Receiver ECU) → Actuator Node

The system simulates a simplified Body Control Module (BCM) architecture where sensor data is transmitted over CAN and decoded to control lighting and safety alerts.

🏗 System Architecture
🔹 Transmitter ECU (Board A)

Functions:

Reads ambient light via I2C sensor (100 kHz)

Reads potentiometer via 12-bit ADC

Maps ADC value to speed range (0–200 km/h)

Executes decision logic

Transmits CAN frame (Std ID: 0x123)

Peripherals Used:

I2C2

ADC1

CAN1

GPIO

🔹 Receiver ECU (Board B)

Functions:

Receives CAN frame using hardware filter

Controls:

Low Beam

High Beam

Buzzer (speed warning logic)

Speed Warning Logic:

Speed ≥ 80 km/h → Single warning beep

Speed ≥ 120 km/h → Continuous buzzer

⚙️ Technical Configuration
Parameter	Configuration
MCU	STM32F405RGT6
System Clock	16 MHz (HSI)
CAN Bitrate	125 kbps
CAN Prescaler	8
Time Segment 1	13 TQ
Time Segment 2	2 TQ
I2C Speed	100 kHz
ADC Resolution	12-bit
CAN Bitrate Calculation
Bitrate = 16 MHz / (Prescaler × (1 + BS1 + BS2))
Bitrate = 16 MHz / (8 × 16)
Bitrate = 125 kbps

📦 CAN Frame Structure
Byte	Data
0	Low Beam Status (0/1)
1	High Beam Status (0/1)
2	Speed (0–200 km/h)
3–7	Reserved
🌙 Light Control Logic
Condition	Action
Lux > 500	Lights OFF
250 < Lux ≤ 500	Low Beam ON
Lux ≤ 250	Low Beam ON + High Beam conditional

High beam is enabled at night when speed > 90 km/h.

🚨 Speed Warning Logic

On Receiver:

If speed ≥ 120 → Continuous buzzer

If 80 ≤ speed < 120 → Single beep (edge detected)

Else → Buzzer OFF

🧠 Key Embedded Concepts Demonstrated

CAN bit timing configuration

Hardware CAN filtering (Mask mode)

Interrupt-driven CAN reception

I2C sensor interfacing

12-bit ADC scaling

Real-time decision logic

Distributed embedded architecture

Edge detection for event-based alerting

🔍 Notable Implementation Details

CAN RX handled using HAL_CAN_RxFifo0MsgPendingCallback

Filter configured to accept only Std ID 0x123

ADC scaled using:

speed = (adc_value * 200) / 4095;


I2C sensor operated in continuous high-resolution mode

Speed warning logic avoids repeated single beep using previous state tracking

🚀 Future Improvements

Replace blocking delays with timer-based state machine

Add rolling counter in CAN payload

Implement CAN bus-off recovery

Add hysteresis to light thresholds

Migrate from HAL to bare-metal implementation

Add fault detection and timeout handling

📂 Repository Structure
/Transmitter
   ├── Core
   ├── Drivers
   └── main.c

/Receiver
   ├── Core
   ├── Drivers
   └── main.c

🧪 Applications

Automotive lighting control systems

Distributed ECU communication

CAN-based sensor networks

Embedded systems learning project

📈 What This Project Demonstrates

This project reflects real-world automotive embedded design principles:

Modular ECU separation

CAN-based communication

Interrupt-driven firmware

Deterministic peripheral configuration

Multi-protocol integration

👨‍💻 Author

Davidkumar
Embedded Systems Enthusiast
Focused on STM32, CAN, Automotive Firmware, and Bare-Metal Development
