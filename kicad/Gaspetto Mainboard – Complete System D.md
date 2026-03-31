
# COMPLETE DESCRIPTION OF THE GASPETTO SYSTEM
**Version:** 1.3
**Date:** March 31, 2026
**Subject:** Mainboard Architecture & Power Management

---


## 1. GENERAL ARCHITECTURE
Perfboard mainboard controlling an **intelligent LED cover** with ultra-low power management and wireless communication. The system is designed to run on battery with charging via an external USB-C port.

> **Power flow:** > USB-C (External) ➔ UPS Module ➔ Battery ➔ Mainboard ➔ LED Cover + Sensors

---


## 2. EXTERNAL POWER AND BATTERY

### Waterproof USB-C Connector (Panel Mount)
| Pin | Signal | Destination | Function |
| :--- | :--- | :--- | :--- |
| **VBUS** | 5V | UPS Input | Wall charger power |
| **GND** | Ground | Common | System reference |
| **D+/D-** | Data | PA11 / PA12 | STM32 programming without opening |
| **CC1/CC2**| 5.1k Res. | to GND | USB-C charge negotiation |

### UPS Module (Charger + 5V Boost)
* **Input:** 2-pin connector (5V/GND) from USB-C.
* **Battery:** LiPo connected to **B+/B-** terminals.
* **Output:** Stable 5V to **J3** connector (`5V_ALWAYS` rail).
* ⚠️ **WARNING:** No *Power Path*. Prefer battery operation.

---


## 3. ALWAYS-ON SYSTEM

### 3.3V LDO Regulator (U5 - TO-92)
* **Input:** `5V_ALWAYS` | **Output:** `3V3_ALWAYS`.
* **Filtering:** 1µF (input) / 10µF (output) ceramic.
* **Role:** Permanent power for MCU and radio (standby consumption < 1mA).

### STM32F103C8T6 Microcontroller (BluePill)
* **Central brain:** In low-power mode, manages radio listening, wake button, MOSFET control, I2C, and LED signal.

### nRF24L01+ Radio Module
* **SPI1 Interface:** PA5 (SCK), PA6 (MISO), PA7 (MOSI).
* **Control:** PB0 (CE), PB1 (CSN).
* **Wake-up:** PA0 (IRQ) for immediate MCU wake-up.

---


## 4. SWITCHABLE POWER MANAGEMENT (Load Control)

### LED Switch (Q1 - P-MOSFET)
* **Component:** AO3401A (SOT-23).
* **Control:** Pin **PB8** (LOW = ON).
* **Output:** `5V_SWITCHED` (To J4 Pin 1).
* **Startup:** Soft-start (10kΩ/100nF) to avoid voltage drops.

### Sensors/OLED Switch (Q2 - P-MOSFET)
* **Preparation:** Second 3.3V LDO (`3V3_PRE_SWITCHED`) with **220µF** reservoir.
* **Control:** Pin **PB9** (0V = ON).
* **Output:** `3V3_SWITCHED` (To J4 Pin 2).

---


## 5. EXTERNAL INTERFACE (J4 Connector - IDC 10)

| Pin | Signal | STM32 | Function |
| :--- | :--- | :--- | :--- |
| **1** | 5V_SWITCHED | - | LED Cover Power (Q1) |
| **2** | 3V3_SWITCHED | - | Sensors + OLED Power (Q2) |
| **3** | GND | - | Common ground |
| **4** | I2C1_SDA | PB7 | Main I2C Data |
| **5** | I2C1_SCL | PB6 | Main I2C Clock |
| **6** | GND | - | Common ground |
| **7** | I2C2_SDA | PB11 | Secondary I2C Data |
| **8** | I2C2_SCL | PB10 | Secondary I2C Clock |
| **9** | WAKE_BUTTON | PA2 | External wake button |
| **10** | LED_DATA | PA1 | WS2812B signal |

---

    LED Strip Connector:
    +5V → J4 Pin 1 (5V_SWITCHED)
    GND → J4 Pins 3+6
    DIN → J4 Pin 10 (PA1 data signal)

## 6. TEST AND DEBUG INFRASTRUCTURE

### Test Points (TP)
* **TP_GND:** Reference ground.
* **TP_5V_ALWAYS:** UPS output (5.0V).
* **TP_3V3_ALWAYS:** MCU/Radio power (3.3V).
* **TP_5V_SWITCHED:** State of LED MOSFET.
* **TP_3V3_SWITCHED:** State of Sensors MOSFET.

### I2C Debug Header (5 pins)
`[GND] [SCL1] [SDA1] [SCL2] [SDA2]`

---


## 7. USAGE RECOMMENDATIONS
* **Battery life:** Estimated between **6 and 12 months** with a 2000mAh battery (based on 5 min activity/day).
* **Maintenance:** Avoid permanent USB use to prevent battery degradation (no power path circuit).
