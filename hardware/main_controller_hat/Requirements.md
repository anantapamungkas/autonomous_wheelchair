# Hardware Requirements & System Architecture Document

**Project Name:** EV Master Controller  
**Document Owner:** Ananta Pamungkas  
**Date:** 20-07-2026  
**Version:** 1.0

---

## 1. Executive Summary
A custom embedded master controller designed for an autonomous electric vehicle platform. This system acts as the central hub, bridging high-level global and local path-planning processing (such as A* and DWA) executed on a Jetson Orin with a JUKEN 10 ECU. It integrates robust 12-24V power distribution, critical telemetry sensing, and high-speed communication routing to ensure safe, modular, and reliable autonomous navigation.

## 2. System Objectives
*   **Goal 1:** Reliably interface with and execute commands from autonomous navigation and control algorithms.
*   **Goal 2:** Safely manage power distribution from a 12-24V DC main battery source.
*   **Goal 3:** Provide a modular communication architecture utilizing CAN Bus, I2C, SPI, and UART interfaces.
*   **Goal 4:** Safely sequence and control the power-up states for the Jetson Orin NX and JUKEN 10 ECU.
*   **Goal 5:** Act as the master control interface for the JUKEN 10 ECU.

## 3. Functional Requirements
### 3.1 Core Processing & Firmware
*   **Microcontroller:** ESP32 (Dual-core)
*   **Processing Needs:** Minimum 240MHz clock speed to utilize an RTOS, ensuring timing-critical JUKEN 10 serial communications are not blocked by Bluetooth polling or Jetson UART parsing.
*   **Memory/Storage:** Standard 4MB or 8MB Flash memory layout, providing sufficient space for OTA firmware updates and local state-machine logging.

### 3.2 Power Management & Distribution
*   **Primary Power Source:** 12-24V DC Main Pack
*   **Regulated Voltage Rails:** 
    *   3.3V for ESP32 and logic/sensors
    *   5V for logic-level shifting and external peripherals
    *   12V for control signals and JUKEN 10 interfacing
*   **Circuit Protection:** Reverse polarity protection, overcurrent fusing, solid-state thermal shutdown, and TVS diodes for ESD protection on all external connectors.

### 3.3 Sensing & Inputs (Peripherals)
*   **Battery Telemetry:** Current and voltage sensing for battery discharge monitoring.
*   **Thermal Monitoring:** PCB temperature sensor (Si7021-A20 via I2C).
*   **Motor Feedback:** Brushless DC (BLDC) Hall Sensor inputs.
*   **High-Level Data:** Serial or Ethernet data stream from the Jetson Orin NX.
*   **ECU Logic Inputs (JUKEN 10):** 
    *   Stop lamp state (5V - 12V logic)
    *   Engine lamp state (12V logic)
    *   Gauge data cluster telemetry

### 3.4 Actuation & Outputs
*   **Starter Control:** Starter switch signal (5V - 12V logic output to JUKEN 10).
*   **Speed Selection:** Speed selector switch array (5 states, 5V - 12V logic output to JUKEN 10).
*   **Safety Interlock:** Side stand switch signal (1x, 5V - 12V logic output to JUKEN 10).
*   **Motor Control:** Throttle Position Sensor (TPS) control signal sent to the JUKEN 10 (via DAC/PWM or designated protocol).

## 4. Non-Functional Requirements
*   **Physical Footprint:** Designed to fit within a standard waterproof enclosure (e.g., IP65 rated) suitable for EV mounting. 
*   **Thermal Constraints:** Passive cooling required; critical heat-generating components (like the 12-24V buck converters supplying the Jetson) must be thermally coupled to a ground plane or external heatsink.
*   **Operating Environment:** Must withstand vibration profiles typical of an electric wheelchair or light EV chassis, requiring automotive-grade connectors (e.g., JST-JWPF or Molex MX150) rather than standard pin headers.

## 5. System Architecture
### 5.1 High-Level Block Diagram
*[Placeholder: A block diagram showing the ESP32 acting as the central router. The 12-24V DC input branches into the 12V, 5V, and 3.3V regulators. The ESP32 connects to the Jetson Orin via UART/Ethernet, to the JUKEN 10 via heavily protected 5-12V logic-level shifters, and to the localized and steering PCBs via a CAN transceiver.]*

### 5.2 Communication Buses & Bandwidth
*   **I2C:** Si7021-A20 (Temperature & Humidity).
*   **SPI:** [Reserved for potential external SD Card logging or high-speed sensor additions].
*   **UART/Serial:** Jetson Orin NX (High-speed routing), JUKEN 10 (ECU telemetry).
*   **CAN Bus:** Localization PCB, Steering Controller PCB.
*   **Wireless:** PS3 Controller (via ESP32 native Bluetooth).

## 6. Subsystem Validation & Prototyping Plan
*   **Simulation/Modeling (LTSpice):** 
    *   Throttle control signal (TPS) smoothing and DAC filtering.
    *   Reverse polarity and overvoltage protection transient responses.
    *   ESD protection clamping voltages.
    *   Buck converter ripple and switching noise analysis.
*   **Hardware Breadboard Testing:** 
    *   Validating the 5V/12V logic level shifting for the JUKEN 10 interfaces.
    *   Parsing JUKEN 10 UART data and Gauge control signal reception.
    *   Verifying the TPS data transmission from the ESP32 master to the JUKEN 10 ECU.