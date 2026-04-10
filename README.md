# Tiva-WashingMachine-Control-
This repository contains the firmware developed in Embedded C for a microcontroller-based washing machine control system. The project focuses on state-machine logic, real-time user interaction, and hardware safety protocols.

For a detailed technical analysis of the system architecture, circuit diagrams, and logic flow, please refer to the Technical Report.

Language: Embedded C

Architecture: Tiva c series tm4c1294ncpdt

Hardware & Peripherals:HC-SR04 Ultrasonic Sensor: Used for non-contact water level measurement and overflow prevention, 2-Channel Relay Module: Acts as the power interface to switch the high-voltage water pump and motor actuators, Submersible Water Pump: Managed via PWM/Logic signals for the filling and draining stages, MG995 High-Torque Servo Motor: Utilized for mechanical locking systems or precise valve positioning, 16x2 LCD Display: Provides a real-time visual interface for cycle status and user prompts and 4x4 Matrix Keypad: Handles user input for cycle selection and system configuration.

The system manages the full laundry cycle by interfacing with various peripherals via GPIOs, handling asynchronous user input through a keypad, and providing real-time feedback via an integrated display.

Key Features
State-Machine Logic: The core engine is divided into 4 robust stages:

* Filling (Llenado): Manages water inlet sensors and valves.

* Washing (Lavado): Controls motor patterns and timing.

* Draining (Vaciado): Handles pump activation and water level verification.

* Spin-drying (Centrifugado): High-speed motor control for moisture extraction.

User Interface: * Keypad Integration: Allows users to select custom wash cycles.
Real-time Display: Shows current status, elapsed time, and selected options.
Safety & Reliability: * Hardware Interrupts: Implemented emergency stop routines triggered by electrical out-of-range conditions to protect the hardware and the user.
Peripheral Management: Efficient use of GPIOs to control relays, sensors, and feedback components.

