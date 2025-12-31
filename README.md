ESP32 Smart Home Control System

This repository contains an ESP32-based control system developed as part of a smart home capstone project. The project includes the embedded firmware, a custom mobile application (using a voice identification script included in a different repo)

📌 Project Overview

The ESP32 acts as the central controller, handling sensor inputs, actuator control, and communication with a mobile application. In addition to the embedded firmware, I also developed a mobile app (AI-Prompted) and a Python-based voice identification script to enable hands-free interaction with the system.

---

**How It Works (System Level)**

-> System Initialization

ESP32 initializes GPIO, sensors, actuators, and communication interfaces.

Default safe states are applied at boot.

-> User Interaction

Commands can be sent via:

The mobile application (Recognition with design script, and transciption with google speech-to-text)

Manual commands can override automated behavior.

-> Voice Control

A Python-based voice recognition script captures audio, processes it, and maps it to recognized user (if not unknown)

Voice is then mapped to recognized commands to control the actions

Recognized commands are forwarded to the ESP32 through the app using Wifi.

-> Control Logic

The ESP32 processes incoming commands and sensor events.

Simple state-based logic ensures predictable behavior.

-> Actuation & Feedback

GPIO outputs control relays, motors, and indicators.

System status is sent back to the app for display.

-> Continuous Operation

The main loop runs non-blocking logic to maintain responsiveness.

---

**How It Works (Controller Level)**

This firmware turns the ESP32 into a Wi-Fi smart-home controller that connects sensors, actuators, and a mobile app through a simple HTTP interface.

Wi-Fi + HTTP API: the ESP32 runs a web server exposing endpoints to send commands, read system status (JSON), and check alerts (e.g. fire detection).

Sensor handling: reads a DHT11 for temperature (used for fan control), a PIR for presence indication, and a fire sensor with a latched alarm state.

Actuator control: drives room lights, a fan relay (auto or forced modes), a stepper-motor door using a small state machine (open → wait → close), plus a buzzer and alert LED for fire events.

Control logic: combines manual inputs (buttons, app commands) with automatic behavior (temperature-based fan, timed door closing) in a non-blocking main loop.

---

**Mobile Application (AI-Prompted/not included)**

Custom-built app used to send commands and receive system status

Includes google audio transcription

Includes Python script for facial recognition

Provides manual control and system monitoring

Designed quickly to validate end-to-end functionality

-- Demo Video: https://youtu.be/zyLBUapeSAU /// Voice Identification Repository: https://github.com/ODaal/Voice-Authentication

---

**Technologies Used**

ESP32 (C / Arduino framework)

Python (voice recognition script)

GPIO, timers, state-based logic

Wireless communication (Wi-Fi)

---

📄 Notes:
This is a section of my group's capstone project, a demo video is included.
