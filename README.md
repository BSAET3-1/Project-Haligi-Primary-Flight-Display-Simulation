<img width="785" height="216" alt="Screenshot 2025-12-18 221615" src="https://github.com/user-attachments/assets/70366810-969d-4603-9f1e-1345083ebb07" />

# Project Haligi: MPrimary Flight Display-Simulation
A Garmin G5–style Primary Flight Display implemented on an ESP32‑WROOM‑32D using an ST7796 (320×480) TFT and TFT_eSPI. The project focuses on low flicker, partial redraws, and rate‑limited rendering suitable for demos, classrooms, and student projects.


# Features

* Landscape PFD layout (480×320)
* Artificial horizon with pitch ladder & roll arc
* Heading strip with bug and numeric readout
* Airspeed tape with V‑speed color bands
* Altitude tape with windowed readout
* Vertical Speed Indicator (VSI)
* Slip/Skid (inclinometer)
* Flicker‑reduced rendering (no sprites)
* Modular draw functions and clear geometry constants

# Hardware

* MCU: ESP32‑WROOM‑32D
* Display: ST7796 TFT (SPI, 320×480)
* Power: Stable 5 V supply recommended (≥2 A for demos)

# Wiring
Adjust pins as needed; avoid ESP32 boot‑strap pins for CS/DC/RST.

|ST7796 Hardware SPI|	ESP32 |
|-------|-------|
|SCK	|GPIO 18|
|MOSI	|GPIO 23|
|MISO	|GPIO 19 (optional)|
|CS	|GPIO 27|
|DC	|GPIO 26|
|RST	|GPIO 33|
|LED	|3 V|
|VCC	|5 V|
|GND	|GND|

|MPU 6050 (Pitch)|	ESP32 |
|-------|-------|
|VCC	|3.3V|
|GND	|GND|
|SDA	|GPIO 21|
|SCL	|GPIO 22|
|AD0	|GND → Address 0x68 (For the 1st MPU6050)|

|MPU 6050 (Roll) |	ESP32 |
|-------|-------|
|VCC	|3.3V|
|GND	|GND|
|SDA	|GPIO 21|
|SCL	|GPIO 22|
|AD0	|3.3V → Address 0x69 (For the 1st MPU6050)|

|Encoder| 	ESP32| 
|-------|-------|
|CLK (A)	|GPIO 34|
|DT (B)	|GPIO 35|
|SW|	|GPIO 27|
|GND	|GND|
|⚠️ GPIO 34 & 35 are input-only → perfect for encoders|

# Software Setup

1) Install the ST7796 MSP 4030 TFT LCD Library
2) Install the Adafruit MPU6050 Library
3) Select ESP32 Dev Module (or your exact board) in Arduino IDE.
4) Compile and upload.

# Getting Started

* Power the ESP32 and TFT.
* On boot, the static frame is drawn once, then dynamic elements update at different rates.
* The default sketch includes simulated flight data for demo/testing.


# ⚠️ Disclaimer
This project is NOT certified for real flight use. It is intended strictly for educational and experimental purposes only.

