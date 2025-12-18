# Project-Haligi-Primary-Flight-Display-Simulation
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
