<img width="1536" height="1203" alt="Final" src="https://github.com/user-attachments/assets/a95eb3bb-a56c-4b8b-a4e1-62d71a69910b" />


<br />
<br /><br />A Garmin G5–style Primary Flight Display implemented on an ESP32‑WROOM‑32D using an ST7796 (320×480) TFT. The project focuses on low flicker, partial redraws, and rate‑limited rendering suitable for demos, classrooms, and student projects.<br /><br />
<br />


<img width="1655" height="2340" alt="infographics print a4-1" src="https://github.com/user-attachments/assets/60a5a708-c71c-47c6-92fc-2434945c98a1" /><br />

<img width="2000" height="1200" alt="553585888_810624955253793_4183030290552421418_n" src="https://github.com/user-attachments/assets/b65f13b6-badd-4feb-8b00-a2f674aa232b" />

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
* 2 MPU 6050
* EC11 Rotary Encoder (Breakout)

# Wiring

Adjust pins as needed; avoid ESP32 boot‑strap pins for CS/DC/RST.

<img width="2048" height="1448" alt="553508477_1042325464547451_2434230432246443031_n" src="https://github.com/user-attachments/assets/21ce001c-b8af-4b14-9fd0-9438137387ff" /><br />

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
|SW	|GPIO 27|
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
* Requires clean power for reliable startup.


# Acknowledgements
<img width="1352" height="600" alt="Screenshot 2025-12-18 223555" src="https://github.com/user-attachments/assets/2dafb0d8-6d6d-43af-9b8b-1221ad2eef28" /><br />
<img width="761" height="731" alt="Screenshot 2025-12-18 223522" src="https://github.com/user-attachments/assets/03d76905-70e3-483b-8039-3c6c174d273a" />

# ⚠️ Disclaimer
This project is NOT certified for real flight use. It is intended strictly for educational and experimental purposes only.

# 📄 Thesis & Research Notice

This project was developed as part of our undergraduate thesis in the Bachelor of Science in Aviation Electronics Technology (BSAET) at the National Aviation Academy of the Philippines (formerly PhilSCA).<br />
<br />A formal research paper based on this system has been written and presented. If you are interested in the methodology, results, or documentation, feel free to contact us.
<br />
<br />
📬 You may message us through our emails  for academic inquiries, collaboration, or access to the paper.
<br />

© 2025<br />
Denmer John E. Abrigo | denmerjohnabrigo@gmail.com <br />
John Christian A. Ante | johnchristianante@gmail.com <br />
Peter Francis O. Apostol | peterapostol75@gmail.com <br />
Sherwin M. Caballero  | sherwin.caballero@proton.me<br /> 

National Aviation Academy of the Philippines (formerly Philippine State College of Aeronautics).<br />
<br />
All rights reserved except as permitted under the Apache License 2.0.
You are free to use, modify, and distribute this code, provided that proper credit is given and the license terms are followed.
