# Project-Haligi-Primary-Flight-Display-Simulation
A Garmin G5–style Primary Flight Display implemented on an ESP32‑WROOM‑32D using an ST7796 (320×480) TFT and TFT_eSPI. The project focuses on low flicker, partial redraws, and rate‑limited rendering suitable for demos, classrooms, and student projects.

Features

• Landscape PFD layout (480×320)
• Artificial horizon with pitch ladder & roll arc
• Heading strip with bug and numeric readout
• Airspeed tape with V‑speed color bands
• Altitude tape with windowed readout
• Vertical Speed Indicator (VSI)
• Slip/Skid (inclinometer)
• Flicker‑reduced rendering (no sprites)
• Modular draw functions and clear geometry constants

 Hardware

•MCU: ESP32‑WROOM‑32D
•Display: ST7796 TFT (SPI, 320×480)
•Power: Stable 5 V supply recommended (≥2 A for demos)
-Tested With
•TFT_eSPI library
•SPI @ 20–40 MHz (27 MHz recommended for stability)
