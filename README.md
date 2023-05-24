# Arduino_CAN_LED
Using Arduino to intercept CAN signals and power interior LED strips.

## Goals
The goal of this project is to intercept CAN bus signals on my 2022 Toyota GR86 and control the color and brightness of LED strip lights in the footwells. Currently, the color is determined by which gear the vehicle is in (1-6), and the brightness is determined by engine speed.

## Hardware
- Arduino Uno Equivalent
- SEEED Can Bus Shield
- Ansix Auto ASC to OBD Adapter
- OBD to DB9 Adapter
- 2x WS2812B LED Light Strips
- 2x JST SM 3pin extensions
- 220ohm Resistor
- 100uF Capacitor
- Breadboard and jumper wires

Description to come later! Project in progress

## Software
Libraries Used
- https://github.com/Seeed-Studio/Seeed_Arduino_CAN
- https://github.com/FastLED

## CAN Data
Using this project as a guide, (https://github.com/timurrrr/RaceChronoDiyBleDevice) I am able to find the correct IDs and messages to determine gear, engine speed, brake position, clutch position, accelerator position, and sport mode. 

More details to come
