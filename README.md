# Sat display earth

This glob display live satellite location in live globe. I created it's glob in blender and it's machinism part in onshape and i will be using **ESP32 devkit v1** to make it working.
I was inspired to make this after making my satellite tracker website.

# See my project
<img width="1920" height="1080" alt="Screenshot (433)" src="https://github.com/user-attachments/assets/7777e4d7-1617-4016-b1e6-1d0ea3b2e31b" />
<img width="1920" height="1080" alt="Screenshot (437)" src="https://github.com/user-attachments/assets/ae6bffcf-72d1-4e9b-81bd-0f222af5a687" />

# Connections
| Component | ESP32 Pin | Power | Ground | Notes |
|-----------|-----------|--------|---------|-------|
| Globe Servo (Longitude) | GPIO 12 | External 5V | Common GND | Rotates the globe |
| Ring Servo (Latitude) | GPIO 13 | External 5V | Common GND | Rotates the latitude ring |
| Ring Servo | GPIO 14 | External 5V | Common GND | support latitude ring rotation |
| SSD1306 OLED (128×64 I²C) | SDA → GPIO 21 | 3.3V | GND | I²C Data |
| SSD1306 OLED (128×64 I²C) | SCL → GPIO 22 | 3.3V | GND | I²C Clock |
| Li-ion Battery | VIN (through regulator if required) | Battery + | Battery - | Main power source |
| ESP32 DevKit V1 | USB/VIN | 5V | GND | Controller |

# Servo Connections

| Servo | Signal Pin | Function |
|--------|------------|----------|
| Servo 1 | GPIO 12 | Globe rotation (Longitude) |
| Servo 2 | GPIO 13 | Ring rotation (Latitude) |
| Servo 3 | GPIO 14 | Reserved |

# OLED Connections

| OLED Pin | ESP32 Pin |
|----------|-----------|
| VCC | 3.3V |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

I have 1 servo reserved in my cad and that is for expeimrenting later on.
I want to check it's working at home by error correct method but i do not have a 3D printer nither i found a rechable 3D printing service.
