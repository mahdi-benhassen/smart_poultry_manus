# Smart Poultry Management System

A comprehensive, modular, and extensible ESP32-based smart poultry house management system built with **ESP-IDF** (Espressif IoT Development Framework). The system monitors environmental conditions through multiple sensors and automatically controls actuators to maintain optimal conditions for poultry health and productivity.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                        MAIN APPLICATION                         │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌───────────────────┐  │
│  │ Sensor   │ │ Control  │ │  Comm    │ │   Monitoring      │  │
│  │ Task     │ │ Task     │ │  Task    │ │   Task            │  │
│  │ (2s)     │ │ (1s)     │ │  (5s)    │ │   (10s)           │  │
│  └────┬─────┘ └────┬─────┘ └────┬─────┘ └───────┬───────────┘  │
│       │            │            │                │              │
│  ┌────▼────────────▼────────────▼────────────────▼───────────┐  │
│  │              Shared Data (Mutex Protected)                │  │
│  └───────────────────────────────────────────────────────────┘  │
├─────────────────────────────────────────────────────────────────┤
│                        COMPONENTS                               │
│                                                                 │
│  ┌─────────────┐  ┌──────────────┐  ┌────────────────────────┐  │
│  │   Sensor    │  │   Actuator   │  │    Control Logic       │  │
│  │   Manager   │  │   Manager    │  │  ┌─────┐ ┌──────────┐ │  │
│  │  ┌───────┐  │  │  ┌────────┐  │  │  │ PID │ │Hysteresis│ │  │
│  │  │DHT22  │  │  │  │Fan PWM │  │  │  └─────┘ └──────────┘ │  │
│  │  │MQ135  │  │  │  │Exhaust │  │  │  ┌─────┐ ┌──────────┐ │  │
│  │  │MQ7    │  │  │  │Heater  │  │  │  │ AQI │ │ Adaptive │ │  │
│  │  │MQ4    │  │  │  │Cooler  │  │  │  │Calc │ │  Vent    │ │  │
│  │  │MQ136  │  │  │  │Light   │  │  │  └─────┘ └──────────┘ │  │
│  │  │SCD40  │  │  │  │Pump    │  │  │  ┌─────┐ ┌──────────┐ │  │
│  │  │Dust   │  │  │  │Feeder  │  │  │  │Anom.│ │  Alert   │ │  │
│  │  │Light  │  │  │  │Alarm   │  │  │  │Det. │ │  System  │ │  │
│  │  │Water  │  │  │  │Servo   │  │  │  └─────┘ └──────────┘ │  │
│  │  │Sound  │  │  │  └────────┘  │  └────────────────────────┘  │
│  │  │Door   │  │  │              │                              │
│  │  └───────┘  │  │              │  ┌────────────────────────┐  │
│  └─────────────┘  └──────────────┘  │   Comm Manager         │  │
│                                     │  ┌──────┐ ┌──────────┐ │  │
│  ┌─────────────┐  ┌──────────────┐  │  │ WiFi │ │   MQTT   │ │  │
│  │   Config    │  │     OTA      │  │  └──────┘ └──────────┘ │  │
│  │   Manager   │  │   Manager    │  │  ┌──────────────────┐  │  │
│  │   (NVS)     │  │   (HTTPS)    │  │  │   HTTP Server    │  │  │
│  └─────────────┘  └──────────────┘  │  └──────────────────┘  │  │
│                                     └────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

## Features

### Sensors (12 types)

| Sensor | Component | Interface | Measures |
|--------|-----------|-----------|----------|
| **DHT22** | `sensor_dht22` | GPIO (bit-bang) | Temperature (°C), Humidity (%) |
| **MQ-135** | `sensor_mq135` | ADC | NH3 / Air Quality (ppm) |
| **MQ-7** | `sensor_mq7` | ADC | Carbon Monoxide CO (ppm) |
| **MQ-4** | `sensor_mq4` | ADC | Methane CH4 (ppm) |
| **MQ-136** | `sensor_mq136` | ADC | Hydrogen Sulfide H2S (ppm) |
| **SCD40** | `sensor_scd40` | I2C | CO2 (ppm) + Temp + Humidity |
| **GP2Y1010AU0F** | `sensor_dust` | ADC + GPIO | Dust/PM (µg/m³) |
| **BH1750** | `sensor_light` | I2C | Light Intensity (lux) |
| **Analog Level** | `sensor_water_level` | ADC | Water Level (%) |
| **Analog Mic** | `sensor_sound` | ADC | Sound Level (dB) |
| **Reed Switch** | `sensor_door` | GPIO | Door Open/Closed |

All sensors use a **moving average filter** (window size 10) for noise reduction.

### Actuators (9 types)

| Actuator | Component | Interface | Function |
|----------|-----------|-----------|----------|
| **Main Fan** | `actuator_fan` | LEDC PWM (25kHz) | Variable-speed ventilation |
| **Exhaust Fan** | `actuator_exhaust_fan` | GPIO relay | On/off exhaust |
| **Heater** | `actuator_heater` | GPIO relay | Heating element |
| **Cooler** | `actuator_cooler` | GPIO relay | Evaporative cooler |
| **Lighting** | `actuator_light` | LEDC PWM (1kHz) | Dimmable lights with fade |
| **Water Pump** | `actuator_water_pump` | GPIO relay | Water refill |
| **Feed Dispenser** | `actuator_feed_dispenser` | GPIO + Timer | Timed feed dispensing |
| **Alarm Buzzer** | `actuator_alarm` | GPIO | Patterns: continuous, intermittent, SOS |
| **Door Servo** | `actuator_door_servo` | LEDC PWM (50Hz) | 0-180° door/vent control |

### Smart Control Algorithms

- **PID Temperature Control**: Proportional-Integral-Derivative controller with anti-windup for precise temperature regulation
- **Hysteresis Humidity Control**: Deadband-based on/off control to prevent oscillation
- **Air Quality Index (AQI)**: Weighted combination of NH3, CO, CO2, CH4, H2S, and dust into a 0-500 index with 6 severity categories
- **Adaptive Ventilation**: Multi-factor fan speed calculation based on AQI, temperature, and humidity with minimum ventilation guarantee
- **Light Schedule Manager**: Configurable on/off times with gradual dimming/brightening transitions; ambient light compensation
- **Water Management**: Automatic refill with hysteresis between min/max levels
- **Feed Scheduling**: Configurable daily feeding times with timed dispensing
- **Anomaly Detection**: Detects sensor failures (stuck values, out-of-range) and sudden spikes
- **Multi-Level Alert System**: Four severity levels (INFO, WARNING, CRITICAL, EMERGENCY) with escalating responses

### Communication

- **WiFi**: Station mode with auto-reconnect (up to 10 retries)
- **MQTT Client**: Publishes sensor data, actuator states, AQI, and alerts; subscribes to command topics for remote control
- **HTTP Server**: REST API endpoints for local dashboard integration
- **OTA Updates**: Over-the-air firmware updates via HTTPS, triggered remotely via MQTT

## Project Structure

```
smart_poultry_system/
├── CMakeLists.txt              # Top-level project CMake
├── sdkconfig.defaults          # Default SDK configuration
├── partitions.csv              # Custom partition table (OTA-enabled)
├── README.md                   # This file
├── main/
│   ├── CMakeLists.txt
│   ├── Kconfig.projbuild       # Menuconfig options
│   └── main.c                  # Application entry point & FreeRTOS tasks
└── components/
    ├── sensor_manager/         # Central sensor data aggregator
    ├── sensor_dht22/           # Temperature & humidity
    ├── sensor_mq135/           # NH3 / air quality
    ├── sensor_mq7/             # Carbon monoxide
    ├── sensor_mq4/             # Methane
    ├── sensor_mq136/           # Hydrogen sulfide
    ├── sensor_scd40/           # CO2 (I2C)
    ├── sensor_dust/            # Particulate matter
    ├── sensor_light/           # Light intensity (I2C)
    ├── sensor_water_level/     # Water level
    ├── sensor_sound/           # Sound level
    ├── sensor_door/            # Door/entry detection
    ├── actuator_manager/       # Central actuator controller
    ├── actuator_fan/           # PWM ventilation fan
    ├── actuator_exhaust_fan/   # On/off exhaust fan
    ├── actuator_heater/        # Heating relay
    ├── actuator_cooler/        # Cooling relay
    ├── actuator_light/         # Dimmable lighting
    ├── actuator_water_pump/    # Water pump relay
    ├── actuator_feed_dispenser/# Timed feed dispenser
    ├── actuator_alarm/         # Buzzer with patterns
    ├── actuator_door_servo/    # Servo motor for door/vent
    ├── control_logic/          # PID, AQI, adaptive control, alerts
    ├── comm_manager/           # WiFi, MQTT, HTTP server
    ├── config_manager/         # NVS persistent configuration
    └── ota_manager/            # OTA firmware updates
```

## Hardware Wiring

### Default Pin Assignments

| Function | GPIO | Notes |
|----------|------|-------|
| DHT22 Data | 4 | 4.7kΩ pull-up to 3.3V |
| MQ-135 (NH3) | ADC1_CH0 (36) | Analog input |
| MQ-7 (CO) | ADC1_CH1 (37) | Analog input |
| MQ-4 (CH4) | ADC1_CH2 (38) | Analog input |
| MQ-136 (H2S) | ADC1_CH3 (39) | Analog input |
| Dust Sensor ADC | ADC1_CH4 (32) | Analog input |
| Dust Sensor LED | 5 | Digital output |
| Water Level | ADC1_CH6 (34) | Analog input |
| Sound Mic | ADC1_CH7 (35) | Analog input |
| Door Sensor | 18 | Digital input, pull-up |
| I2C SDA | 21 | SCD40 + BH1750 |
| I2C SCL | 22 | SCD40 + BH1750 |
| Main Fan PWM | 25 | MOSFET gate |
| Exhaust Fan | 26 | Relay module |
| Heater | 27 | Relay module |
| Cooler | 14 | Relay module |
| Light PWM | 12 | MOSFET/dimmer |
| Water Pump | 13 | Relay module |
| Feed Dispenser | 15 | Relay module |
| Alarm Buzzer | 2 | Direct or transistor |
| Door Servo | 33 | Standard servo signal |

### Power Notes

- MQ-series sensors require 5V heater power and produce analog output (use voltage divider if needed for 3.3V ADC)
- Relay modules should be opto-isolated and rated for your actuator loads
- Servo motor should have a separate 5V power supply
- Use appropriate MOSFET drivers for PWM-controlled fans and lights

## Build Instructions

### Prerequisites

1. Install ESP-IDF v5.x: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/
2. Set up environment: `. $HOME/esp/esp-idf/export.sh`

### Build and Flash

```bash
cd smart_poultry_system

# Configure (optional - modify settings via menuconfig)
idf.py menuconfig

# Build
idf.py build

# Flash to ESP32
idf.py -p /dev/ttyUSB0 flash

# Monitor serial output
idf.py -p /dev/ttyUSB0 monitor

# Build + Flash + Monitor in one command
idf.py -p /dev/ttyUSB0 flash monitor
```

### Configuration via Menuconfig

Navigate to **Smart Poultry System Configuration** to set:
- WiFi credentials
- MQTT broker URI
- Sensor pin assignments
- Sensor thresholds (temperature, humidity, gas levels)
- Actuator GPIO assignments
- PID controller gains (Kp, Ki, Kd)
- Light schedule (on/off hours)
- Feed schedule (times per day, duration)

## MQTT Topics

### Published Topics

| Topic | QoS | Frequency | Description |
|-------|-----|-----------|-------------|
| `poultry/sensors` | 0 | 5s | JSON with all sensor readings |
| `poultry/actuators` | 0 | 5s | JSON with all actuator states |
| `poultry/aqi` | 0 | 5s | Air Quality Index and category |
| `poultry/alerts` | 1 | On event | Alert notifications with severity |

### Command Topics (Subscribe)

| Topic | Payload | Description |
|-------|---------|-------------|
| `poultry/cmd/fan` | `0-100` | Set fan speed percentage |
| `poultry/cmd/light` | `0-100` | Set light level percentage |
| `poultry/cmd/heater` | `1` or `0` | Turn heater on/off |
| `poultry/cmd/cooler` | `1` or `0` | Turn cooler on/off |
| `poultry/cmd/ota` | `http://url/firmware.bin` | Trigger OTA update |

### Example Sensor Data JSON

```json
{
  "temp": 25.3,
  "hum": 62.1,
  "nh3": 12.5,
  "co": 3.2,
  "co2": 850,
  "ch4": 45,
  "h2s": 1.2,
  "dust": 35.6,
  "lux": 450,
  "water": 75,
  "sound": 52.3,
  "door": false
}
```

## HTTP API

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/status` | GET | Full system status (sensors + actuators + AQI) |
| `/api/health` | GET | System health check |
| `/api/control` | POST | Send control commands (JSON body) |

### Example: Get Status

```bash
curl http://<esp32-ip>/api/status
```

### Example: Send Control Command

```bash
curl -X POST http://<esp32-ip>/api/control -d '{"fan":80}'
```

## Alert Levels

| Level | Trigger Conditions | System Response |
|-------|-------------------|-----------------|
| **INFO** | AQI > 100, humidity out of range | Log only |
| **WARNING** | Gas above threshold, temp out of range, low water, sensor spike | Intermittent alarm, MQTT alert |
| **CRITICAL** | Gas 1.5x threshold, AQI > 300, temp ±5° beyond range | Continuous alarm, boost ventilation to 80% |
| **EMERGENCY** | CO > 2x, H2S > 2x, temp > max+10 | SOS alarm, max ventilation, all exhaust on |

## Extending the System

### Adding a New Sensor

1. Create a new directory: `components/sensor_new/`
2. Add files: `CMakeLists.txt`, `include/sensor_new.h`, `sensor_new.c`
3. Implement `sensor_new_init()` and `sensor_new_read()` functions
4. Add the reading field to `sensor_data_t` in `sensor_manager.h`
5. Call the new sensor's init/read in `sensor_manager.c`
6. Update `control_logic.c` if the new sensor affects control decisions

### Adding a New Actuator

1. Create a new directory: `components/actuator_new/`
2. Add files: `CMakeLists.txt`, `include/actuator_new.h`, `actuator_new.c`
3. Implement `actuator_new_init()` and `actuator_new_set()` functions
4. Add the state field to `actuator_state_t` in `actuator_manager.h`
5. Call the new actuator's init/set in `actuator_manager.c`
6. Update `control_logic.c` to drive the new actuator

### Adding a New Control Algorithm

1. Add the algorithm function in `control_logic.c`
2. Call it from `control_logic_run()` at the appropriate point
3. If it needs configuration, add parameters to `Kconfig.projbuild` and `config_manager`

## License

This project is provided as-is for educational and development purposes.
