# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

SleighVo is an ESP32-based Christmas/holiday animatronic controller that operates servo motors for displays like sleighs or reindeer. The system supports multiple operational modes and integrates with professional lighting software.

## Development Commands

**Build and Upload:**
```bash
# Build the project
pio run

# Upload to ESP32
pio run --target upload

# Build and upload in one command
pio run --target upload

# Monitor serial output
pio device monitor

# Clean build files
pio run --target clean
```

**Development Environment:**
```bash
# Install dependencies
pio lib install

# Update libraries
pio lib update

# Check project configuration
pio project config
```

## Library Dependencies

The project requires the following libraries (defined in `platformio.ini`):

**PlatformIO-managed:**
- `adafruit/Adafruit PWM Servo Driver Library` - PCA9685 16-channel PWM servo driver
- `forkineye/ESPAsyncE131` - Asynchronous E1.31 (sACN) protocol implementation

**ESP32 Framework libraries (Arduino):**
- `WiFi.h` - WiFi connectivity
- `Wire.h` - I2C communication for PCA9685
- `WebServer.h` - HTTP web server for control interface
- `PubSubClient.h` - MQTT client for Home Assistant integration
- `ArduinoJson.h` - JSON parsing for web/MQTT communication
- `SD.h` / `SPI.h` - SD card storage access
- `AudioFileSourceSD.h` / `AudioGeneratorMP3.h` / `AudioOutputI2S.h` - ESP32-audioI2S library for MP3 playback

**Note:** ESP32 framework libraries are included with the Arduino framework for ESP32. Only the PlatformIO-managed libraries need explicit installation.

## Architecture Overview

### Core System Design
The application implements a **state machine architecture** with four primary modes:
- `MODE_E131`: Real-time control from xLights lighting software (highest priority)
- `MODE_STANDALONE`: Pre-programmed animations with audio playback
- `MODE_IDLE`: Subtle movements when waiting for triggers
- `MODE_STARTUP`: Initial system state

### Key Architectural Components

**Hardware Abstraction Layer:**
- `Adafruit_PWMServoDriver`: Controls multiple servos via I2C PCA9685 chip
- `ESPAsyncE131`: Handles real-time lighting protocol packets
- `AudioOutputI2S`: Manages audio playback from SD card

**State Management:**
- `SystemState` struct tracks current mode, timing, and playback status
- `ServoState` array maintains individual servo positions and control sources
- Mode transitions based on E1.31 activity, triggers, and timeouts

**Multi-Protocol Integration:**
- E1.31 packets take absolute priority and switch system to real-time mode
- MQTT integration for Home Assistant compatibility
- Web server provides control interface and status monitoring
- SD card storage for audio files (`/songs/`) and animation sequences (`/animations/`)

### Home Assistant / MQTT Integration
The system implements full MQTT support with Home Assistant auto-discovery:

**Auto-discovered Entities:**
- Switch: Standalone Mode control (ON/OFF)
- Sensor: Current Mode (E1.31/Standalone/Idle/Startup)
- Binary Sensor: E1.31 Active status
- Binary Sensor: Audio Playing status
- Sensor: E1.31 Packets Received counter
- Button: Manual trigger for animations

**State Publishing:**
- System publishes state to `sleighvo/state` every 30 seconds
- Includes mode, playback status, statistics, uptime, WiFi RSSI, free heap
- State updates automatically on mode changes

**Command Topics:**
- `sleighvo/command/standalone` - Control standalone mode (ON/OFF)
- `sleighvo/command/trigger` - Manually trigger animation (TRIGGER)
- `sleighvo/command/stop` - Stop current playback

**Implementation Details:**
- `publishHomeAssistantDiscovery()` sends MQTT discovery messages on connection (src/main.cpp:725)
- `publishMQTTState()` publishes current state as JSON (src/main.cpp:849)
- `handleMQTTCommand()` processes incoming commands (src/main.cpp:907)
- `reconnectMQTT()` handles automatic reconnection with exponential backoff (src/main.cpp:951)

### Configuration File
The project uses `include/config.h` which contains hardware pin definitions and system constants like:
- `PCA9685_ADDRESS`, `NUM_SERVOS`, `WEB_SERVER_PORT`
- `E131_UNIVERSE`, `MQTT_ENABLED`, `MQTT_SERVER`, `MQTT_USER`, `MQTT_PASSWORD`
- `MQTT_TOPIC_STATE`, `MQTT_TOPIC_COMMAND`, `MQTT_RECONNECT_INTERVAL`
- GPIO pin assignments for buttons, PIR sensor, I2S audio, SD card
- WiFi credentials and optional static IP configuration

**This file is now included in the repository** - customize WiFi/MQTT credentials before building.

### Animation System
Animations use a keyframe-based system stored as `AnimationKeyframe` vectors with timestamp, servo index, and angle. The system interpolates between keyframes during playback.

### Trigger System
Standalone mode activation via:
- Physical button press (debounced)
- PIR motion sensor detection
- Web interface commands
- MQTT messages

### Priority Hierarchy
1. E1.31 packets (immediate override)
2. Standalone animations (triggered mode)
3. Idle animations (background movement)
4. Manual web/MQTT control

## File Structure

- `src/main.cpp`: Complete application in single file (~800+ lines)
- `platformio.ini`: ESP32 build configuration and dependencies
- `include/`: Header files (currently empty, expects `config.h`)
- `lib/`: Custom libraries (currently uses standard PlatformIO dependencies)

## Hardware Dependencies

The system expects specific hardware connections defined in the missing `config.h`:
- ESP32 development board
- PCA9685 PWM driver for servo control
- SD card module for audio/animation storage
- I2S audio output (optional)
- Physical button and PIR sensor inputs
- Optional status LEDs