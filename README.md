# SleighVo

ESP32-based Christmas/holiday animatronic controller that operates servo motors for displays like sleighs, reindeer, or other animated decorations. Integrates with xLights professional lighting software and Home Assistant.

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Platform](https://img.shields.io/badge/platform-ESP32-blue.svg)
![PlatformIO](https://img.shields.io/badge/PlatformIO-compatible-orange.svg)

## Features

### 🎄 Multi-Mode Operation
- **E1.31 Mode**: Real-time control from xLights lighting software (highest priority)
- **Standalone Mode**: Pre-programmed animations with synchronized audio playback
- **Idle Mode**: Subtle random movements when waiting for triggers
- **Smart Priority**: E1.31 always takes precedence, standalone mode only activates when idle

### 🏠 Home Assistant Integration
- **MQTT Auto-Discovery**: Automatically appears in Home Assistant
- **Remote Control**: Start/stop animations from your smart home
- **Real-time Status**: Monitor mode, playback status, and system health
- **Statistics Tracking**: Button presses, motion events, E1.31 packets

### 🎮 Multiple Trigger Options
- Physical button (short press to trigger, long press to stop)
- PIR motion sensor (automatic activation)
- Home Assistant/MQTT commands
- Web interface (planned)

### 🔊 Audio Playback
- MP3 playback from SD card via I2S
- Synchronized with servo animations
- Volume control

### 🤖 Servo Control
- Up to 8 servos via PCA9685 16-channel PWM driver
- Individual servo configuration (enable/disable, reverse, trim, pulse width)
- Keyframe-based animation system
- Real-time E1.31/DMX control

## Hardware Requirements

### Required Components
- ESP32 development board (ESP32-DevKit or compatible)
- PCA9685 16-channel PWM servo driver board
- Servo motors (up to 8)
- 5V power supply for servos (adequate amperage for your servo count)

### Optional Components
- SD card module (required for standalone mode)
- I2S audio DAC module (MAX98357A or similar)
- PIR motion sensor (HC-SR501 or similar)
- Physical button
- Speaker (if using audio playback)

### Wiring

**I2C (PCA9685):**
- SDA: GPIO 21
- SCL: GPIO 22

**SD Card (SPI):**
- CS: GPIO 5
- MOSI: GPIO 23
- MISO: GPIO 19
- SCK: GPIO 18

**I2S Audio:**
- BCLK: GPIO 26
- LRC: GPIO 25
- DIN: GPIO 27

**Triggers:**
- Button: GPIO 32 (with internal pullup)
- PIR Sensor: GPIO 33
- LED Indicator: GPIO 2 (built-in LED)

*All pin assignments can be customized in `include/config.h`*

## Software Setup

### Prerequisites
- [PlatformIO](https://platformio.org/) (via VS Code extension or CLI)
- [Git](https://git-scm.com/)

### Installation

1. **Clone the repository:**
```bash
git clone https://github.com/yourusername/SleighVo.git
cd SleighVo
```

2. **Configure your settings:**

Edit `include/config.h` and update:
- WiFi credentials (`WIFI_SSID`, `WIFI_PASSWORD`)
- MQTT broker settings (`MQTT_SERVER`, `MQTT_USER`, `MQTT_PASSWORD`)
- Pin assignments (if different from defaults)
- Servo names and configurations

See the TODO checklist in `config.h` for complete setup steps.

3. **Build and upload:**
```bash
# Build the project
pio run

# Upload to ESP32
pio run --target upload

# Monitor serial output
pio device monitor
```

## SD Card Setup

For standalone mode, prepare an SD card with the following structure:

```
SD Card Root/
├── songs/
│   ├── jingle_bells.mp3
│   ├── rudolph.mp3
│   └── frosty.mp3
└── animations/
    ├── jingle_bells.txt
    ├── rudolph.txt
    └── frosty.txt
```

### Animation File Format

Animation files are text files with keyframes in the format:
```
# timestamp(ms) servo_index angle(0-180)
0 0 90
1000 0 45
1500 1 120
2000 0 90
```

Lines starting with `#` are comments. Each keyframe specifies when a servo should move to a specific angle.

## Home Assistant Configuration

Once the device connects to your MQTT broker, it will automatically appear in Home Assistant with the following entities:

- **Switch**: `switch.sleighvo_standalone_mode` - Control standalone playback
- **Sensor**: `sensor.sleighvo_current_mode` - Current operating mode
- **Binary Sensor**: `binary_sensor.sleighvo_e131_active` - E1.31 connection status
- **Binary Sensor**: `binary_sensor.sleighvo_audio_playing` - Audio playback status
- **Sensor**: `sensor.sleighvo_e131_packets_received` - Packet counter
- **Button**: `button.sleighvo_trigger_animation` - Manual trigger

### Example Automation

```yaml
automation:
  - alias: "Trigger Sleigh on Doorbell"
    trigger:
      - platform: state
        entity_id: binary_sensor.front_door_bell
        to: "on"
    action:
      - service: button.press
        target:
          entity_id: button.sleighvo_trigger_animation
```

## xLights Setup

1. Add a new controller of type E1.31
2. Set the universe number to match `E131_UNIVERSE` in config.h (default: 1)
3. Configure 3 channels per servo (only first channel is used for position)
4. Map servos to channels starting at channel 1

**Channel Mapping:**
- Channels 1-3: Servo 0
- Channels 4-6: Servo 1
- Channels 7-9: Servo 2
- etc.

## Development

### Project Structure
```
SleighVo/
├── include/
│   └── config.h          # Hardware configuration
├── src/
│   └── main.cpp          # Main application (~1100 lines)
├── platformio.ini        # Build configuration
├── CLAUDE.md            # AI assistant context
└── README.md            # This file
```

### Key Functions

- `setupWiFi()` - WiFi connection
- `setupPCA9685()` - Servo driver initialization
- `setupE131()` - E1.31 listener setup
- `setupMQTT()` - MQTT client configuration
- `updateSystemMode()` - State machine logic
- `publishHomeAssistantDiscovery()` - MQTT auto-discovery
- `processE131Packet()` - Real-time servo control
- `updateAnimation()` - Keyframe playback

See `CLAUDE.md` for detailed architecture documentation.

## Troubleshooting

### WiFi Won't Connect
- Verify credentials in `config.h`
- Check 2.4GHz network (ESP32 doesn't support 5GHz)
- Set `DEBUG_ENABLED true` for verbose logging

### Servos Not Moving
- Verify I2C wiring (SDA/SCL)
- Check PCA9685 power supply
- Confirm servo is enabled in `SERVO_CONFIGS`
- Check `pio device monitor` for error messages

### MQTT Not Connecting
- Verify broker IP and port
- Check authentication credentials
- Ensure broker allows connections from ESP32's IP
- Monitor serial output for connection errors

### SD Card Not Detected
- Verify SPI pin assignments
- Format card as FAT32
- Check SD module power (3.3V)
- Test with a smaller card (32GB or less recommended)

### Audio Not Playing
- Verify I2S wiring
- Check speaker connection
- Confirm MP3 file format (mono/stereo, sample rate)
- Ensure SD card is properly mounted

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Built with [PlatformIO](https://platformio.org/)
- Uses [Adafruit PWM Servo Driver Library](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library)
- Uses [ESPAsyncE131](https://github.com/forkineye/ESPAsyncE131) for E1.31 protocol
- Integrates with [xLights](https://xlights.org/) sequencing software
- Home Assistant integration via MQTT Discovery

## Support

For issues, questions, or suggestions:
- Open an issue on GitHub
- Check existing issues for solutions
- Review `CLAUDE.md` for architecture details

---

**Made with ❤️ for the holiday season**
