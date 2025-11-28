/*
 * SleighVo Configuration File
 *
 * This file contains all hardware pin definitions, WiFi settings,
 * MQTT configuration, and system constants for the SleighVo
 * ESP32 animatronic controller.
 *
 * IMPORTANT: Customize these values for your specific hardware setup!
 *
 * ============================================
 * SETUP TODO LIST - Complete before first use:
 * ============================================
 * [ ] Update WIFI_SSID with your WiFi network name
 * [ ] Update WIFI_PASSWORD with your WiFi password
 * [ ] Configure MQTT_SERVER IP address (if using MQTT)
 * [ ] Update MQTT_USER and MQTT_PASSWORD (if using authentication)
 * [ ] Verify I2C pins (I2C_SDA, I2C_SCL) match your wiring
 * [ ] Verify SD card pins (SD_CS, SD_MOSI, SD_MISO, SD_SCK) match your module
 * [ ] Verify I2S audio pins (I2S_BCLK, I2S_LRC, I2S_DIN) match your DAC
 * [ ] Verify trigger pins (BUTTON_PIN, PIR_SENSOR_PIN) match your wiring
 * [ ] Customize SERVO_NAMES array for your animatronic parts
 * [ ] Adjust SERVO_CONFIGS for each servo (enable/disable, min/max pulse, trim, reverse)
 * [ ] Test each servo's direction and set 'reverse' flag if needed
 * [ ] Calibrate servo pulse widths (min_pulse, max_pulse) for your servos
 * [ ] Create SD card folder structure: /songs/ and /animations/
 * [ ] Add MP3 files to /songs/ directory on SD card
 * [ ] Create animation files in /animations/ directory
 * [ ] Update DEFAULT_SONG_FILE and DEFAULT_ANIMATION_FILE paths
 * [ ] Set E131_UNIVERSE to match your xLights configuration
 * [ ] Adjust PIR_COOLDOWN_MS for desired motion sensor behavior
 * [ ] Set DEBUG_ENABLED to true for initial testing
 * ============================================
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================
// DEFAULT CREDENTIALS
// ============================================
// These are placeholder values. WiFi and MQTT credentials are now
// configured via the web interface and stored in NVS (non-volatile storage).
// The web portal will start automatically if no WiFi credentials are found.

#define WIFI_SSID ""                      // Configured via web portal
#define WIFI_PASSWORD ""                  // Configured via web portal
#define MQTT_SERVER "192.168.1.100"       // Placeholder - configure via settings page
#define MQTT_USER ""                      // Configured via settings page
#define MQTT_PASSWORD ""                  // Configured via settings page

// ============================================
// HARDWARE CONFIGURATION
// ============================================
#define PCA9685_ENABLED false              // Enable/disable PCA9685 servo driver (set false for direct GPIO control)

// ============================================
// WIFI CONFIGURATION
// ============================================
// Static IP Configuration (optional)
#define USE_STATIC_IP false               // Set to true to use static IP
#define STATIC_IP IPAddress(192, 168, 1, 100)
#define GATEWAY IPAddress(192, 168, 1, 1)
#define SUBNET IPAddress(255, 255, 255, 0)
#define DNS IPAddress(192, 168, 1, 1)

// ============================================
// MQTT CONFIGURATION
// ============================================
#define MQTT_ENABLED false                // Enable/disable MQTT integration (disabled by default, enable via settings page)
#define MQTT_PORT 1883                    // MQTT broker port
#define MQTT_CLIENT_ID "sleighvo"         // Unique client ID for MQTT

// MQTT Topics
#define MQTT_TOPIC_STATE "sleighvo/state"
#define MQTT_TOPIC_COMMAND "sleighvo/command"
#define MQTT_TOPIC_STATUS "sleighvo/status"
#define MQTT_RECONNECT_INTERVAL 5000      // Reconnect attempt interval (ms)

// ============================================
// E1.31 (sACN) CONFIGURATION
// ============================================
#define USE_E131_MULTICAST true           // Use multicast (true) or unicast (false)
#define E131_UNIVERSE 1                   // Universe number for E1.31
#define E131_CHANNELS 512                 // Number of DMX channels
#define E131_IDLE_TIMEOUT 5000            // Time (ms) before considering E1.31 inactive

// ============================================
// DDP (Distributed Display Protocol) CONFIGURATION
// ============================================
#define DDP_ENABLED false                 // Enable/disable DDP protocol support (disabled by default, enable via settings page)
#define DDP_PORT 4048                     // DDP UDP port (standard is 4048)
#define DDP_IDLE_TIMEOUT 5000             // Time (ms) before considering DDP inactive
#define DDP_BYTES_PER_SERVO 3             // RGB bytes per servo (use R channel for position)

// ============================================
// WEB SERVER CONFIGURATION
// ============================================
#define ENABLE_WEB_SERVER true            // Enable/disable web interface
#define WEB_SERVER_PORT 80                // HTTP port for web interface

// ============================================
// I2C CONFIGURATION (for PCA9685)
// ============================================
// Note: PCA9685_ENABLED is defined above (can be overridden in config_local.h)
#define I2C_SDA 21                        // I2C SDA pin (default ESP32)
#define I2C_SCL 22                        // I2C SCL pin (default ESP32)
#define PCA9685_ADDRESS 0x40              // I2C address of PCA9685 board

// ============================================
// SERVO CONFIGURATION
// ============================================
#define NUM_SERVOS 8                      // Total number of servos
#define SERVO_FREQ 50                     // Servo PWM frequency (Hz) - standard is 50Hz

// PCA9685 pulse lengths (when using PCA9685)
#define SERVOMIN 150                      // Minimum pulse length (out of 4096)
#define SERVOMAX 600                      // Maximum pulse length (out of 4096)

// Direct GPIO servo pulse widths in microseconds (when PCA9685_ENABLED = false)
#define SERVO_MIN_PULSE_US 500            // Minimum pulse width (µs) - typically 500-1000
#define SERVO_MAX_PULSE_US 2500           // Maximum pulse width (µs) - typically 2000-2500
#define SERVO_PWM_RESOLUTION 16           // PWM resolution in bits (1-16)

// Direct GPIO pin assignments for servos (only used when PCA9685_ENABLED = false)
// Connect servos directly to these ESP32 GPIO pins
const int SERVO_GPIO_PINS[NUM_SERVOS] = {
    13,  // Servo 0: Reindeer Head    (GPIO 13)
    12,  // Servo 1: Reindeer Front   (GPIO 12)
    14,  // Servo 2: Reindeer Rear    (GPIO 14)
    27,  // Servo 3: Sleigh Tilt      (GPIO 27)
    26,  // Servo 4: Santa Wave       (GPIO 26)
    25,  // Servo 5: Santa Nod        (GPIO 25)
    33,  // Servo 6: Spare 1          (GPIO 33)
    32   // Servo 7: Spare 2          (GPIO 32)
};

#define TEST_SERVOS_ON_STARTUP false      // Test all servos during startup (disabled by default to avoid issues)

// Servo names for debugging/display
const char* SERVO_NAMES[] = {
    "Reindeer Head",    // Servo 0
    "Reindeer Front",   // Servo 1
    "Reindeer Rear",    // Servo 2
    "Sleigh Tilt",      // Servo 3
    "Santa Wave",       // Servo 4
    "Santa Nod",        // Servo 5
    "Spare 1",          // Servo 6
    "Spare 2"           // Servo 7
};

// Individual servo configuration
struct ServoConfig {
    bool enabled;       // Is this servo active?
    uint16_t min_pulse; // Minimum pulse width
    uint16_t max_pulse; // Maximum pulse width
    int16_t trim;       // Trim adjustment (-100 to +100)
    bool reverse;       // Reverse servo direction
    char name[32];      // Servo name (runtime editable)
};

// Configure each servo (customize for your setup)
const ServoConfig SERVO_CONFIGS[NUM_SERVOS] = {
    // enabled, min,  max,  trim, reverse, name
    {true,     150,  600,  0,    false,   "Reindeer Head"},  // Servo 0
    {true,     150,  600,  0,    false,   "Reindeer Front"}, // Servo 1
    {true,     150,  600,  0,    false,   "Reindeer Rear"},  // Servo 2
    {true,     150,  600,  0,    false,   "Sleigh Tilt"},    // Servo 3
    {true,     150,  600,  0,    false,   "Santa Wave"},     // Servo 4
    {true,     150,  600,  0,    false,   "Santa Nod"},      // Servo 5
    {false,    150,  600,  0,    false,   "Spare 1"},        // Servo 6 (disabled)
    {false,    150,  600,  0,    false,   "Spare 2"}         // Servo 7 (disabled)
};

// ============================================
// SD CARD CONFIGURATION
// ============================================
#define SD_ENABLED false                  // Enable/disable SD card (disabled by default, enable if SD card is present)
#define SD_CS 5                           // SD card chip select pin
#define SD_MOSI 23                        // SD card MOSI pin
#define SD_MISO 19                        // SD card MISO pin
#define SD_SCK 18                         // SD card SCK pin

// ============================================
// I2S AUDIO CONFIGURATION
// ============================================
#define I2S_BCLK 26                       // I2S bit clock pin
#define I2S_LRC 25                        // I2S left/right clock pin (word select)
#define I2S_DIN 22                        // I2S data in pin (was 22, conflicts with I2C SCL - use 27)

// ============================================
// TRIGGER INPUT PINS
// ============================================
#define BUTTON_PIN 4                      // Physical button input (active LOW with pullup)
#define PIR_SENSOR_PIN 16                 // PIR motion sensor input (active HIGH)
#define BUTTON_LED_PIN 2                  // LED indicator (built-in LED on most ESP32)

// Button timing
#define BUTTON_DEBOUNCE_MS 50             // Button debounce time (ms)
#define BUTTON_LONG_PRESS_MS 2000         // Long press threshold (ms)

// PIR sensor timing
#define PIR_COOLDOWN_MS 30000             // PIR cooldown period (ms) - 30 seconds

// ============================================
// STANDALONE MODE CONFIGURATION
// ============================================
#define STANDALONE_MODE_ENABLED false     // Enable/disable standalone playback mode (requires SD card)

// Default files for standalone mode (stored on SD card)
#define DEFAULT_SONG_FILE "/songs/jingle_bells.mp3"
#define DEFAULT_ANIMATION_FILE "/animations/jingle_bells.txt"

// ============================================
// IDLE MODE CONFIGURATION
// ============================================
#define IDLE_ANIMATION_ENABLED false      // Enable/disable idle animations (disabled by default to avoid servo issues)
#define IDLE_ANIMATION_INTERVAL 5000      // Time between idle movements (ms)

// ============================================
// DEBUGGING
// ============================================
#define DEBUG_ENABLED false               // Enable verbose serial debugging

// ============================================
// SYSTEM INFORMATION
// ============================================
#define FIRMWARE_VERSION "1.0.0"
#define DEVICE_NAME "SleighVo"

#endif // CONFIG_H
