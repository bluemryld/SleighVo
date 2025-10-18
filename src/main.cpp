/*
 * ESP32 E1.31 Servo Controller with Standalone Mode
 * 
 * Features:
 * - E1.31 control from xLights (primary mode)
 * - MQTT/Home Assistant integration
 * - Standalone playback mode when E1.31 idle:
 *   - Physical button trigger
 *   - PIR motion sensor trigger
 *   - Local audio playback (SD card)
 *   - Pre-programmed animations
 * - Idle mode with subtle movements
 * - Web interface
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncE131.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <SD.h>
#include <SPI.h>
#include "AudioFileSourceSD.h"
#include "AudioGeneratorMP3.h"
#include "AudioOutputI2S.h"
#include "config.h"

#ifdef DISABLE_BROWNOUT_DETECTOR
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif

// ============================================
// GLOBAL OBJECTS
// ============================================
#if PCA9685_ENABLED
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);
#endif
ESPAsyncE131 e131(1);
WiFiClient espClient;
PubSubClient mqttClient(espClient);
WebServer server(WEB_SERVER_PORT);

// Audio objects
AudioGeneratorMP3 *mp3 = nullptr;
AudioFileSourceSD *file = nullptr;
AudioOutputI2S *out = nullptr;

// ============================================
// SYSTEM STATE
// ============================================
enum SystemMode {
    MODE_E131,          // xLights is playing
    MODE_STANDALONE,    // Local playback active
    MODE_IDLE,          // Waiting, subtle movements
    MODE_STARTUP        // Initial state
};

struct SystemState {
    SystemMode current_mode;
    SystemMode previous_mode;
    bool audio_playing;
    bool animation_playing;
    unsigned long mode_start_time;
    unsigned long last_e131_packet;
    unsigned long last_trigger_time;
    String current_song;
    String current_animation;
};

SystemState system_state = {
    MODE_STARTUP, MODE_STARTUP, false, false, 0, 0, 0, "", ""
};

// ============================================
// SERVO STATE
// ============================================
struct ServoState {
    uint16_t current_position;
    uint16_t target_position;
    uint8_t current_angle;
    bool initialized;
    unsigned long last_update;
    String control_source;
};

ServoState servoStates[NUM_SERVOS];

// ============================================
// ANIMATION SEQUENCE
// ============================================
struct AnimationKeyframe {
    unsigned long timestamp;  // Milliseconds from start
    uint8_t servo_index;
    uint8_t angle;
};

std::vector<AnimationKeyframe> animation_sequence;
unsigned long animation_start_time = 0;
size_t animation_keyframe_index = 0;

// ============================================
// STATISTICS
// ============================================
struct Stats {
    unsigned long packets_received;
    unsigned long standalone_triggers;
    unsigned long button_presses;
    unsigned long motion_detects;
    bool mqtt_connected;
    bool sd_card_available;
};

Stats stats = {0, 0, 0, 0, false, false};

// ============================================
// BUTTON/SENSOR STATE
// ============================================
struct ButtonState {
    bool current;
    bool previous;
    unsigned long press_time;
    unsigned long release_time;
    bool long_press_triggered;
};

ButtonState button = {false, false, 0, 0, false};
bool pir_triggered = false;
unsigned long last_pir_trigger = 0;

// Forward declarations
void setupWiFi();
void setupPCA9685();
void setupE131();
void setupMQTT();
void setupWebServer();
void setupAudio();
void setupSDCard();
void setupTriggers();
void updateButtonState();
void updatePIRState();
void processE131Packet();
void updateServoFromE131(int servoIndex, uint8_t dmxValue);
void setServoAngle(int servoIndex, uint8_t angle, const char* source);
void loadAnimationSequence(const char* filename);
void updateAnimation();
void startStandalonePlayback();
void stopStandalonePlayback();
void updateIdleAnimation();
void switchMode(SystemMode newMode);
bool isE131Active();

// ============================================
// SETUP FUNCTIONS
// ============================================

void setupWiFi() {
    Serial.println("\n=== WiFi Setup ===");
    Serial.print("Connecting to: ");
    Serial.println(WIFI_SSID);
    
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(MQTT_CLIENT_ID);
    
    #ifdef USE_STATIC_IP
    if (USE_STATIC_IP) {
        WiFi.config(STATIC_IP, GATEWAY, SUBNET, DNS);
    }
    #endif
    
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✓ WiFi connected!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\n✗ WiFi failed!");
    }
}

bool checkI2CDevice(uint8_t address) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    return (error == 0);
}

void scanI2CBus() {
    Serial.println("\n=== I2C Bus Scan ===");
    Serial.print("Scanning I2C bus (SDA=");
    Serial.print(I2C_SDA);
    Serial.print(", SCL=");
    Serial.print(I2C_SCL);
    Serial.println(")...");

    int deviceCount = 0;
    for (uint8_t address = 1; address < 127; address++) {
        if (checkI2CDevice(address)) {
            Serial.print("  Found device at 0x");
            if (address < 16) Serial.print("0");
            Serial.println(address, HEX);
            deviceCount++;
        }
    }

    if (deviceCount == 0) {
        Serial.println("✗ No I2C devices found!");
        Serial.println("  Check wiring:");
        Serial.print("    SDA -> GPIO ");
        Serial.println(I2C_SDA);
        Serial.print("    SCL -> GPIO ");
        Serial.println(I2C_SCL);
        Serial.println("    VCC -> 5V");
        Serial.println("    GND -> GND");
    } else {
        Serial.print("✓ Found ");
        Serial.print(deviceCount);
        Serial.println(" I2C device(s)");
    }
}

void setupPCA9685() {
    Serial.println("\n=== PCA9685 Setup ===");

#if !PCA9685_ENABLED
    Serial.println("⚠ PCA9685 DISABLED in config.h - running in simulation mode");
    Serial.println("✓ Servo commands will be logged but not executed");

    // Initialize servo states for simulation
    for (int i = 0; i < NUM_SERVOS; i++) {
        if (SERVO_CONFIGS[i].enabled) {
            servoStates[i].current_angle = 90;
            servoStates[i].initialized = true;
            Serial.print("Servo ");
            Serial.print(i);
            Serial.print(" (");
            Serial.print(SERVO_NAMES[i]);
            Serial.println(") - simulation mode ready");
        }
    }
    return;
#endif

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);  // 100kHz I2C speed (standard mode)
    delay(100);

    // Check if PCA9685 is present
    Serial.print("Looking for PCA9685 at address 0x");
    Serial.println(PCA9685_ADDRESS, HEX);

    if (!checkI2CDevice(PCA9685_ADDRESS)) {
        Serial.println("✗ PCA9685 not found!");
        scanI2CBus();
        Serial.println("\n⚠ Continuing without servo control...");
        return;
    }

    Serial.println("✓ PCA9685 detected");

#if PCA9685_ENABLED
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ);
    delay(100);
#endif

    // Initialize all servos to center
    for (int i = 0; i < NUM_SERVOS; i++) {
        if (SERVO_CONFIGS[i].enabled) {
            setServoAngle(i, 90, "startup");
            Serial.print("Servo ");
            Serial.print(i);
            Serial.print(" (");
            Serial.print(SERVO_NAMES[i]);
            Serial.println(") ready");
        }
    }
    Serial.println("✓ Servos initialized");
}

void testAllServos() {
    Serial.println("\n=== Servo Test Sequence ===");
    Serial.println("Testing each servo with full range sweep...");

    // Test each servo individually
    for (int i = 0; i < NUM_SERVOS; i++) {
        if (!SERVO_CONFIGS[i].enabled) {
            Serial.print("Servo ");
            Serial.print(i);
            Serial.print(" (");
            Serial.print(SERVO_NAMES[i]);
            Serial.println(") - DISABLED");
            continue;
        }

        Serial.print("Testing Servo ");
        Serial.print(i);
        Serial.print(" (");
        Serial.print(SERVO_NAMES[i]);
        Serial.println(")...");

        // Center position
        setServoAngle(i, 90, "test");
        delay(300);

        // Sweep to minimum
        for (int angle = 90; angle >= 0; angle -= 10) {
            setServoAngle(i, angle, "test");
            delay(50);
        }

        // Sweep to maximum
        for (int angle = 0; angle <= 180; angle += 10) {
            setServoAngle(i, angle, "test");
            delay(50);
        }

        // Back to center
        for (int angle = 180; angle >= 90; angle -= 10) {
            setServoAngle(i, angle, "test");
            delay(50);
        }

        Serial.print("  ✓ Servo ");
        Serial.print(i);
        Serial.println(" test complete");
        delay(200);
    }

    Serial.println("✓ All servo tests complete!");
    delay(500);  // Brief pause before continuing
}

void setupE131() {
    Serial.println("\n=== E1.31 Setup ===");

    if (USE_E131_MULTICAST) {
        if (e131.begin(E131_MULTICAST, E131_UNIVERSE)) {
            Serial.print("✓ E1.31 multicast universe ");
            Serial.println(E131_UNIVERSE);
        }
    } else {
        if (e131.begin(E131_UNICAST)) {
            Serial.print("✓ E1.31 unicast mode");
        }
    }
}

void setupSDCard() {
    if (!SD_ENABLED) return;
    
    Serial.println("\n=== SD Card Setup ===");
    
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    
    if (SD.begin(SD_CS)) {
        Serial.println("✓ SD card mounted");
        stats.sd_card_available = true;
        
        // List available songs
        File root = SD.open("/songs");
        if (root) {
            Serial.println("Available songs:");
            File file = root.openNextFile();
            while (file) {
                if (!file.isDirectory()) {
                    Serial.print("  - ");
                    Serial.println(file.name());
                }
                file = root.openNextFile();
            }
        }
        
        // List available animations
        root = SD.open("/animations");
        if (root) {
            Serial.println("Available animations:");
            File file = root.openNextFile();
            while (file) {
                if (!file.isDirectory()) {
                    Serial.print("  - ");
                    Serial.println(file.name());
                }
                file = root.openNextFile();
            }
        }
    } else {
        Serial.println("✗ SD card mount failed!");
        Serial.println("  Standalone mode requires SD card");
        stats.sd_card_available = false;
    }
}

void setupAudio() {
    if (!STANDALONE_MODE_ENABLED) return;
    
    Serial.println("\n=== Audio Setup ===");
    
    // Initialize I2S audio output
    out = new AudioOutputI2S();
    out->SetPinout(I2S_BCLK, I2S_LRC, I2S_DIN);
    out->SetGain(0.5);  // Volume 0.0 to 1.0
    
    Serial.println("✓ I2S audio initialized");
}

void setupTriggers() {
    Serial.println("\n=== Trigger Setup ===");
    
    // Button input
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    Serial.print("Button on GPIO ");
    Serial.println(BUTTON_PIN);
    
    // PIR sensor input
    pinMode(PIR_SENSOR_PIN, INPUT);
    Serial.print("PIR sensor on GPIO ");
    Serial.println(PIR_SENSOR_PIN);
    
    // Button LED output (optional)
    pinMode(BUTTON_LED_PIN, OUTPUT);
    digitalWrite(BUTTON_LED_PIN, LOW);
    
    Serial.println("✓ Triggers initialized");
}

// ============================================
// SERVO CONTROL
// ============================================

void setServoAngle(int servoIndex, uint8_t angle, const char* source) {
    if (servoIndex < 0 || servoIndex >= NUM_SERVOS) return;
    if (!SERVO_CONFIGS[servoIndex].enabled) return;
    
    angle = constrain(angle, 0, 180);
    
    // Map angle to pulse width
    uint16_t pulse;
    if (SERVO_CONFIGS[servoIndex].reverse) {
        pulse = map(180 - angle, 0, 180,
                   SERVO_CONFIGS[servoIndex].min_pulse,
                   SERVO_CONFIGS[servoIndex].max_pulse);
    } else {
        pulse = map(angle, 0, 180,
                   SERVO_CONFIGS[servoIndex].min_pulse,
                   SERVO_CONFIGS[servoIndex].max_pulse);
    }
    
    // Apply trim
    pulse = constrain(pulse + SERVO_CONFIGS[servoIndex].trim, SERVOMIN, SERVOMAX);

    // Update servo
#if PCA9685_ENABLED
    pwm.setPWM(servoIndex, 0, pulse);
#else
    // Simulation mode - log servo commands
    if (DEBUG_ENABLED) {
        Serial.print("[SIM] Servo ");
        Serial.print(servoIndex);
        Serial.print(" (");
        Serial.print(SERVO_NAMES[servoIndex]);
        Serial.print("): ");
        Serial.print(angle);
        Serial.print("° [pulse=");
        Serial.print(pulse);
        Serial.print(", source=");
        Serial.print(source);
        Serial.println("]");
    }
#endif
    servoStates[servoIndex].current_position = pulse;
    servoStates[servoIndex].current_angle = angle;
    servoStates[servoIndex].control_source = source;
    servoStates[servoIndex].last_update = millis();
}

void updateServoFromE131(int servoIndex, uint8_t dmxValue) {
    uint8_t angle = map(dmxValue, 0, 255, 0, 180);
    setServoAngle(servoIndex, angle, "e131");
}

// ============================================
// ANIMATION SYSTEM
// ============================================

void loadAnimationSequence(const char* filename) {
    animation_sequence.clear();
    
    if (!stats.sd_card_available) {
        Serial.println("Cannot load animation: SD card unavailable");
        return;
    }
    
    File file = SD.open(filename);
    if (!file) {
        Serial.print("Cannot open animation file: ");
        Serial.println(filename);
        return;
    }
    
    Serial.print("Loading animation: ");
    Serial.println(filename);
    
    while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();
        
        // Skip empty lines and comments
        if (line.length() == 0 || line.startsWith("#")) continue;
        
        // Parse: timestamp servo_index angle
        int space1 = line.indexOf(' ');
        int space2 = line.lastIndexOf(' ');
        
        if (space1 > 0 && space2 > space1) {
            AnimationKeyframe kf;
            kf.timestamp = line.substring(0, space1).toInt();
            kf.servo_index = line.substring(space1 + 1, space2).toInt();
            kf.angle = line.substring(space2 + 1).toInt();
            
            animation_sequence.push_back(kf);
        }
    }
    
    file.close();
    
    Serial.print("✓ Loaded ");
    Serial.print(animation_sequence.size());
    Serial.println(" keyframes");
}

void updateAnimation() {
    if (!system_state.animation_playing) return;
    if (animation_sequence.empty()) return;
    
    unsigned long elapsed = millis() - animation_start_time;
    
    // Process all keyframes that should have happened by now
    while (animation_keyframe_index < animation_sequence.size()) {
        AnimationKeyframe& kf = animation_sequence[animation_keyframe_index];
        
        if (elapsed >= kf.timestamp) {
            setServoAngle(kf.servo_index, kf.angle, "animation");
            animation_keyframe_index++;
        } else {
            break;  // Wait for next keyframe time
        }
    }
    
    // Check if animation finished
    if (animation_keyframe_index >= animation_sequence.size()) {
        Serial.println("Animation sequence complete");
        system_state.animation_playing = false;
        animation_keyframe_index = 0;
    }
}

// ============================================
// AUDIO PLAYBACK
// ============================================

void startAudioPlayback(const char* filename) {
    if (!stats.sd_card_available) {
        Serial.println("Cannot play audio: SD card unavailable");
        return;
    }
    
    // Stop any existing playback
    if (mp3 && mp3->isRunning()) {
        mp3->stop();
        delete mp3;
        delete file;
    }
    
    Serial.print("Playing: ");
    Serial.println(filename);
    
    // Create new audio objects
    file = new AudioFileSourceSD(filename);
    mp3 = new AudioGeneratorMP3();
    
    if (mp3->begin(file, out)) {
        system_state.audio_playing = true;
        system_state.current_song = filename;
    } else {
        Serial.println("✗ Audio playback failed to start");
        system_state.audio_playing = false;
        delete mp3;
        delete file;
        mp3 = nullptr;
        file = nullptr;
    }
}

void updateAudioPlayback() {
    if (!system_state.audio_playing) return;
    if (!mp3) return;
    
    if (mp3->isRunning()) {
        if (!mp3->loop()) {
            // Audio finished or error
            mp3->stop();
            system_state.audio_playing = false;
            Serial.println("Audio playback complete");
        }
    } else {
        system_state.audio_playing = false;
    }
}

// ============================================
// STANDALONE MODE
// ============================================

void startStandalonePlayback() {
    if (!STANDALONE_MODE_ENABLED) return;
    
    Serial.println("\n=== Starting Standalone Playback ===");
    
    // Load animation sequence
    loadAnimationSequence(DEFAULT_ANIMATION_FILE);
    
    // Start animation
    animation_start_time = millis();
    animation_keyframe_index = 0;
    system_state.animation_playing = true;
    
    // Start audio
    startAudioPlayback(DEFAULT_SONG_FILE);
    
    // Switch mode
    switchMode(MODE_STANDALONE);
    
    // Light up button LED
    digitalWrite(BUTTON_LED_PIN, HIGH);
    
    stats.standalone_triggers++;
}

void stopStandalonePlayback() {
    Serial.println("Stopping standalone playback");
    
    // Stop audio
    if (mp3 && mp3->isRunning()) {
        mp3->stop();
    }
    system_state.audio_playing = false;
    system_state.animation_playing = false;
    
    // Turn off button LED
    digitalWrite(BUTTON_LED_PIN, LOW);
    
    // Return servos to neutral
    for (int i = 0; i < NUM_SERVOS; i++) {
        if (SERVO_CONFIGS[i].enabled) {
            setServoAngle(i, 90, "idle");
        }
    }
    
    switchMode(MODE_IDLE);
}

// ============================================
// IDLE MODE
// ============================================

void updateIdleAnimation() {
    if (!IDLE_ANIMATION_ENABLED) return;
    if (system_state.current_mode != MODE_IDLE) return;
    
    static unsigned long last_idle_move = 0;
    
    if (millis() - last_idle_move > IDLE_ANIMATION_INTERVAL) {
        // Random subtle movement
        int servo = random(0, NUM_SERVOS);
        if (SERVO_CONFIGS[servo].enabled) {
            int angle = random(80, 100);  // Small range around center
            setServoAngle(servo, angle, "idle");
            
            #if DEBUG_ENABLED
            Serial.print("Idle: Servo ");
            Serial.print(servo);
            Serial.print(" → ");
            Serial.println(angle);
            #endif
        }
        
        last_idle_move = millis();
    }
}

// ============================================
// MODE MANAGEMENT
// ============================================

bool isE131Active() {
    return (millis() - system_state.last_e131_packet) < E131_IDLE_TIMEOUT;
}

void switchMode(SystemMode newMode) {
    if (system_state.current_mode == newMode) return;
    
    system_state.previous_mode = system_state.current_mode;
    system_state.current_mode = newMode;
    system_state.mode_start_time = millis();
    
    Serial.print("Mode: ");
    switch (newMode) {
        case MODE_E131:
            Serial.println("E1.31 (xLights)");
            digitalWrite(BUTTON_LED_PIN, LOW);
            break;
        case MODE_STANDALONE:
            Serial.println("Standalone");
            digitalWrite(BUTTON_LED_PIN, HIGH);
            break;
        case MODE_IDLE:
            Serial.println("Idle");
            digitalWrite(BUTTON_LED_PIN, LOW);
            break;
        case MODE_STARTUP:
            Serial.println("Startup");
            break;
    }
}

void updateSystemMode() {
    // E1.31 has highest priority
    if (isE131Active()) {
        if (system_state.current_mode != MODE_E131) {
            // E1.31 activated, stop any standalone playback
            if (system_state.current_mode == MODE_STANDALONE) {
                stopStandalonePlayback();
            }
            switchMode(MODE_E131);
        }
        return;
    }
    
    // E1.31 not active
    if (system_state.current_mode == MODE_E131) {
        // E1.31 just became inactive
        switchMode(MODE_IDLE);
        return;
    }
    
    // Check if standalone playback finished
    if (system_state.current_mode == MODE_STANDALONE) {
        if (!system_state.audio_playing && !system_state.animation_playing) {
            // Standalone finished
            stopStandalonePlayback();
            system_state.last_trigger_time = millis();
        }
    }
    
    // Idle timeout
    if (system_state.current_mode == MODE_IDLE) {
        // Just stay idle, waiting for trigger
    }
}

// ============================================
// TRIGGER HANDLING
// ============================================

void updateButtonState() {
    button.previous = button.current;
    button.current = !digitalRead(BUTTON_PIN);  // Active LOW
    
    // Debouncing
    static unsigned long last_change = 0;
    if (button.current != button.previous) {
        if (millis() - last_change < BUTTON_DEBOUNCE_MS) {
            button.current = button.previous;
            return;
        }
        last_change = millis();
    }
    
    // Button pressed (rising edge)
    if (button.current && !button.previous) {
        button.press_time = millis();
        button.long_press_triggered = false;
        stats.button_presses++;
    }
    
    // Button released (falling edge)
    if (!button.current && button.previous) {
        button.release_time = millis();
        unsigned long press_duration = button.release_time - button.press_time;
        
        // Short press
        if (press_duration < BUTTON_LONG_PRESS_MS && !button.long_press_triggered) {
            Serial.println("\n🔔 Button pressed!");
            
            // Trigger standalone mode if not E1.31
            if (!isE131Active()) {
                if (system_state.current_mode == MODE_STANDALONE) {
                    stopStandalonePlayback();
                } else {
                    startStandalonePlayback();
                }
            } else {
                Serial.println("Button ignored: E1.31 active");
            }
        }
    }
    
    // Long press detection (while still pressed)
    if (button.current && !button.long_press_triggered) {
        if (millis() - button.press_time > BUTTON_LONG_PRESS_MS) {
            button.long_press_triggered = true;
            Serial.println("Button long press - Emergency stop");
            stopStandalonePlayback();
            switchMode(MODE_IDLE);
        }
    }
}

void updatePIRState() {
    bool pir_current = digitalRead(PIR_SENSOR_PIN);
    
    // PIR triggered (rising edge)
    if (pir_current && !pir_triggered) {
        // Check cooldown
        if (millis() - last_pir_trigger > PIR_COOLDOWN_MS) {
            Serial.println("\n👤 Motion detected!");
            stats.motion_detects++;
            last_pir_trigger = millis();
            
            // Trigger standalone mode if not E1.31 and not already playing
            if (!isE131Active() && system_state.current_mode != MODE_STANDALONE) {
                startStandalonePlayback();
            }
        }
    }
    
    pir_triggered = pir_current;
}

// ============================================
// E1.31 PROCESSING
// ============================================

void processE131Packet() {
    if (!e131.isEmpty()) {
        e131_packet_t packet;
        e131.pull(&packet);
        
        stats.packets_received++;
        system_state.last_e131_packet = millis();
        
        // Process servos
        for (int servo = 0; servo < NUM_SERVOS; servo++) {
            if (SERVO_CONFIGS[servo].enabled) {
                int channelIndex = servo * 3;
                if (channelIndex < E131_CHANNELS) {
                    uint8_t value = packet.property_values[channelIndex + 1];
                    updateServoFromE131(servo, value);
                }
            }
        }
    }
}

// ============================================
// MQTT IMPLEMENTATION
// ============================================

unsigned long last_mqtt_reconnect = 0;
unsigned long last_state_publish = 0;
bool mqtt_discovery_sent = false;

void publishHomeAssistantDiscovery() {
    if (!MQTT_ENABLED || !mqttClient.connected()) return;

    Serial.println("\n=== Publishing Home Assistant Discovery ===");

    // Create unique device identifier
    String device_id = String(MQTT_CLIENT_ID);

    // Device information (shared across all entities)
    String device_json = "\"device\":{";
    device_json += "\"identifiers\":[\"" + device_id + "\"],";
    device_json += "\"name\":\"" + String(DEVICE_NAME) + "\",";
    device_json += "\"model\":\"ESP32 Animatronic Controller\",";
    device_json += "\"manufacturer\":\"SleighVo\",";
    device_json += "\"sw_version\":\"" + String(FIRMWARE_VERSION) + "\"";
    device_json += "}";

    // 1. Main Switch - Control standalone mode
    {
        String topic = "homeassistant/switch/" + device_id + "/standalone/config";
        String config = "{";
        config += "\"name\":\"Standalone Mode\",";
        config += "\"unique_id\":\"" + device_id + "_standalone\",";
        config += "\"state_topic\":\"" + String(MQTT_TOPIC_STATE) + "\",";
        config += "\"command_topic\":\"" + String(MQTT_TOPIC_COMMAND) + "/standalone\",";
        config += "\"value_template\":\"{{ value_json.standalone_active }}\",";
        config += "\"payload_on\":\"ON\",";
        config += "\"payload_off\":\"OFF\",";
        config += "\"icon\":\"mdi:animation-play\",";
        config += device_json;
        config += "}";

        mqttClient.publish(topic.c_str(), config.c_str(), true);
        Serial.println("✓ Published standalone switch discovery");
    }

    // 2. Mode Sensor
    {
        String topic = "homeassistant/sensor/" + device_id + "/mode/config";
        String config = "{";
        config += "\"name\":\"Current Mode\",";
        config += "\"unique_id\":\"" + device_id + "_mode\",";
        config += "\"state_topic\":\"" + String(MQTT_TOPIC_STATE) + "\",";
        config += "\"value_template\":\"{{ value_json.mode }}\",";
        config += "\"icon\":\"mdi:state-machine\",";
        config += device_json;
        config += "}";

        mqttClient.publish(topic.c_str(), config.c_str(), true);
        Serial.println("✓ Published mode sensor discovery");
    }

    // 3. E1.31 Active Binary Sensor
    {
        String topic = "homeassistant/binary_sensor/" + device_id + "/e131/config";
        String config = "{";
        config += "\"name\":\"E1.31 Active\",";
        config += "\"unique_id\":\"" + device_id + "_e131\",";
        config += "\"state_topic\":\"" + String(MQTT_TOPIC_STATE) + "\",";
        config += "\"value_template\":\"{{ value_json.e131_active }}\",";
        config += "\"payload_on\":true,";
        config += "\"payload_off\":false,";
        config += "\"device_class\":\"connectivity\",";
        config += device_json;
        config += "}";

        mqttClient.publish(topic.c_str(), config.c_str(), true);
        Serial.println("✓ Published E1.31 sensor discovery");
    }

    // 4. Audio Playing Binary Sensor
    {
        String topic = "homeassistant/binary_sensor/" + device_id + "/audio/config";
        String config = "{";
        config += "\"name\":\"Audio Playing\",";
        config += "\"unique_id\":\"" + device_id + "_audio\",";
        config += "\"state_topic\":\"" + String(MQTT_TOPIC_STATE) + "\",";
        config += "\"value_template\":\"{{ value_json.audio_playing }}\",";
        config += "\"payload_on\":true,";
        config += "\"payload_off\":false,";
        config += "\"icon\":\"mdi:music\",";
        config += device_json;
        config += "}";

        mqttClient.publish(topic.c_str(), config.c_str(), true);
        Serial.println("✓ Published audio sensor discovery");
    }

    // 5. Statistics Sensors
    {
        String topic = "homeassistant/sensor/" + device_id + "/packets/config";
        String config = "{";
        config += "\"name\":\"E1.31 Packets Received\",";
        config += "\"unique_id\":\"" + device_id + "_packets\",";
        config += "\"state_topic\":\"" + String(MQTT_TOPIC_STATE) + "\",";
        config += "\"value_template\":\"{{ value_json.packets_received }}\",";
        config += "\"icon\":\"mdi:counter\",";
        config += device_json;
        config += "}";

        mqttClient.publish(topic.c_str(), config.c_str(), true);
        Serial.println("✓ Published packets sensor discovery");
    }

    // 6. Button - Manual trigger
    {
        String topic = "homeassistant/button/" + device_id + "/trigger/config";
        String config = "{";
        config += "\"name\":\"Trigger Animation\",";
        config += "\"unique_id\":\"" + device_id + "_trigger\",";
        config += "\"command_topic\":\"" + String(MQTT_TOPIC_COMMAND) + "/trigger\",";
        config += "\"payload_press\":\"TRIGGER\",";
        config += "\"icon\":\"mdi:play-circle\",";
        config += device_json;
        config += "}";

        mqttClient.publish(topic.c_str(), config.c_str(), true);
        Serial.println("✓ Published trigger button discovery");
    }

    mqtt_discovery_sent = true;
    Serial.println("✓ Home Assistant discovery complete\n");
}

void publishMQTTState() {
    if (!MQTT_ENABLED || !mqttClient.connected()) return;

    // Create JSON state document
    JsonDocument doc;

    // System mode
    switch (system_state.current_mode) {
        case MODE_E131:
            doc["mode"] = "E1.31";
            break;
        case MODE_STANDALONE:
            doc["mode"] = "Standalone";
            break;
        case MODE_IDLE:
            doc["mode"] = "Idle";
            break;
        case MODE_STARTUP:
            doc["mode"] = "Startup";
            break;
    }

    // State information
    doc["standalone_active"] = (system_state.current_mode == MODE_STANDALONE) ? "ON" : "OFF";
    doc["e131_active"] = isE131Active();
    doc["audio_playing"] = system_state.audio_playing;
    doc["animation_playing"] = system_state.animation_playing;

    // Current content
    if (system_state.current_song.length() > 0) {
        doc["current_song"] = system_state.current_song;
    }
    if (system_state.current_animation.length() > 0) {
        doc["current_animation"] = system_state.current_animation;
    }

    // Statistics
    doc["packets_received"] = stats.packets_received;
    doc["standalone_triggers"] = stats.standalone_triggers;
    doc["button_presses"] = stats.button_presses;
    doc["motion_detects"] = stats.motion_detects;

    // System info
    doc["uptime"] = millis() / 1000;
    doc["wifi_rssi"] = WiFi.RSSI();
    doc["free_heap"] = ESP.getFreeHeap();

    // Serialize and publish
    String output;
    serializeJson(doc, output);
    mqttClient.publish(MQTT_TOPIC_STATE, output.c_str(), false);

    #if DEBUG_ENABLED
    Serial.print("MQTT State: ");
    Serial.println(output);
    #endif
}

void handleMQTTCommand(char* topic, byte* payload, unsigned int length) {
    String message;
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }

    Serial.print("MQTT Command [");
    Serial.print(topic);
    Serial.print("]: ");
    Serial.println(message);

    String topicStr = String(topic);

    // Standalone mode control
    if (topicStr.endsWith("/standalone")) {
        if (message == "ON") {
            if (!isE131Active() && system_state.current_mode != MODE_STANDALONE) {
                startStandalonePlayback();
            }
        } else if (message == "OFF") {
            if (system_state.current_mode == MODE_STANDALONE) {
                stopStandalonePlayback();
            }
        }
    }

    // Manual trigger
    else if (topicStr.endsWith("/trigger")) {
        if (message == "TRIGGER" && !isE131Active() && system_state.current_mode != MODE_STANDALONE) {
            startStandalonePlayback();
        }
    }

    // Stop command
    else if (topicStr.endsWith("/stop")) {
        if (system_state.current_mode == MODE_STANDALONE) {
            stopStandalonePlayback();
        }
    }

    // Publish updated state
    publishMQTTState();
}

void reconnectMQTT() {
    if (!MQTT_ENABLED) return;
    if (WiFi.status() != WL_CONNECTED) return;
    if (mqttClient.connected()) return;

    unsigned long now = millis();
    if (now - last_mqtt_reconnect < MQTT_RECONNECT_INTERVAL) return;

    last_mqtt_reconnect = now;

    Serial.print("Connecting to MQTT broker...");

    bool connected = false;
    if (strlen(MQTT_USER) > 0) {
        connected = mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD);
    } else {
        connected = mqttClient.connect(MQTT_CLIENT_ID);
    }

    if (connected) {
        Serial.println(" ✓ Connected!");
        stats.mqtt_connected = true;

        // Subscribe to command topics
        String cmd_topic = String(MQTT_TOPIC_COMMAND) + "/#";
        mqttClient.subscribe(cmd_topic.c_str());
        Serial.print("Subscribed to: ");
        Serial.println(cmd_topic);

        // Publish discovery and initial state
        publishHomeAssistantDiscovery();
        publishMQTTState();

    } else {
        Serial.print(" ✗ Failed, rc=");
        Serial.println(mqttClient.state());
        stats.mqtt_connected = false;
        mqtt_discovery_sent = false;
    }
}

void setupMQTT() {
    if (!MQTT_ENABLED) {
        Serial.println("\n=== MQTT Disabled ===");
        return;
    }

    Serial.println("\n=== MQTT Setup ===");
    Serial.print("Broker: ");
    Serial.print(MQTT_SERVER);
    Serial.print(":");
    Serial.println(MQTT_PORT);

    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    mqttClient.setCallback(handleMQTTCommand);
    mqttClient.setBufferSize(1024);  // Increase buffer for discovery messages

    Serial.println("✓ MQTT configured");
}

void updateMQTT() {
    if (!MQTT_ENABLED) return;

    // Maintain connection
    if (!mqttClient.connected()) {
        reconnectMQTT();
    } else {
        mqttClient.loop();

        // Periodic state publishing
        unsigned long now = millis();
        if (now - last_state_publish > 30000) {  // Every 30 seconds
            last_state_publish = now;
            publishMQTTState();
        }
    }
}

// ============================================
// WEB SERVER HANDLERS
// ============================================

const char* getModeName(SystemMode mode) {
    switch(mode) {
        case MODE_E131: return "E1.31";
        case MODE_STANDALONE: return "Standalone";
        case MODE_IDLE: return "Idle";
        case MODE_STARTUP: return "Startup";
        default: return "Unknown";
    }
}

void handleRoot() {
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>SleighVo Control</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
            color: #333;
        }
        .container {
            max-width: 900px;
            margin: 0 auto;
        }
        .header {
            text-align: center;
            color: white;
            margin-bottom: 30px;
        }
        .header h1 {
            font-size: 2.5em;
            margin-bottom: 10px;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }
        .header p {
            font-size: 1.1em;
            opacity: 0.9;
        }
        .card {
            background: white;
            border-radius: 12px;
            padding: 25px;
            margin-bottom: 20px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
        }
        .card h2 {
            font-size: 1.5em;
            margin-bottom: 15px;
            color: #667eea;
            border-bottom: 2px solid #f0f0f0;
            padding-bottom: 10px;
        }
        .status-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 15px;
            margin-top: 15px;
        }
        .status-item {
            padding: 15px;
            background: #f8f9fa;
            border-radius: 8px;
            border-left: 4px solid #667eea;
        }
        .status-label {
            font-size: 0.85em;
            color: #666;
            margin-bottom: 5px;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }
        .status-value {
            font-size: 1.3em;
            font-weight: 600;
            color: #333;
        }
        .status-value.active { color: #28a745; }
        .status-value.inactive { color: #dc3545; }
        .status-value.playing { color: #17a2b8; }
        .btn {
            display: inline-block;
            padding: 12px 24px;
            margin: 5px;
            border: none;
            border-radius: 6px;
            font-size: 1em;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.3s ease;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }
        .btn:hover {
            transform: translateY(-2px);
            box-shadow: 0 4px 8px rgba(0,0,0,0.2);
        }
        .btn-primary {
            background: #667eea;
            color: white;
        }
        .btn-primary:hover { background: #5568d3; }
        .btn-success {
            background: #28a745;
            color: white;
        }
        .btn-success:hover { background: #218838; }
        .btn-danger {
            background: #dc3545;
            color: white;
        }
        .btn-danger:hover { background: #c82333; }
        .btn-warning {
            background: #ffc107;
            color: #333;
        }
        .btn-warning:hover { background: #e0a800; }
        .btn:disabled {
            opacity: 0.5;
            cursor: not-allowed;
            transform: none;
        }
        .control-group {
            display: flex;
            flex-wrap: wrap;
            gap: 10px;
            margin-top: 15px;
        }
        .servo-list {
            display: grid;
            grid-template-columns: repeat(auto-fill, minmax(150px, 1fr));
            gap: 10px;
            margin-top: 15px;
        }
        .servo-item {
            padding: 12px;
            background: #f8f9fa;
            border-radius: 6px;
            text-align: center;
        }
        .servo-name {
            font-size: 0.9em;
            color: #666;
            margin-bottom: 5px;
        }
        .servo-angle {
            font-size: 1.5em;
            font-weight: 700;
            color: #667eea;
        }
        .servo-source {
            font-size: 0.75em;
            color: #999;
            margin-top: 3px;
        }
        .stats-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
            gap: 12px;
            margin-top: 15px;
        }
        .stat-box {
            text-align: center;
            padding: 15px;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            border-radius: 8px;
            color: white;
        }
        .stat-value {
            font-size: 2em;
            font-weight: 700;
        }
        .stat-label {
            font-size: 0.85em;
            opacity: 0.9;
            margin-top: 5px;
        }
        .indicator {
            display: inline-block;
            width: 12px;
            height: 12px;
            border-radius: 50%;
            margin-right: 8px;
            animation: pulse 2s infinite;
        }
        .indicator.green { background: #28a745; }
        .indicator.red { background: #dc3545; }
        .indicator.yellow { background: #ffc107; }
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }
        .footer {
            text-align: center;
            color: white;
            margin-top: 30px;
            opacity: 0.8;
            font-size: 0.9em;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🎄 SleighVo Control</h1>
            <p>ESP32 Animatronic Controller</p>
        </div>

        <div class="card">
            <h2>System Status</h2>
            <div class="status-grid">
                <div class="status-item">
                    <div class="status-label">Current Mode</div>
                    <div class="status-value" id="mode">--</div>
                </div>
                <div class="status-item">
                    <div class="status-label">E1.31 Active</div>
                    <div class="status-value" id="e131">--</div>
                </div>
                <div class="status-item">
                    <div class="status-label">Audio Playing</div>
                    <div class="status-value" id="audio">--</div>
                </div>
                <div class="status-item">
                    <div class="status-label">Animation</div>
                    <div class="status-value" id="animation">--</div>
                </div>
                <div class="status-item">
                    <div class="status-label">Uptime</div>
                    <div class="status-value" id="uptime">--</div>
                </div>
                <div class="status-item">
                    <div class="status-label">WiFi RSSI</div>
                    <div class="status-value" id="rssi">--</div>
                </div>
            </div>
        </div>

        <div class="card">
            <h2>Controls</h2>
            <div class="control-group">
                <button class="btn btn-success" onclick="triggerAnimation()">
                    ▶️ Trigger Animation
                </button>
                <button class="btn btn-danger" onclick="stopPlayback()">
                    ⏹️ Stop Playback
                </button>
                <button class="btn btn-warning" onclick="toggleStandalone()" id="standaloneBtn">
                    🔄 Toggle Standalone
                </button>
                <button class="btn btn-primary" onclick="testServos()">
                    🎮 Test Servos
                </button>
            </div>
        </div>

        <div class="card">
            <h2>Servo Positions</h2>
            <div class="servo-list" id="servoList">
                <div class="servo-item">Loading...</div>
            </div>
        </div>

        <div class="card">
            <h2>Statistics</h2>
            <div class="stats-grid">
                <div class="stat-box">
                    <div class="stat-value" id="stat-e131">0</div>
                    <div class="stat-label">E1.31 Packets</div>
                </div>
                <div class="stat-box">
                    <div class="stat-value" id="stat-triggers">0</div>
                    <div class="stat-label">Triggers</div>
                </div>
                <div class="stat-box">
                    <div class="stat-value" id="stat-buttons">0</div>
                    <div class="stat-label">Button Presses</div>
                </div>
                <div class="stat-box">
                    <div class="stat-value" id="stat-motion">0</div>
                    <div class="stat-label">Motion Detects</div>
                </div>
            </div>
        </div>

        <div class="footer">
            <p>SleighVo v1.0.0 | <span id="ip">--</span></p>
        </div>
    </div>

    <script>
        function updateStatus() {
            fetch('/api/status')
                .then(response => response.json())
                .then(data => {
                    // System status
                    document.getElementById('mode').textContent = data.mode;
                    document.getElementById('e131').textContent = data.e131_active ? 'Active' : 'Inactive';
                    document.getElementById('e131').className = 'status-value ' + (data.e131_active ? 'active' : 'inactive');
                    document.getElementById('audio').textContent = data.audio_playing ? 'Playing' : 'Stopped';
                    document.getElementById('audio').className = 'status-value ' + (data.audio_playing ? 'playing' : 'inactive');
                    document.getElementById('animation').textContent = data.animation_playing ? 'Running' : 'Stopped';
                    document.getElementById('animation').className = 'status-value ' + (data.animation_playing ? 'playing' : 'inactive');
                    document.getElementById('uptime').textContent = formatUptime(data.uptime);
                    document.getElementById('rssi').textContent = data.rssi + ' dBm';

                    // Statistics
                    document.getElementById('stat-e131').textContent = data.stats.e131_packets;
                    document.getElementById('stat-triggers').textContent = data.stats.standalone_triggers;
                    document.getElementById('stat-buttons').textContent = data.stats.button_presses;
                    document.getElementById('stat-motion').textContent = data.stats.motion_detects;

                    // Servo positions
                    let servoHTML = '';
                    data.servos.forEach((servo, i) => {
                        if (servo.enabled) {
                            servoHTML += `
                                <div class="servo-item">
                                    <div class="servo-name">${servo.name}</div>
                                    <div class="servo-angle">${servo.angle}°</div>
                                    <div class="servo-source">${servo.source}</div>
                                </div>
                            `;
                        }
                    });
                    document.getElementById('servoList').innerHTML = servoHTML || '<div class="servo-item">No servos enabled</div>';

                    document.getElementById('ip').textContent = data.ip;
                })
                .catch(err => console.error('Status update failed:', err));
        }

        function formatUptime(ms) {
            const seconds = Math.floor(ms / 1000);
            const minutes = Math.floor(seconds / 60);
            const hours = Math.floor(minutes / 60);
            const days = Math.floor(hours / 24);

            if (days > 0) return `${days}d ${hours % 24}h`;
            if (hours > 0) return `${hours}h ${minutes % 60}m`;
            if (minutes > 0) return `${minutes}m ${seconds % 60}s`;
            return `${seconds}s`;
        }

        function triggerAnimation() {
            fetch('/api/trigger', { method: 'POST' })
                .then(response => response.json())
                .then(data => {
                    console.log('Trigger response:', data);
                    updateStatus();
                })
                .catch(err => console.error('Trigger failed:', err));
        }

        function stopPlayback() {
            fetch('/api/stop', { method: 'POST' })
                .then(response => response.json())
                .then(data => {
                    console.log('Stop response:', data);
                    updateStatus();
                })
                .catch(err => console.error('Stop failed:', err));
        }

        function toggleStandalone() {
            fetch('/api/standalone', { method: 'POST' })
                .then(response => response.json())
                .then(data => {
                    console.log('Standalone toggle response:', data);
                    updateStatus();
                })
                .catch(err => console.error('Standalone toggle failed:', err));
        }

        function testServos() {
            fetch('/api/test', { method: 'POST' })
                .then(response => response.json())
                .then(data => {
                    console.log('Test response:', data);
                    updateStatus();
                })
                .catch(err => console.error('Test failed:', err));
        }

        // Update status every 1 second
        setInterval(updateStatus, 1000);

        // Initial update
        updateStatus();
    </script>
</body>
</html>
)rawliteral";

    server.send(200, "text/html", html);
}

void handleAPIStatus() {
    StaticJsonDocument<2048> doc;

    // System info
    doc["mode"] = getModeName(system_state.current_mode);
    doc["e131_active"] = isE131Active();
    doc["audio_playing"] = system_state.audio_playing;
    doc["animation_playing"] = system_state.animation_playing;
    doc["uptime"] = millis();
    doc["rssi"] = WiFi.RSSI();
    doc["ip"] = WiFi.localIP().toString();
    doc["free_heap"] = ESP.getFreeHeap();

    // Statistics
    JsonObject stats_obj = doc.createNestedObject("stats");
    stats_obj["e131_packets"] = stats.packets_received;
    stats_obj["standalone_triggers"] = stats.standalone_triggers;
    stats_obj["button_presses"] = stats.button_presses;
    stats_obj["motion_detects"] = stats.motion_detects;

    // Servo positions
    JsonArray servos = doc.createNestedArray("servos");
    for (int i = 0; i < NUM_SERVOS; i++) {
        JsonObject servo = servos.createNestedObject();
        servo["name"] = SERVO_NAMES[i];
        servo["enabled"] = SERVO_CONFIGS[i].enabled;
        servo["angle"] = servoStates[i].current_angle;
        servo["source"] = servoStates[i].control_source;
    }

    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}

void handleAPITrigger() {
    if (system_state.current_mode != MODE_E131) {
        startStandalonePlayback();
        server.send(200, "application/json", "{\"status\":\"triggered\"}");
    } else {
        server.send(200, "application/json", "{\"status\":\"blocked\",\"reason\":\"E1.31 active\"}");
    }
}

void handleAPIStop() {
    if (system_state.current_mode == MODE_STANDALONE) {
        stopStandalonePlayback();
    }
    server.send(200, "application/json", "{\"status\":\"stopped\"}");
}

void handleAPIStandalone() {
    if (system_state.current_mode == MODE_STANDALONE) {
        stopStandalonePlayback();
        server.send(200, "application/json", "{\"status\":\"disabled\"}");
    } else if (system_state.current_mode == MODE_IDLE) {
        startStandalonePlayback();
        server.send(200, "application/json", "{\"status\":\"enabled\"}");
    } else {
        server.send(200, "application/json", "{\"status\":\"unavailable\",\"reason\":\"E1.31 active\"}");
    }
}

void handleAPITest() {
    // Queue a servo test (don't block)
    server.send(200, "application/json", "{\"status\":\"test_queued\"}");
    // Note: Actual test would need to be non-blocking or queued
}

void setupWebServer() {
    if (!ENABLE_WEB_SERVER) {
        Serial.println("\n=== Web Server DISABLED ===");
        return;
    }

    Serial.println("\n=== Web Server Setup ===");

    // Route handlers
    server.on("/", handleRoot);
    server.on("/api/status", handleAPIStatus);
    server.on("/api/trigger", HTTP_POST, handleAPITrigger);
    server.on("/api/stop", HTTP_POST, handleAPIStop);
    server.on("/api/standalone", HTTP_POST, handleAPIStandalone);
    server.on("/api/test", HTTP_POST, handleAPITest);

    // 404 handler
    server.onNotFound([]() {
        server.send(404, "text/plain", "404: Not Found");
    });

    server.begin();

    Serial.print("✓ Web server started on http://");
    Serial.print(WiFi.localIP());
    Serial.print(":");
    Serial.println(WEB_SERVER_PORT);
}

// ============================================
// SETUP
// ============================================

void setup() {
    #ifdef DISABLE_BROWNOUT_DETECTOR
    // Disable brownout detector (use better power supply instead)
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    #endif

    Serial.begin(115200);
    delay(1000);

    Serial.println("\n\n=========================================");
    Serial.println("   ESP32 Servo Controller");
    Serial.println("   with Standalone Mode");
    Serial.println("=========================================");
    
    setupWiFi();
    setupPCA9685();

    #if TEST_SERVOS_ON_STARTUP
    testAllServos();  // Test all servos on startup
    #endif

    setupE131();
    setupSDCard();
    setupAudio();
    setupTriggers();
    setupMQTT();
    setupWebServer();

    switchMode(MODE_IDLE);
    
    Serial.println("\n=========================================");
    Serial.println("✓ System Ready!");
    Serial.println("=========================================");
    Serial.println("Modes:");
    Serial.println("  - E1.31: xLights show (priority)");
    Serial.println("  - Standalone: Local playback");
    Serial.println("  - Idle: Waiting for trigger");
    Serial.println("\nTriggers:");
    Serial.println("  - Button press (short)");
    Serial.println("  - Motion sensor");
    Serial.println("=========================================\n");
}

// ============================================
// MAIN LOOP
// ============================================

void loop() {
    // Process E1.31
    processE131Packet();
    
    // Update system mode
    updateSystemMode();
    
    // Update triggers (only if not in E1.31 mode)
    if (system_state.current_mode != MODE_E131) {
        updateButtonState();
        updatePIRState();
    }
    
    // Update standalone playback
    if (system_state.current_mode == MODE_STANDALONE) {
        updateAudioPlayback();
        updateAnimation();
    }
    
    // Update idle animation
    if (system_state.current_mode == MODE_IDLE) {
        updateIdleAnimation();
    }

    // MQTT
    updateMQTT();

    // Web server
    if (ENABLE_WEB_SERVER) {
        server.handleClient();
    }

    delay(1);
}
// ```

// ---

// ## SD Card Preparation

// ### Create this folder structure on SD card:
// ```
// SD Card Root/
// ├── songs/
// │   ├── jingle_bells.mp3
// │   ├── rudolph.mp3
// │   └── frosty.mp3
// └── animations/
//     ├── jingle_bells.txt
//     ├── rudolph.txt
//     └── frosty.txt