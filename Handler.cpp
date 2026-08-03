#include <Arduino.h>
#include <driver/i2s.h>
#include <cmath>
#include <TinyGPS++.h>
#include <LittleFS.h>

TinyGPSPlus gps;

// Pin Definitions
constexpr uint8_t GPS_Rx = 1;  // Rx signifies that the ESP32 is recieving data from the GPS
constexpr uint8_t GPS_Tx = 42; // Tx signifies that the ESP32 is sending data to the GPS
constexpr uint8_t WIFI_Rx = 4; // Recieving data from the secondary ESP32 (Wi-Fi & BLE Sweeping)
constexpr uint8_t WIFI_Tx = 5; // Transmits GPS speed
constexpr uint8_t PotCLK = 11; // Rotary Encoder Pin A
constexpr uint8_t PotDT = 13;  // Rotary Encoder Pin B
constexpr uint8_t PotSW = 14;  // Rotary Encoder Switch
constexpr uint8_t DacBCK = 45; // Digital -> Audio Converter Bit Clock
constexpr uint8_t DacDIN = 38; // Digital -> Audio Converter Data In
constexpr uint8_t DacLCK = 47; // Digital -> Audio Converter Word Select

// System Constants
constexpr float ALERT_DISTANCE_METERS = 150.0f; // Distance threshold in meters 
constexpr unsigned long CAMERA_COOLDOWN_MS = 15000; // Cooldown between camera alerts
constexpr int SAMPLE_RATE = 16000; // Stuff to pre-calculate sin waves, to save CPU cycles during audio playback
constexpr int LUT_SIZE = 1024;
int16_t sineLUT[LUT_SIZE];

// Global Trackers
unsigned long lastCameraScan = 0;
unsigned long lastCameraAlertTime = 0;

volatile double volume = 0.5; 
volatile int8_t encoderPosition = 10;

enum class SystemState {
    ENABLED,
    DISABLED
};

enum class AnnounceModes {
    ALL,
    SOME,
    BEEP
};

enum class ButtonEvent {
    NONE,
    SINGLE,
    DOUBLE
};

enum class AlertType {
    FLOCK_CAMERA,      // Known Flock Safety Camera
    GENERIC_ALPR,      // Generic ALPR / Axis / Traffic Camera
    WIFI_BLE_TARGET    // Signal received from Secondary Board (Wi-Fi or BLE)
};

SystemState sysState = SystemState::ENABLED;
AnnounceModes announceMode = AnnounceModes::ALL;

struct CameraPoint {
    float lat;      // 4 bytes
    float lng;      // 4 bytes
    uint8_t type;   // 1 byte (0 = Generic ALPR, 1 = Flock Safety)
};

struct AudioCommand {
    bool isWav;
    char filepath[64];
    int freq;
    int duration;
};
QueueHandle_t audioQueue;

void audioTask(void *pvParameters) {
    AudioCommand cmd;
    while (true) {
        if (xQueueReceive(audioQueue, &cmd, portMAX_DELAY)) {
            float safeVolume = encoderPosition / 20.0f; 

            if (cmd.isWav) {
                void playWavFileTask(const char* filepath, float currentVolume);
            } else {
                void playToneTask(cmd.freq, cmd.duration, safeVolume);
            }
        }
    }
}

// Forward Declarations
void handleButtonActions();
void updateGPS();
void scanForNearbyCameras(float myLat, float myLng);
void playToneTask(int freq, int duration, double volume);
void playWavFile(const char* filepath);
void triggerAlert(AlertType type, bool isNewTarget);
void onDeviceDetected(bool isNewTarget, bool isFlock);
void updateSecondaryBoardListener();

// ==========================================
// HARDWARE INTERRUPT FOR ROTARY ENCODER
// ==========================================
void IRAM_ATTR readEncoderISR() {
    static uint8_t lastCLK = HIGH;
    uint8_t currentCLK = digitalRead(PotCLK);
    uint8_t currentDT  = digitalRead(PotDT);

    if (lastCLK == HIGH && currentCLK == LOW) {
        if (currentDT == LOW) {
            if (encoderPosition < 20) encoderPosition++;
        } else {
            if (encoderPosition > 0)  encoderPosition--;
        }
    }
    lastCLK = currentCLK;
}

// ==========================================
// SETUP
// ==========================================
void setup() {
    Serial.begin(115200);
    for (int i = 0; i < LUT_SIZE; i++) {
        // Pre-calculate a perfect sine wave and store it in RAM
        sineLUT[i] = (int16_t)(32767.0 * sin(2.0 * M_PI * i / LUT_SIZE));
    }
    // Configure Encoder Pins with Internal Pullups
    pinMode(PotCLK, INPUT_PULLUP); 
    pinMode(PotDT, INPUT_PULLUP);
    pinMode(PotSW, INPUT_PULLUP);

    // Attach Hardware Interrupt to CLK pin for 100% reliable rotary tracking
    attachInterrupt(digitalPinToInterrupt(PotCLK), readEncoderISR, CHANGE);

    if (!LittleFS.begin(true)) { 
        Serial.println("[FS] LittleFS Mount Failed!");
    } else {
        Serial.println("[FS] LittleFS Mounted Successfully.");
    }

    Serial1.begin(9600, SERIAL_8N1, GPS_Rx, GPS_Tx);  // GPS Interface
    Serial2.begin(115200, SERIAL_8N1, WIFI_Rx, WIFI_Tx);   // Comms from Secondary ESP32 Board

    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX), 
        .sample_rate = 16000,                               
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,       
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,       
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,  
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,           
        .dma_buf_count = 8,                                 
        .dma_buf_len = 64                                   
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = DacBCK,            
        .ws_io_num = DacLCK,             
        .data_out_num = DacDIN,          
        .data_in_num = I2S_PIN_NO_CHANGE 
    };

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);

    audioQueue = xQueueCreate(10, sizeof(AudioCommand));
    xTaskCreatePinnedToCore(audioTask, "AudioTask", 4096, NULL, 1, NULL, 0);
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
    handleButtonActions();
    updateGPS();
    updateSecondaryBoardListener();

    unsigned long currentMS = millis();
    if (currentMS - lastCameraScan >= 1000) {
        lastCameraScan = currentMS;

        // 1. Send updated speed to Board #2
        sendSpeedToSecondary();

        // 2. Scan for nearby GPS cameras on LittleFS
        if (sysState == SystemState::ENABLED && gps.location.isValid()) {
            float currentLat = gps.location.lat();
            float currentLng = gps.location.lng();

            scanForNearbyCameras(currentLat, currentLng);
        }
    }
}

// ==========================================
// AUDIO ENGINE & LITTLEFS READER
// ==========================================
const char* getAudioPath(const char* filename) {
    static char pathBuffer[64];
    snprintf(pathBuffer, sizeof(pathBuffer), "/Audio/%s", filename);
    return pathBuffer;
}
void playWavFileTask(const char* filepath) {
    AudioCommand cmd;
    cmd.isWav = true;
    strlcpy(cmd.filepath, filepath, sizeof(cmd.filepath));
    xQueueSend(audioQueue, &cmd, 0); // Send to background task instantly
}

void playWavFile(const char* filepath) {
    if (!LittleFS.exists(filepath)) {
        Serial.printf("[AUDIO ERROR] Missing file: %s\n", filepath);
        return;
    }

    File file = LittleFS.open(filepath, "r");
    if (!file) return;

    file.seek(44); // Skip 44-byte WAV header

    constexpr size_t BUFFER_SIZE = 512;
    uint8_t rawBuffer[BUFFER_SIZE];
    int16_t sampleBuffer[BUFFER_SIZE / 2];

    size_t bytesRead = 0;
    size_t bytesWritten = 0;

    while (file.available()) {
        bytesRead = file.read(rawBuffer, BUFFER_SIZE);
        int sampleCount = bytesRead / 2;

        for (int i = 0; i < sampleCount; ++i) {
            int16_t rawSample = (int16_t)(rawBuffer[i * 2] | (rawBuffer[i * 2 + 1] << 8));
            sampleBuffer[i] = (int16_t)(rawSample * volume);
        }

        i2s_write(I2S_NUM_0, sampleBuffer, sampleCount * sizeof(int16_t), &bytesWritten, portMAX_DELAY);
    }

    file.close();
}

void playToneTask(int freq, int duration) {
    AudioCommand cmd;
    cmd.isWav = false;
    cmd.freq = freq;
    cmd.duration = duration;
    xQueueSend(audioQueue, &cmd, 0);
}

void playToneTask(int freq, int duration, double currentVolume) {
    if (currentVolume > 1.0) currentVolume = 1.0;
    if (currentVolume < 0.0) currentVolume = 0.0;

    int totalSamples = (duration * SAMPLE_RATE) / 1000;
    constexpr size_t BUFFER_SIZE = 128;
    int16_t buffer[BUFFER_SIZE];
    int samplesProcessed = 0;

    // How fast we move through the Lookup Table based on frequency
    float phaseInc = (freq * (float)LUT_SIZE) / SAMPLE_RATE;
    float phase = 0;

    while (samplesProcessed < totalSamples) {
        int samplesToProcess = std::min((int)BUFFER_SIZE, totalSamples - samplesProcessed);

        for (int i = 0; i < samplesToProcess; ++i) {
            buffer[i] = (int16_t)(sineLUT[(int)phase % LUT_SIZE] * currentVolume);
            phase += phaseInc;
        }

        size_t bytesWritten;
        i2s_write(I2S_NUM_0, buffer, samplesToProcess * sizeof(int16_t), &bytesWritten, portMAX_DELAY);
        samplesProcessed += samplesToProcess;
    }
}


ButtonEvent checkButtonEvent() { // Check Rotary Encoder
    static unsigned long lastClickTime = 0;
    static int clickCount = 0;
    static bool lastButtonState = HIGH;
    bool currentButtonState = digitalRead(PotSW);
    unsigned long currentMS = millis();

    if (lastButtonState == HIGH && currentButtonState == LOW) { 
        if (currentMS - lastClickTime >= 50) {
            clickCount++;
            lastClickTime = currentMS;
        }
    }
    lastButtonState = currentButtonState;

    if (clickCount > 0 && (currentMS - lastClickTime >= 300)) { 
        ButtonEvent event = (clickCount == 1) ? ButtonEvent::SINGLE : ButtonEvent::DOUBLE;
        clickCount = 0; 
        return event;
    }

    return ButtonEvent::NONE;
}

void handleButtonActions() {  // Handle what happens when the rotary encoder is pressed one time or two times
    ButtonEvent event = checkButtonEvent();
    if (event == ButtonEvent::SINGLE) {
        if (sysState == SystemState::ENABLED) {
            sysState = SystemState::DISABLED;
            playWavFile(getAudioPath("Detector Disabled.wav"));
        } else {
            sysState = SystemState::ENABLED;
            playWavFile(getAudioPath("Detector Enabled.wav"));
        }
    } else if (event == ButtonEvent::DOUBLE) {
        if (sysState == SystemState::ENABLED) {
            if (announceMode == AnnounceModes::ALL) {
                announceMode = AnnounceModes::SOME;
                playWavFile(getAudioPath("Unknown Only.wav"));
            } else if (announceMode == AnnounceModes::SOME) {
                announceMode = AnnounceModes::BEEP;
                playWavFile(getAudioPath("None.wav"));
            } else {
                announceMode = AnnounceModes::ALL;
                playWavFile(getAudioPath("All Announcements.wav"));
            }
        }
    }
}

void triggerAlert(AlertType type, bool isNewTarget) {
    if (sysState == SystemState::DISABLED) return;

    switch (announceMode) {
        case AnnounceModes::ALL:
            if (type == AlertType::FLOCK_CAMERA) {
                playWavFile(getAudioPath("Flock Detected.wav"));
            } else if (type == AlertType::GENERIC_ALPR) {
                playWavFile(getAudioPath("Generic Detected.wav"));
            } else if (type == AlertType::WIFI_BLE_TARGET) {
                playWavFile(getAudioPath("Unknown - Wifi.wav"));
            }
            break;

        case AnnounceModes::SOME:
            if (isNewTarget) {
                playWavFile(getAudioPath("Unknown - Wifi.wav"));
            } else {
                playToneTask(1200, 80); 
            }
            break;

        case AnnounceModes::BEEP:
            playToneTask(1000, 60); 
            break;
    }
}

void onDeviceDetected(bool isNewTarget, bool isFlock) {
    AlertType type = isFlock ? AlertType::FLOCK_CAMERA : AlertType::GENERIC_ALPR;
    triggerAlert(type, isNewTarget);
}

void updateGPS() {
    while (Serial1.available() > 0) {
        char c = Serial1.read();
        gps.encode(c); 
    }
}

float calculateDistanceMeters(float lat1, float lon1, float lat2, float lon2) { // Changed from Haversine for efficiency
    constexpr float EARTH_RADIUS = 6371000.0f;
    constexpr float DEG_TO_RAD = M_PI / 180.0f;

    float lat1Rad = lat1 * DEG_TO_RAD;
    float lat2Rad = lat2 * DEG_TO_RAD;
    float lon1Rad = lon1 * DEG_TO_RAD;
    float lon2Rad = lon2 * DEG_TO_RAD;

    float x = (lon2Rad - lon1Rad) * cos((lat1Rad + lat2Rad) / 2.0f);
    float y = (lat2Rad - lat1Rad);

    return EARTH_RADIUS * sqrt(x * x + y * y);
}

void scanForNearbyCameras(float myLat, float myLng) {
    if (millis() - lastCameraAlertTime < CAMERA_COOLDOWN_MS) return;

    File file = LittleFS.open("/cameras.bin", "r");
    if (!file) return;

    size_t recordSize = sizeof(CameraPoint);
    size_t totalRecords = file.size() / recordSize;
    if (totalRecords == 0) return;

    int low = 0; 
    int high = totalRecords - 1;
    int mid = 0;
    CameraPoint cam;
    bool foundChunk = false;

    // 1. Binary search to find only cameras near the current latitude band (±0.002 degrees)
    while (low <= high) {
        mid = low + (high - low) / 2;
        file.seek(mid * recordSize);
        file.read((uint8_t*)&cam, recordSize);

        if (cam.lat < myLat - 0.002f) {
            low = mid + 1; // Look higher
        } else if (cam.lat > myLat + 0.002f) {
            high = mid - 1; // Look lower
        } else {
            foundChunk = true; 
            break; 
        }
    }

    if (foundChunk) { // Go to the first element of the chunk
        int searchStart = mid;
        while (searchStart > 0) {
            file.seek((searchStart - 1) * recordSize);
            file.read((uint8_t*)&cam, recordSize);
            if (cam.lat < myLat - 0.002f) break; 
            searchStart--;
        }


        file.seek(searchStart * recordSize); // Check all cameras in the chunk if they're close to
        while (file.read((uint8_t*)&cam, recordSize) == recordSize) {
            if (cam.lat > myLat + 0.002f) break; 
            if (std::abs(cam.lng - myLng) < 0.002f) {
                float distance = calculateDistanceMeters(myLat, myLng, cam.lat, cam.lng);
                if (distance <= ALERT_DISTANCE_METERS) {
                    lastCameraAlertTime = millis(); 
                    onDeviceDetected(false, (cam.type == 1)); 
                    break; 
                }
            }
        }
    }
    file.close();
}

void updateSecondaryBoardListener() {
    while (Serial2.available() > 0) {
        String msg = Serial2.readStringUntil('\n');
        msg.trim();

        if (msg.length() == 0) continue;
        if (msg == "FLOCK_ALERT") { // If the Wifi/BLE detects a Flock Camera (Through Mac, SSID's, etc)
            triggerAlert(AlertType::FLOCK_CAMERA, true);
        } 
        else if (msg == "WIFI_ALERT" || msg == "BLE_ALERT") { 
            triggerAlert(AlertType::WIFI_BLE_TARGET, true);
        } 
    }
}
void sendSpeedToSecondary() { // How long it scans WiFi and BLE before switching is based on speed 
    static int speedHistory[5] = {0, 0, 0, 0, 0};
    static uint8_t historyIndex = 0;
    static int lastSentSpeed = -1;

    int currentSpeed = 0;

    if (gps.location.isValid() && gps.speed.isValid()) {
        currentSpeed = (int)gps.speed.mph();
    }

    speedHistory[historyIndex] = currentSpeed;
    historyIndex = (historyIndex + 1) % 5;

    int sum = 0; // Uses 5 samples and finds the avg
    for (int i = 0; i < 5; ++i) {
        sum += speedHistory[i];
    }
    int smoothedSpeed = sum / 5;

    if (smoothedSpeed != lastSentSpeed) { // Only send data if the average changes
        lastSentSpeed = smoothedSpeed;
        Serial2.printf("SPEED:%d\n", smoothedSpeed);
        //Serial.printf("[COMMS] Sent to Board #2: SPEED:%d\n", smoothedSpeed);
    }
}
