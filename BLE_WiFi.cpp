#include <Arduino.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <cstring> // Required for strcasestr

// ==========================================
// PIN CONFIGURATION & COOLDOWNS
// ==========================================
// UART Pins to connect to Board #1
// Wire Board #2 TX (17) to Board #1 RX (4)
// Wire Board #2 RX (16) to Board #1 TX (5)
constexpr uint8_t RX_PIN = 16; 
constexpr uint8_t TX_PIN = 17; 
bool wifiScanRunning = false;

// Flock Safety / ALPR known MAC Address OUIs
const uint32_t FLOCK_OUIS[] = {
    0x70C94E, 0x3C9180, 0xD8F3BC, 0x803049, 0xB83532,
    0x145AFC, 0x744CA1, 0x083A88, 0x9C2F9D, 0xC03532,
    0x940853, 0xE4AAEA, 0xF46ADD, 0xF8A2D6, 0x24B2B9,
    0x00F48D, 0xD03957, 0xE8D0FC, 0xE04F43, 0xB81EA4,
    0x700894, 0x588E81, 0xEC1BBD, 0x3C71BF, 0x5800E3,
    0x9035EA, 0x5C93A2, 0x646E69, 0x4827EA, 0xA4CF12,
    0x826BF2, 0xB41E52
};
constexpr int FLOCK_OUI_COUNT = 32;

constexpr uint16_t COMPANY_ID_AXON = 0x0590; 

// Prevent spamming Board #1 with alerts for the same device
constexpr unsigned long ALERT_COOLDOWN_MS = 10000; 
unsigned long lastBleAlert = 0;
unsigned long lastWifiAlert = 0;

// ==========================================
// DYNAMIC SPEED HYSTERESIS ENGINE
// ==========================================
enum class SpeedTier { SLOW, CITY, HIGHWAY };

SpeedTier currentTier = SpeedTier::CITY;
int currentSpeedMph = 0;

// Scan Durations (Wi-Fi is given more time than BLE in all tiers)
int bleScanSec = 2;          // BLE takes seconds (integer)
unsigned long wifiScanMs = 3000; // Wi-Fi takes milliseconds

void updateSpeedTier() {
    // Hysteresis: We require the speed to move well past the boundary before changing tiers
    // This prevents "thrashing" if you are stuck at exactly 40 mph
    if (currentTier == SpeedTier::CITY) {
        if (currentSpeedMph >= 43) currentTier = SpeedTier::HIGHWAY;
        else if (currentSpeedMph <= 4) currentTier = SpeedTier::SLOW;
    } 
    else if (currentTier == SpeedTier::HIGHWAY) {
        if (currentSpeedMph <= 38) currentTier = SpeedTier::CITY; // Drop back to city only if well below 40
    } 
    else if (currentTier == SpeedTier::SLOW) {
        if (currentSpeedMph >= 8) currentTier = SpeedTier::CITY;  // Move up to city only if actually driving
    }

    // Assign scanning parameters based on the locked-in tier
    if (currentTier == SpeedTier::HIGHWAY) {
        bleScanSec = 1; 
        wifiScanMs = 1500;
    } else if (currentTier == SpeedTier::CITY) {
        bleScanSec = 2; 
        wifiScanMs = 3000;
    } else { // SLOW / PARKED
        bleScanSec = 3; 
        wifiScanMs = 5000;
    }
}

void readSpeedFromMainBoard() {
    static char buffer[32]; 
    static uint8_t index = 0;

    while (Serial1.available() > 0) {
        char c = Serial1.read();

        // If end of message is reached
        if (c == '\n') {
            buffer[index] = '\0'; // Null-terminate the string

            // Parse incoming "SPEED:45" using standard C string functions
            if (strncmp(buffer, "SPEED:", 6) == 0) {
                currentSpeedMph = atoi(&buffer[6]);
            }
            
            index = 0; // Reset index for the next incoming message
        } 
        // Ignore carriage returns, ensure we don't overflow the buffer
        else if (c != '\r' && index < sizeof(buffer) - 1) {
            buffer[index++] = c;
        }
    }
}

// ==========================================
// BLE SCANNER LOGIC
// ==========================================
BLEScan* pBLEScan;

class SurveillanceBleCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) override {
        bool matchFound = false;

        // Get the raw constant character pointer
        const char* deviceName = advertisedDevice.getName().c_str();

        // 1. Check Name (Case-insensitive substring search)
        if (strlen(deviceName) > 0) {
            if (strcasestr(deviceName, "axon") != nullptr || 
                strcasestr(deviceName, "flock") != nullptr || 
                strcasestr(deviceName, "signal") != nullptr) {
                matchFound = true;
            }
        }

        // 2. Check Manufacturer ID 
        if (!matchFound && advertisedDevice.haveManufacturerData()) {
            std::string rawData = advertisedDevice.getManufacturerData();
            if (rawData.length() >= 2) {
                uint16_t companyId = ((uint8_t)rawData[1] << 8) | (uint8_t)rawData[0];
                if (companyId == COMPANY_ID_AXON) {
                    matchFound = true;
                }
            }
        }

        if (matchFound) {
            unsigned long currentMS = millis();
            if (currentMS - lastBleAlert >= ALERT_COOLDOWN_MS) {
                lastBleAlert = currentMS;
                Serial1.println("BLE_ALERT"); 
                Serial.printf("[BLE TARGET] Found: %s\n", deviceName);
            }
        }
    }
};

// ==========================================
// SETUP
// ==========================================
void setup() {
    Serial.begin(115200); // For USB Debugging
    
    // Hardware Serial1 to communicate with the Main ESP32
    Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

    // Init BLE
    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new SurveillanceBleCallbacks());
    pBLEScan->setActiveScan(true); 
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);

    // Init Wi-Fi in station mode (we just need the radio on for scanning)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    
    Serial.println("[BOOT] Secondary Scanner Ready.");
}

// ==========================================
// MAIN LOOP: TIME-SLICED STATE MACHINE
// ========================================== 
void loop() {
    // 1. Read incoming speed from Main Board and adjust timing tiers
    readSpeedFromMainBoard();
    updateSpeedTier();

    // ----------------------------------------------------
    // PHASE 1: BLUETOOTH SCAN
    // ----------------------------------------------------
    // pBLEScan->start() is blocking. It will safely hold here for exactly 'bleScanSec'
    pBLEScan->start(bleScanSec, false);
    pBLEScan->clearResults(); 

    // Flush the serial buffer between phases so we don't miss speed updates
    readSpeedFromMainBoard(); 

    // ----------------------------------------------------
    // PHASE 2: WI-FI SCAN
    // ----------------------------------------------------
    if (!wifiScanRunning) {
        // Start a scan asynchronously (true, true)
        WiFi.scanNetworks(true, true);
        wifiScanRunning = true;
    }

    // Check if the background scan is finished without blocking
    int16_t networkCount = WiFi.scanComplete();
    
    if (networkCount >= 0) { // Scan is done!
        for (int i = 0; i < networkCount; ++i) {
            
            // Safely extract the SSID string and convert to c_str 
            // We store it in a local String object first to prevent the pointer from dangling
            String ssidStr = WiFi.SSID(i); 
            const char* ssid = ssidStr.c_str();
            
            // Get the raw BSSID (MAC Address) byte array
            uint8_t* bssid = WiFi.BSSID(i); 

            // Shift the first 3 bytes into a single 32-bit integer for fast comparison
            uint32_t targetOUI = (bssid[0] << 16) | (bssid[1] << 8) | bssid[2];

            bool isFlockHardware = false;
            
            // 1. Check against the Flock OUI list
            for (int j = 0; j < FLOCK_OUI_COUNT; j++) {
                if (targetOUI == FLOCK_OUIS[j]) {
                    isFlockHardware = true;
                    break;
                }
            }

            // 2. Trigger the appropriate alert
            unsigned long currentMS = millis();
            
            if (isFlockHardware) {
                if (currentMS - lastWifiAlert >= ALERT_COOLDOWN_MS) {
                    lastWifiAlert = currentMS;
                    // Send a specific FLOCK_ALERT so Board #1 knows exactly what it is
                    Serial1.println("FLOCK_ALERT"); 
                    // Print the first 3 bytes of the MAC to the debug console
                    Serial.printf("[WIFI TARGET] Flock OUI Found: %02x:%02x:%02x:...\n", bssid[0], bssid[1], bssid[2]);
                }
            } 
            // 3. Fast Case-Insensitive search on the SSID
            else if (strcasestr(ssid, "axon") != nullptr || 
                     strcasestr(ssid, "flock") != nullptr || 
                     strcasestr(ssid, "camera") != nullptr ||
                     strcasestr(ssid, "surveillance") != nullptr) {
                
                if (currentMS - lastWifiAlert >= ALERT_COOLDOWN_MS) {
                    lastWifiAlert = currentMS;
                    // Send a generic WIFI_ALERT for unknown/suspicious networks
                    Serial1.println("WIFI_ALERT"); 
                    Serial.printf("[WIFI TARGET] Suspicious SSID: %s\n", ssid);
                }
            }
        }
        
        // Clear memory buffer and reset the flag so the next scan can trigger
        WiFi.scanDelete();
        wifiScanRunning = false;
        
    } 
    else if (networkCount == WIFI_SCAN_FAILED) {
        // Failsafe in case the internal Wi-Fi radio crashes
        wifiScanRunning = false; 
    }
}