#include <Arduino.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <cstring> 

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


enum class SpeedTier { SLOW, CITY, HIGHWAY }; // Dynamically changes scan rate based on speed

SpeedTier currentTier = SpeedTier::CITY;
int currentSpeedMph = 0;

int bleScanSec = 2;          // BLE takes seconds (integer)
unsigned long wifiScanMs = 3000; // Wi-Fi takes milliseconds

void updateSpeedTier() {
    // Uses Hysterisis so sitting at 40mph would not confuse the processor
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

    if (currentTier == SpeedTier::HIGHWAY) {
        bleScanSec = 1; 
        wifiScanMs = 1500;
    } else if (currentTier == SpeedTier::CITY) {
        bleScanSec = 2; 
        wifiScanMs = 3000;
    } else { 
        bleScanSec = 3; 
        wifiScanMs = 5000;
    }
}

void readSpeedFromMainBoard() {
    static char buffer[32]; 
    static uint8_t index = 0;

    while (Serial1.available() > 0) {
        char c = Serial1.read();

        if (c == '\n') {
            buffer[index] = '\0'; 

            // Parse the message to get the speed
            if (strncmp(buffer, "SPEED:", 6) == 0) {
                currentSpeedMph = atoi(&buffer[6]);
            }
            
            index = 0; 
        } 
        else if (c != '\r' && index < sizeof(buffer) - 1) {
            buffer[index++] = c;
        }
    }

BLEScan* pBLEScan;

class SurveillanceBleCallbacks : public BLEAdvertisedDeviceCallbacks { // BLE Scanner logic
    void onResult(BLEAdvertisedDevice advertisedDevice) override {
        bool matchFound = false;

        const char* deviceName = advertisedDevice.getName().c_str();

        if (strlen(deviceName) > 0) { // Check the name
            if (strcasestr(deviceName, "axon") != nullptr || 
                strcasestr(deviceName, "flock") != nullptr || 
                strcasestr(deviceName, "signal") != nullptr) {
                matchFound = true;
            }
        }

        if (!matchFound && advertisedDevice.haveManufacturerData()) { // Check the Manufacturer ID
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

void setup() {
    Serial.begin(115200); 
    Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

    BLEDevice::init(""); // Initialize the scanner
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new SurveillanceBleCallbacks());
    pBLEScan->setActiveScan(true); 
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);

    WiFi.mode(WIFI_STA); // Initialize WiFI in Station Mode
    WiFi.disconnect();
    delay(100);
    
    Serial.println("[BOOT] Secondary Scanner Ready.");
}

void loop() {
    readSpeedFromMainBoard(); // Change scan rates if applicable
    updateSpeedTier();

    pBLEScan->start(bleScanSec, false);
    pBLEScan->clearResults(); 

    readSpeedFromMainBoard(); 

    if (!wifiScanRunning) {
        WiFi.scanNetworks(true, true); // Start an Async scan
        wifiScanRunning = true;
    }

    int16_t networkCount = WiFi.scanComplete(); // Check if the scan is finished without blocking
    
    if (networkCount >= 0) { // When the scan is done
        for (int i = 0; i < networkCount; ++i) {
            String ssidStr = WiFi.SSID(i); 
            const char* ssid = ssidStr.c_str();
            
            uint8_t* bssid = WiFi.BSSID(i); // Get the MAC Address
            uint32_t targetOUI = (bssid[0] << 16) | (bssid[1] << 8) | bssid[2];
            bool isFlockHardware = false;
            
            for (int j = 0; j < FLOCK_OUI_COUNT; j++) { // Check the MAC against the known list
                if (targetOUI == FLOCK_OUIS[j]) {
                    isFlockHardware = true;
                    break;
                }
            }

            unsigned long currentMS = millis();
            
            if (isFlockHardware) {
                if (currentMS - lastWifiAlert >= ALERT_COOLDOWN_MS) {
                    lastWifiAlert = currentMS;
                    Serial1.println("FLOCK_ALERT"); 
                    Serial.printf("[WIFI TARGET] Flock OUI Found: %02x:%02x:%02x:...\n", bssid[0], bssid[1], bssid[2]);
                }
            } 
            else if (strcasestr(ssid, "axon") != nullptr || // If its not flock but contains suspicious words, send an alert
                     strcasestr(ssid, "flock") != nullptr || 
                     strcasestr(ssid, "camera") != nullptr ||
                     strcasestr(ssid, "surveillance") != nullptr) {
                
                if (currentMS - lastWifiAlert >= ALERT_COOLDOWN_MS) {
                    lastWifiAlert = currentMS;
                    Serial1.println("WIFI_ALERT"); // Generic WiFi alert
                    Serial.printf("[WIFI TARGET] Suspicious SSID: %s\n", ssid);
                }
            }
        }
        
        WiFi.scanDelete(); // Clear memory buffer
        wifiScanRunning = false;
        
    } 
    else if (networkCount == WIFI_SCAN_FAILED) { // If the WiFi scanner crashes
        wifiScanRunning = false; 
    }
}
