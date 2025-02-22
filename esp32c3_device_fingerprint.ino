/*
*******************************************************************************
fingerprint and detect ble/wifi devices and send detection over serial
* 
*******************************************************************************
*/

#include <WiFi.h>
#include <NimBLEDevice.h>
#include <vector>
#include <algorithm>


// ================================
// Pin Definitions
// ================================
#define SERIAL1_RX_PIN  D5 // GPIO5
#define SERIAL1_TX_PIN  D4  // GPIO4

HardwareSerial Serial2(2);

// ------------ Baseline Data ------------
static std::vector<String> baselineWiFi;
static std::vector<String> baselineBLE;
bool isBaselineScan = true;

// ------------ Current and Previous Scan Data ------------
static std::vector<String> currentWiFi;
static std::vector<String> currentBLE;

// Previous scan's detected devices
static std::vector<String> previousDetectedNonWhitelistedWiFi;
static std::vector<String> previousDetectedNonWhitelistedBLE;

// ------------ Non-Whitelisted Devices Data ------------
struct DetectedDevice {
    String macAddress;
};
static std::vector<DetectedDevice> detectedNonWhitelistedWiFi;
static std::vector<DetectedDevice> detectedNonWhitelistedBLE;

// ------------ Flag: Baseline Scan Complete ------------
bool baselineSet = false;

// ------------ Function Declarations ------------
void scanWiFiNetworks(std::vector<String> &results, uint32_t duration_ms = 0); 
void scanBLEDevices(std::vector<String> &results, uint32_t durationSec);
bool isInVector(const std::vector<String> &vec, const String &val);
bool updateDetectedDevices();

// ------------ Bluetooth Scanning Callbacks ------------
class MyAdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks {
public:
    void onResult(NimBLEAdvertisedDevice* advertisedDevice) override {
        // Skip reporting during baseline scan
        if (isBaselineScan) return;
        // Get MAC address of the detected device
        std::string macAddress = advertisedDevice->getAddress().toString();

        // Convert MAC address to uppercase for consistency
        std::transform(macAddress.begin(), macAddress.end(), macAddress.begin(), ::toupper);

        // Check if MAC address starts with any of the baseline prefixes
        bool isWhitelisted = false;
        for (auto &prefix : baselineWiFi) {
            if (macAddress.find(prefix.c_str()) == 0) {
                isWhitelisted = true;
                break;
            }
        }
        if (!isWhitelisted) {
            for (auto &prefix : baselineBLE) {
                if (macAddress.find(prefix.c_str()) == 0) {
                    isWhitelisted = true;
                    break;
                }
            }
        }

        if (!isWhitelisted) {
            String addr = String(macAddress.c_str());
            Serial.println("Detected non-whitelisted device: " + addr);
            // The device is eventually processed in scanBLEDevices()
        }
    }
};

// ------------ Setup Function ------------
void setup() {
    // Short delay for boot stability
    delay(2000);

    // Initialize Serial1 for UART communication
    Serial.begin(115200);
    Serial.println("USB Serial started.");

    // Initialize Serial1 for UART communication
    Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);
    Serial.println("Serial1 started.");

    // Initialize Serial2
    Serial2.begin(115200);
    Serial.println("Serial2 started.");

    // Initialize NimBLE
    NimBLEDevice::init("");
    Serial.println("NimBLE initialized.");

    // Stop any ongoing BLE scan (ensure no devices are detected prematurely)
    NimBLEDevice::getScan()->stop();

    isBaselineScan = true;

    // Perform combined scan for WiFi and BLE for 7 minutes to detect all devices and set baseline
    unsigned long scanStartTime = millis();
while (millis() - scanStartTime < 420000) {  // 7 minutes
    scanWiFiNetworks(baselineWiFi, 10000);   // 10-second WiFi scan
    scanBLEDevices(baselineBLE, 10);         // 10-second BLE scan
    delay(1000);  // Small delay between scans
}

    // Set baseline
    baselineWiFi = currentWiFi;
    baselineBLE = currentBLE;
    baselineSet = true;
    isBaselineScan = false; 

    // Start BLE scanning now after baseline
    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pScan->setActiveScan(true);
    pScan->start(0, nullptr, false);
    Serial.println("BLE scan started (after baseline).");
}

// ------------ Loop Function ------------
void loop() {
    // Conditional Continuous Scanning
    if (baselineSet) {
        // Perform WiFi scan
        currentWiFi.clear();
        Serial.println("Starting WiFi scan...");
        scanWiFiNetworks(currentWiFi, 2000);
        Serial.println("WiFi scan completed.");

        // Perform BLE scan
        currentBLE.clear();
        Serial.println("Starting BLE scan...");
        scanBLEDevices(currentBLE, 2);
        Serial.println("BLE scan completed.");

        // Update detected devices list and check for new devices
        bool newDeviceDetected = updateDetectedDevices();

        // If any new devices were detected, send over Serial1
        if(newDeviceDetected){
            Serial.println("Non-whitelisted device detected!");
            Serial1.println("New device alert");
        }
    }

    delay(3000); // Wait before next scan cycle
}

// ------------ Update Detected Devices List ------------
bool updateDetectedDevices(){
    bool newDeviceDetected = false;

    // Temporary lists for current detected devices
    std::vector<String> currentDetectedWiFi;
    std::vector<String> currentDetectedBLE;

    // Process WiFi
    for(auto &dev : currentWiFi){
        if(!isInVector(baselineWiFi, dev)){
            currentDetectedWiFi.push_back(dev);
        }
    }

    // Process BLE
    for(auto &dev : currentBLE){
        if(!isInVector(baselineBLE, dev)){
            currentDetectedBLE.push_back(dev);
        }
    }

    // Identify new WiFi devices
    for(auto &dev : currentDetectedWiFi){
        if(!isInVector(previousDetectedNonWhitelistedWiFi, dev)){
            newDeviceDetected = true;
            Serial.println("New non-whitelisted WiFi device detected: " + dev);
            Serial2.println("New non-whitelisted WiFi device: " + dev);

            DetectedDevice newDev;
            newDev.macAddress = dev;
            detectedNonWhitelistedWiFi.push_back(newDev);
        }
    }

    // Identify new BLE devices
    for(auto &dev : currentDetectedBLE){
        if(!isInVector(previousDetectedNonWhitelistedBLE, dev)){
            newDeviceDetected = true;
            Serial.println("New non-whitelisted BLE device detected: " + dev);
            Serial2.println("New non-whitelisted BLE device: " + dev);

            DetectedDevice newDev;
            newDev.macAddress = dev;
            detectedNonWhitelistedBLE.push_back(newDev);
        }
    }

    // Update previous-detected lists
    previousDetectedNonWhitelistedWiFi = currentDetectedWiFi;
    previousDetectedNonWhitelistedBLE = currentDetectedBLE;

    // Remove devices that are no longer detected
    detectedNonWhitelistedWiFi.erase(
        std::remove_if(
            detectedNonWhitelistedWiFi.begin(),
            detectedNonWhitelistedWiFi.end(),
            [&](const DetectedDevice &d){
                return !isInVector(currentDetectedWiFi, d.macAddress);
            }
        ),
        detectedNonWhitelistedWiFi.end()
    );

    detectedNonWhitelistedBLE.erase(
        std::remove_if(
            detectedNonWhitelistedBLE.begin(),
            detectedNonWhitelistedBLE.end(),
            [&](const DetectedDevice &d){
                return !isInVector(currentDetectedBLE, d.macAddress);
            }
        ),
        detectedNonWhitelistedBLE.end()
    );

    return newDeviceDetected;
}

// ------------ Scan WiFi Networks ------------
void scanWiFiNetworks(std::vector<String> &results, uint32_t duration_ms){
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true);

    unsigned long scanStart = millis();
    unsigned long scanEnd = scanStart + duration_ms;

    while(millis() < scanEnd){
        int n = WiFi.scanNetworks(false, true, false, 500);
        if(n == -1){
            Serial.println("WiFi scan failed.");
            break;
        }
        Serial.printf("Found %d WiFi networks in this scan\n", n);
        for(int i = 0; i < n; i++){
            String bssid = WiFi.BSSIDstr(i);
            if(!isInVector(results, bssid)){
                results.push_back(bssid);
                Serial.println("WiFi BSSID: " + bssid);
            }
        }
        delay(500);
    }
    Serial.printf("Total unique WiFi networks found: %d\n", (int)results.size());
    WiFi.scanDelete();
}

// ------------ Scan BLE Devices (Blocking) ------------
void scanBLEDevices(std::vector<String> &results, uint32_t durationSec){
    NimBLEScan *pScan = NimBLEDevice::getScan();
    pScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pScan->setActiveScan(true);
    pScan->setInterval(45);
    pScan->setWindow(15);

    Serial.println("Starting BLE scan...");
    NimBLEScanResults scanResults = pScan->start(durationSec, false);
    int count = scanResults.getCount();
    Serial.printf("Found %d BLE devices\n", count);

    for(int i = 0; i < count; i++){
        NimBLEAdvertisedDevice dev = scanResults.getDevice(i);
        String addr = String(dev.getAddress().toString().c_str());
        if(!isInVector(results, addr)){
            results.push_back(addr);
            Serial.println("BLE Address: " + addr);
        }
    }
    pScan->clearResults();
}

// ------------ Check if a Value Exists in a Vector ------------
bool isInVector(const std::vector<String> &vec, const String &val){
    return (std::find(vec.begin(), vec.end(), val) != vec.end());
}
