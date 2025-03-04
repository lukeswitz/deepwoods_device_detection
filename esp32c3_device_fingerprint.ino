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
#define SERIAL1_RX_PIN D5  // GPIO5
#define SERIAL1_TX_PIN D4  // GPIO4

HardwareSerial Serial2(2);

// ------------ Baseline Data ------------
static std::vector<String> baselineWiFi;
static std::vector<String> baselineBLE;
bool isBaselineScan = true;

// ------------ Current Scan Data ------------
static std::vector<String> currentWiFi;
static std::vector<String> currentBLE;

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

// ------------ Global pointer for BLE scan results ------------
std::vector<String>* bleResultVector = nullptr;

// ------------ Bluetooth Scanning Callbacks ------------
class MyAdvertisedDeviceCallbacks : public NimBLEScanCallbacks {
public:
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
    // Get MAC address of the detected device and convert it to uppercase
    std::string macAddress = advertisedDevice->getAddress().toString();
    std::transform(macAddress.begin(), macAddress.end(), macAddress.begin(), ::toupper);
    String addr = String(macAddress.c_str());
    
    // Update the vector pointed to by bleResultVector
    if (bleResultVector && !isInVector(*bleResultVector, addr)) {
      bleResultVector->push_back(addr);
      
      if (isBaselineScan) {
        Serial.println("Baseline BLE device found: " + addr);
      } else {
        // If device not in baseline, report on both USB Serial and UART
        if (!isInVector(baselineBLE, addr)) {
          Serial.println("Detected non-whitelisted BLE device: " + addr);
          Serial1.println("New device alert: BLE " + addr);
        }
      }
    }
  }
};

// ------------ Setup Function ------------
void setup() {
  delay(2000); // Short delay for boot stability

  // Initialize USB Serial
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
  NimBLEDevice::getScan()->stop();

  isBaselineScan = true;

  // Baseline scan: 7 minutes of consecutive 20-second scans
  unsigned long scanStartTime = millis();
  while (millis() - scanStartTime < 420000) {  // 7 minutes
    scanWiFiNetworks(currentWiFi, 20000);     // 20-second WiFi scan
    scanBLEDevices(currentBLE, 20);            // 20-second BLE scan
    delay(1500);                              // Small delay between scans
  }

  // Set baseline using the collected current values
  baselineWiFi = currentWiFi;
  baselineBLE = currentBLE;
  baselineSet = true;
  isBaselineScan = false;

  Serial.println("Baseline complete.");
}

// ------------ Loop Function ------------
void loop() {
  if (baselineSet) {
    // --------- 5-Second BLE Scan ---------
    currentBLE.clear();
    Serial.println("Starting BLE scan for 5 seconds...");
    scanBLEDevices(currentBLE, 5); // 5-second BLE scan
    Serial.println("BLE scan completed.");

    // --------- 5-Second WiFi Scan ---------
    currentWiFi.clear();
    Serial.println("Starting WiFi scan for 5 seconds...");
    scanWiFiNetworks(currentWiFi, 5000); // 5-second WiFi scan
    Serial.println("WiFi scan completed.");
  }
  delay(3000);  // Delay before next cycle
}

// ------------ Scan WiFi Networks ------------
void scanWiFiNetworks(std::vector<String> &results, uint32_t duration_ms) {
  WiFi.mode(WIFI_MODE_STA);
  WiFi.disconnect(true);

  int previousCount = results.size();
  int n = WiFi.scanNetworks(false, true, false, duration_ms);
  
  if (n == -1) {
    Serial.println("WiFi scan failed.");
    return;
  }
  
  for (int i = 0; i < n; i++) {
    String bssid = WiFi.BSSIDstr(i);
    if (!isInVector(results, bssid)) {
      results.push_back(bssid);
      if (isBaselineScan) {
        Serial.println("WiFi BSSID: " + bssid);
      } else {
        // After baseline, report on both USB Serial and UART if not in baseline
        if (!isInVector(baselineWiFi, bssid)) {
          Serial.println("Detected non-whitelisted WiFi device: " + bssid);
          Serial1.println("New device alert: WiFi " + bssid);
        }
      }
    }
  }
  
  // Print tally summary only during baseline scanning
  if (isBaselineScan) {
    int newDevices = results.size() - previousCount;
    Serial.printf("WiFi scan summary: %d new devices discovered, %d total unique WiFi networks\n",
                  newDevices, (int)results.size());
  }
  WiFi.scanDelete();
}

// ------------ Scan BLE Devices (Blocking) ------------
void scanBLEDevices(std::vector<String> &results, uint32_t durationSec) {
  // Point bleResultVector to the vector passed in for this scan
  bleResultVector = &results;
  
  int previousCount = results.size();
  NimBLEScan *pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(new MyAdvertisedDeviceCallbacks());
  pScan->setActiveScan(true);
  pScan->setInterval(60);
  pScan->setWindow(30);
  
  pScan->start(durationSec, false);
  
  // Print tally summary only during baseline scanning
  if (isBaselineScan) {
    int newDevices = results.size() - previousCount;
    Serial.printf("BLE scan summary: %d new devices discovered, %d total unique BLE devices\n",
                  newDevices, (int)results.size());
  }
  pScan->clearResults();
}

// ------------ Check if a Value Exists in a Vector ------------
bool isInVector(const std::vector<String> &vec, const String &val) {
  return (std::find(vec.begin(), vec.end(), val) != vec.end());
}
