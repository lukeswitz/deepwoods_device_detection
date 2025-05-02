/*
*******************************************************************************
fingerprint and detect ble/wifi devices and send detection over serial
*******************************************************************************
*/

#include <Arduino.h>
#include <WiFi.h>
#include <NimBLEDevice.h>
#include <NimBLEScan.h>
#include <vector>
#include <algorithm>

// ================================
// Pin Definitions
// ================================
#define SERIAL1_RX_PIN D5  // GPIO5
#define SERIAL1_TX_PIN D4  // GPIO4

// Use the built‑in Serial2 (UART2) instance provided by the ESP32 core

// ------------ Baseline Data ------------
static std::vector<String> baselineWiFi;
static std::vector<String> baselineBLE;
bool isBaselineScan = true;

// ------------ Current Scan Data ------------
static std::vector<String> currentWiFi;
static std::vector<String> currentBLE;

// ------------ Non‑Whitelisted Devices Data ------------
struct DetectedDevice { String macAddress; };
static std::vector<DetectedDevice> detectedNonWhitelistedWiFi;
static std::vector<DetectedDevice> detectedNonWhitelistedBLE;

// ------------ Flag: Baseline Scan Complete ------------
bool baselineSet = false;

// ------------ BLE Scan Globals ------------
std::vector<String>* bleResultVector = nullptr;
NimBLEScan*          pBLEScan          = nullptr;

// ------------ Forward Declarations ------------
void scanWiFiNetworks(std::vector<String> &results, uint32_t duration_ms = 0);
void scanBLEDevices   (std::vector<String> &results, uint32_t duration_ms);
bool isInVector       (const std::vector<String> &vec, const String &val);

// ------------ BLE Scan Callback ------------
class MyScanCallbacks : public NimBLEScanCallbacks {
public:
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
    std::string mac = advertisedDevice->getAddress().toString();
    std::transform(mac.begin(), mac.end(), mac.begin(), ::toupper);
    String addr(mac.c_str());

    if (bleResultVector && !isInVector(*bleResultVector, addr)) {
      bleResultVector->push_back(addr);

      if (isBaselineScan) {
        Serial.println("Baseline BLE device found: " + addr);
      } else if (!isInVector(baselineBLE, addr)) {
        Serial.println("Detected non‑whitelisted BLE device: " + addr);
        Serial1.println("New device alert: BLE " + addr);
      }
    }
  }
};
static MyScanCallbacks bleCallbacks;

// ------------ Setup Function ------------
void setup() {
  delay(2000); // Boot stabilization

  // USB Serial
  Serial.begin(115200);
  Serial.println("USB Serial started.");

  // UART1 (Serial1) on D5 (RX) / D4 (TX)
  Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);
  Serial.println("Serial1 started.");

  // UART2 (Serial2) — default pins, no custom instantiation needed
  Serial2.begin(115200);
  Serial.println("Serial2 started.");

  // Initialize NimBLE and configure scanner
  NimBLEDevice::init("");
  Serial.println("NimBLE initialized.");

  pBLEScan = NimBLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(60);
  pBLEScan->setWindow(30);
  pBLEScan->setScanCallbacks(&bleCallbacks, /*duplicates*/ false);

  // Perform 7 minutes of back-to-back 20‑second baseline scans
  isBaselineScan = true;
  unsigned long start = millis();
  while (millis() - start < 420000UL) { // 7×60×1000 ms
    currentWiFi.clear();
    currentBLE.clear();

    scanWiFiNetworks(currentWiFi, 20000); // 20 000 ms
    scanBLEDevices   (currentBLE,  20000); // 20 000 ms

    delay(1500);
    yield();
  }

  // Lock in baseline lists
  baselineWiFi = currentWiFi;
  baselineBLE  = currentBLE;
  baselineSet  = true;
  isBaselineScan = false;

  Serial.println("Baseline complete.");
}

// ------------ Loop Function ------------
void loop() {
  if (baselineSet) {
    // 5-second BLE scan
    currentBLE.clear();
    Serial.println("Starting BLE scan for 5 seconds...");
    scanBLEDevices(currentBLE, 5000); // 5 000 ms
    Serial.println("BLE scan completed.");

    // 5-second WiFi scan
    currentWiFi.clear();
    Serial.println("Starting WiFi scan for 5 seconds...");
    scanWiFiNetworks(currentWiFi, 5000); // 5 000 ms
    Serial.println("WiFi scan completed.");
  }
  delay(3000); // Pause before next cycle
}

// ------------ Scan WiFi Networks ------------
void scanWiFiNetworks(std::vector<String> &results, uint32_t duration_ms) {
  WiFi.mode(WIFI_MODE_STA);
  WiFi.disconnect(true);

  int prevCount = results.size();
  int n = (duration_ms > 0)
          ? WiFi.scanNetworks(false, true, false, duration_ms)
          : WiFi.scanNetworks();

  if (n < 0) {
    Serial.println("WiFi scan failed.");
    return;
  }

  for (int i = 0; i < n; i++) {
    String bssid = WiFi.BSSIDstr(i);
    if (!isInVector(results, bssid)) {
      results.push_back(bssid);
      if (isBaselineScan) {
        Serial.println("WiFi BSSID: " + bssid);
      } else if (!isInVector(baselineWiFi, bssid)) {
        Serial.println("Detected non‑whitelisted WiFi device: " + bssid);
        Serial1.println("New device alert: WiFi " + bssid);
      }
    }
  }

  if (isBaselineScan) {
    int newDevices = results.size() - prevCount;
    Serial.printf("WiFi scan summary: %d new, %d total\n",
                  newDevices, (int)results.size());
  }
  WiFi.scanDelete();
}

// ------------ Scan BLE Devices ------------
void scanBLEDevices(std::vector<String> &results, uint32_t duration_ms) {
  bleResultVector = &results;
  pBLEScan->start(duration_ms, /*isContinue*/ false, /*restart*/ true);
  pBLEScan->clearResults();
}

// ------------ Helper: Vector Search  ------------
bool isInVector(const std::vector<String> &vec, const String &val) {
  return std::find(vec.begin(), vec.end(), val) != vec.end();
}
