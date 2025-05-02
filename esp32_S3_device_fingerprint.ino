/*
*******************************************************************************
fingerprint and detect ble/wifi devices and send detection over serial
RTOS‑based baseline scanning: Wi‑Fi on core 0, BLE on core 1
Updated for NimBLE‑Arduino v2.x
*******************************************************************************
*/

#include <Arduino.h>
#include <WiFi.h>
#include <NimBLEDevice.h>
#include <NimBLEScan.h>
#include <vector>
#include <algorithm>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ================================
// Pin Definitions
// ================================
#define SERIAL1_RX_PIN D5  // GPIO5
#define SERIAL1_TX_PIN D4  // GPIO4

// ------------ Baseline Data ------------
static std::vector<String> baselineWiFi;
static std::vector<String> baselineBLE;
static bool isBaselineScan = true;
static bool baselineSet      = false;

// ------------ Baseline Task Flags ------------
static volatile bool wifiBaselineDone = false;
static volatile bool bleBaselineDone  = false;

// ------------ Current Scan Data ------------
static std::vector<String> currentWiFi;
static std::vector<String> currentBLE;

// ------------ Non‑Whitelisted Devices Data ------------
struct DetectedDevice { String macAddress; };
static std::vector<DetectedDevice> detectedNonWhitelistedWiFi;
static std::vector<DetectedDevice> detectedNonWhitelistedBLE;

// ------------ BLE Scan Globals ------------
std::vector<String>* bleResultVector = nullptr;
NimBLEScan*          pBLEScan          = nullptr;

// ------------ Forward Declarations ------------
void scanWiFiNetworks(std::vector<String> &results, uint32_t duration_ms);
void chunkedBLEScan   (std::vector<String> &results, uint32_t total_ms);
bool isInVector       (const std::vector<String> &vec, const String &val);

// ------------ FreeRTOS Baseline Tasks ------------
void wifiBaselineTask(void* pv) {
  std::vector<String> results;
  unsigned long start = millis();
  while (millis() - start < 420000UL) { // 7 minutes
    scanWiFiNetworks(results, 1000);
    delay(10); yield();
  }
  baselineWiFi     = results;
  wifiBaselineDone = true;
  vTaskDelete(NULL);
}

void bleBaselineTask(void* pv) {
  std::vector<String> results;
  unsigned long start = millis();
  while (millis() - start < 420000UL) { // 7 minutes
    chunkedBLEScan(results, 1000);
    delay(10); yield();
  }
  baselineBLE    = results;
  bleBaselineDone = true;
  vTaskDelete(NULL);
}

// ------------ BLE Scan Callback ------------
class MyScanCallbacks : public NimBLEScanCallbacks {
public:
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    std::string mac = dev->getAddress().toString();
    std::transform(mac.begin(), mac.end(), mac.begin(), ::toupper);
    String addr(mac.c_str());
    if (bleResultVector && !isInVector(*bleResultVector, addr)) {
      bleResultVector->push_back(addr);
      if (isBaselineScan) {
        Serial.println("Baseline BLE device found: " + addr);
      } else if (!isInVector(baselineBLE, addr)) {
        Serial.println("Detected non‑whitelisted BLE device: " + addr);
        Serial.println("New device alert: BLE " + addr);
        Serial1.println("New device alert: BLE " + addr);
      }
    }
  }
};
static MyScanCallbacks bleCallbacks;

// ------------ Setup Function ------------
void setup() {
  delay(2000); // Boot stabilization

  // USB Serial (native CDC on ESP32‑S3)
  Serial.begin(115200);
  Serial.println("USB Serial started.");

  // UART1 (Serial1) on D5/D4
  Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);
  Serial.println("Serial1 started.");

  // UART2 (Serial2) disabled to prevent USB disconnect issues
  // If needed, re-enable with explicit pins:
  // Serial2.begin(115200, SERIAL_8N1, SERIAL2_RX_PIN, SERIAL2_TX_PIN);
  // Serial.println("Serial2 started.");

  // Initialize BLE
  NimBLEDevice::init("");
  pBLEScan = NimBLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(60);
  pBLEScan->setWindow(30);
  pBLEScan->setScanCallbacks(&bleCallbacks, /*duplicates*/ false);
  Serial.println("NimBLE initialized.");

  // Launch baseline scans on separate cores
  isBaselineScan   = true;
  baselineSet      = false;
  wifiBaselineDone = false;
  bleBaselineDone  = false;
  xTaskCreatePinnedToCore(wifiBaselineTask, "wifiBase", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(bleBaselineTask,  "bleBase",  8192, NULL, 1, NULL, 1);
  Serial.println("Baseline tasks launched.");
}

// ------------ Loop Function ------------
void loop() {
  // Wait until both baseline tasks finish
  if (!baselineSet) {
    if (wifiBaselineDone && bleBaselineDone) {
      isBaselineScan = false;
      baselineSet    = true;
      Serial.println("Baseline complete.");
    }
    delay(100);
    yield();
    return;
  }

  // 5‑second BLE scan
  currentBLE.clear();
  Serial.println("Starting BLE scan for 5 seconds...");
  chunkedBLEScan(currentBLE, 5000);
  Serial.println("BLE scan completed.");

  // 5‑second WiFi scan
  currentWiFi.clear();
  Serial.println("Starting WiFi scan for 5 seconds...");
  scanWiFiNetworks(currentWiFi, 5000);
  Serial.println("WiFi scan completed.");

  delay(3000); // Pause before next cycle
}

// ------------ Async Wi‑Fi Scan ------------
void scanWiFiNetworks(std::vector<String> &results, uint32_t duration_ms) {
  WiFi.mode(WIFI_MODE_STA);
  WiFi.disconnect(true);

  // Start async scan
  WiFi.scanNetworks(/*async=*/true, /*hidden=*/true, /*probe=*/false, duration_ms);
  unsigned long deadline = millis() + duration_ms;
  int n = -1;
  while ((n = WiFi.scanComplete()) < 0 && millis() < deadline) {
    delay(50);
    yield();
  }
  if (n <= 0) {
    WiFi.scanDelete();
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
        Serial.println("New device alert: WiFi " + bssid);
        Serial1.println("New device alert: WiFi " + bssid);
      }
    }
  }
  WiFi.scanDelete();
}

// ------------ Chunked BLE Scan ------------
void chunkedBLEScan(std::vector<String> &results, uint32_t total_ms) {
  uint32_t windows = (total_ms + 999) / 1000;
  bleResultVector = &results;
  for (uint32_t w = 0; w < windows; ++w) {
    pBLEScan->start(1000, /*isContinue=*/false, /*restart=*/true);
    pBLEScan->clearResults();
    delay(10);
    yield();
  }
}

// ------------ Helper: Vector Search ------------
bool isInVector(const std::vector<String> &vec, const String &val) {
  return std::find(vec.begin(), vec.end(), val) != vec.end();
}
