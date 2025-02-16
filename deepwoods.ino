#include <WiFi.h>
#include <NimBLEDevice.h>
#include <vector>
#include <algorithm>

// ---------------- Serial Notification Setup ----------------
// On the ESP32-C3 Xiao board, TX is on D4 and RX is on D5.
const int NOTIF_TX_PIN = 4;  // TX pin for notifications
const int NOTIF_RX_PIN = 5;  // RX pin for notifications

// Use UART1 for notifications
HardwareSerial Serial2(1);

// ---------------- Baseline and Scan Data Structures ----------------
static std::vector<String> baselineWiFi;
static std::vector<String> baselineBLE;

static std::vector<String> currentWiFi;
static std::vector<String> currentBLE;

static std::vector<String> previousDetectedNonWhitelistedWiFi;
static std::vector<String> previousDetectedNonWhitelistedBLE;

struct DetectedDevice {
  String macAddress;
};
static std::vector<DetectedDevice> detectedNonWhitelistedWiFi;
static std::vector<DetectedDevice> detectedNonWhitelistedBLE;

bool baselineSet = false;

// ---------------- Function Declarations ----------------
void captureBaseline();
void scanWiFiNetworks(std::vector<String> &results, uint32_t duration_ms);
void scanBLEDevices(std::vector<String> &results, uint32_t durationSec);
bool updateDetectedDevices();
bool isInVector(const std::vector<String> &vec, const String &val);

// ---------------- BLE Callback ----------------
class MyAdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks {
public:
  void onResult(NimBLEAdvertisedDevice* advertisedDevice) override {
    // Get MAC address in uppercase
    std::string macAddress = advertisedDevice->getAddress().toString();
    std::transform(macAddress.begin(), macAddress.end(), macAddress.begin(), ::toupper);
    String addr = String(macAddress.c_str());
    // (Optional) Uncomment the next line for verbose BLE device output
    // Serial.println("BLE Device found: " + addr);
  }
};

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println("ESP32-C3 Scanner Starting...");

  // Initialize Serial2 for notifications on TX (D4) / RX (D5)
  Serial2.begin(115200, SERIAL_8N1, NOTIF_RX_PIN, NOTIF_TX_PIN);
  Serial.println("Notification Serial (Serial2) initialized on TX D4, RX D5.");

  // Initialize WiFi (station mode)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Initialize NimBLE
  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9); // Set BLE TX power as desired

  // Capture baseline fingerprint (7 minutes total scanning)
  captureBaseline();
}

void loop() {
  if (baselineSet) {
    // Clear current scan results
    currentWiFi.clear();
    currentBLE.clear();

    // ----- Short WiFi Scan (e.g., 2 seconds) -----
    Serial.println("Starting WiFi scan...");
    scanWiFiNetworks(currentWiFi, 2000);
    Serial.println("WiFi scan completed.");

    // ----- Short BLE Scan (e.g., 2 seconds) -----
    Serial.println("Starting BLE scan...");
    scanBLEDevices(currentBLE, 2);
    Serial.println("BLE scan completed.");

    // Check for new non-baseline devices and notify if any are found
    bool newDeviceDetected = updateDetectedDevices();
    if (newDeviceDetected) {
      Serial.println("New non-whitelisted device detected!");
      Serial2.println("Alert: New non-whitelisted device detected!");
    }

    delay(3000); // Wait before next scan cycle
  }
}

// ---------------- Capture Baseline Fingerprint ----------------
void captureBaseline() {
  Serial.println("Capturing baseline fingerprint for 7 minutes...");

  // Clear previous baseline data
  baselineWiFi.clear();
  baselineBLE.clear();

  // 1) BLE scan for 2 minutes (120 seconds)
  Serial.println("Baseline: Starting BLE scan for 2 minutes...");
  scanBLEDevices(baselineBLE, 120);
  Serial.println("Baseline: BLE scan (2 min) completed.");

  // 2) WiFi scan for 2 minutes (120,000 ms)
  Serial.println("Baseline: Starting WiFi scan for 2 minutes...");
  scanWiFiNetworks(baselineWiFi, 120000);
  Serial.println("Baseline: WiFi scan (2 min) completed.");

  // 3) BLE scan for 1 minute (60 seconds)
  Serial.println("Baseline: Starting BLE scan for 1 minute...");
  scanBLEDevices(baselineBLE, 60);
  Serial.println("Baseline: BLE scan (1 min) completed.");

  // 4) WiFi scan for 1 minute (60,000 ms)
  Serial.println("Baseline: Starting WiFi scan for 1 minute...");
  scanWiFiNetworks(baselineWiFi, 60000);
  Serial.println("Baseline: WiFi scan (1 min) completed.");

  // 5) BLE scan for 30 seconds
  Serial.println("Baseline: Starting BLE scan for 30 seconds...");
  scanBLEDevices(baselineBLE, 30);
  Serial.println("Baseline: BLE scan (30 sec) completed.");

  // 6) WiFi scan for 30 seconds (30,000 ms)
  Serial.println("Baseline: Starting WiFi scan for 30 seconds...");
  scanWiFiNetworks(baselineWiFi, 30000);
  Serial.println("Baseline: WiFi scan (30 sec) completed.");

  baselineSet = true;
  Serial.println("Baseline captured! Monitoring for new devices...");
}

// ---------------- WiFi Scanning Function ----------------
void scanWiFiNetworks(std::vector<String> &results, uint32_t duration_ms) {
  unsigned long scanStart = millis();
  unsigned long scanEnd = scanStart + duration_ms;
  
  while (millis() < scanEnd) {
    int n = WiFi.scanNetworks(false, true, false, 500);
    if (n == -1) {
      Serial.println("WiFi scan failed.");
      break;
    }
    Serial.printf("Found %d WiFi networks in this scan\n", n);
    for (int i = 0; i < n; i++) {
      String bssid = WiFi.BSSIDstr(i);
      if (!isInVector(results, bssid)) {
        results.push_back(bssid);
        Serial.println("WiFi BSSID: " + bssid);
      }
    }
    delay(500);
  }
  Serial.printf("Total unique WiFi networks found: %d\n", (int)results.size());
  WiFi.scanDelete();
}

// ---------------- BLE Scanning Function ----------------
void scanBLEDevices(std::vector<String> &results, uint32_t durationSec) {
  NimBLEScan *pScan = NimBLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pScan->setActiveScan(true);
  pScan->setInterval(45);
  pScan->setWindow(15);

  Serial.println("Starting BLE scan...");
  NimBLEScanResults scanResults = pScan->start(durationSec, false);
  int count = scanResults.getCount();
  Serial.printf("Found %d BLE devices\n", count);

  for (int i = 0; i < count; i++) {
    NimBLEAdvertisedDevice dev = scanResults.getDevice(i);
    String addr = String(dev.getAddress().toString().c_str());
    if (!isInVector(results, addr)) {
      results.push_back(addr);
      Serial.println("BLE Address: " + addr);
    }
  }
  pScan->clearResults();
}

// ---------------- Update Detected Devices ----------------
bool updateDetectedDevices() {
  bool newDeviceDetected = false;
  std::vector<String> currentDetectedWiFi;
  std::vector<String> currentDetectedBLE;

  // For WiFi: only include devices that were not in the baseline
  for (auto &dev : currentWiFi) {
    if (!isInVector(baselineWiFi, dev)) {
      currentDetectedWiFi.push_back(dev);
    }
  }

  // For BLE: only include devices that were not in the baseline
  for (auto &dev : currentBLE) {
    if (!isInVector(baselineBLE, dev)) {
      currentDetectedBLE.push_back(dev);
    }
  }

  // Identify new WiFi devices (not seen in the previous scan)
  for (auto &dev : currentDetectedWiFi) {
    if (!isInVector(previousDetectedNonWhitelistedWiFi, dev)) {
      newDeviceDetected = true;
      Serial.println("New non-whitelisted WiFi device detected: " + dev);
      Serial2.println("New non-whitelisted WiFi device: " + dev);
      DetectedDevice newDev;
      newDev.macAddress = dev;
      detectedNonWhitelistedWiFi.push_back(newDev);
    }
  }

  // Identify new BLE devices (not seen in the previous scan)
  for (auto &dev : currentDetectedBLE) {
    if (!isInVector(previousDetectedNonWhitelistedBLE, dev)) {
      newDeviceDetected = true;
      Serial.println("New non-whitelisted BLE device detected: " + dev);
      Serial2.println("New non-whitelisted BLE device: " + dev);
      DetectedDevice newDev;
      newDev.macAddress = dev;
      detectedNonWhitelistedBLE.push_back(newDev);
    }
  }

  // Update the previous-detected lists for the next cycle
  previousDetectedNonWhitelistedWiFi = currentDetectedWiFi;
  previousDetectedNonWhitelistedBLE = currentDetectedBLE;

  // Remove devices that are no longer detected
  detectedNonWhitelistedWiFi.erase(
      std::remove_if(
          detectedNonWhitelistedWiFi.begin(),
          detectedNonWhitelistedWiFi.end(),
          [&](const DetectedDevice &d) {
            return !isInVector(currentDetectedWiFi, d.macAddress);
          }
      ),
      detectedNonWhitelistedWiFi.end()
  );

  detectedNonWhitelistedBLE.erase(
      std::remove_if(
          detectedNonWhitelistedBLE.begin(),
          detectedNonWhitelistedBLE.end(),
          [&](const DetectedDevice &d) {
            return !isInVector(currentDetectedBLE, d.macAddress);
          }
      ),
      detectedNonWhitelistedBLE.end()
  );

  return newDeviceDetected;
}

// ---------------- Check if a Value Exists in a Vector ----------------
bool isInVector(const std::vector<String> &vec, const String &val) {
  return (std::find(vec.begin(), vec.end(), val) != vec.end());
}
