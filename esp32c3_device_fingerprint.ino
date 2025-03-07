/*
*******************************************************************************
fingerprint and detect ble/wifi devices and send detection over serial
* 
*******************************************************************************
*/

#include <WiFi.h>
#include <NimBLEDevice.h>
#include <unordered_set>
#include <string>

// Hash function for String class
struct StringHash {
  std::size_t operator()(const String& s) const {
    return std::hash<std::string>()(s.c_str());
  }
};

// String equality comparison
struct StringEqual {
  bool operator()(const String& a, const String& b) const {
    return a.equals(b);
  }
};

// ================================
// Pin Definitions
// ================================
#define SERIAL1_RX_PIN D5  // GPIO5
#define SERIAL1_TX_PIN D4  // GPIO4

HardwareSerial Serial2(2);

#define MAX_WHITELIST_SIZE 1500
#define MAX_ALERTED_SIZE 500

// Rate-limiting
unsigned long lastSerialSendTime = 0;
const unsigned long MIN_SEND_INTERVAL = 1000; // milliseconds between sends

// Device Database
typedef std::unordered_set<String, StringHash, StringEqual> DeviceSet;

// ------------ Device Sets ------------
DeviceSet whitelistedWiFi;  // Baseline WiFi devices
DeviceSet whitelistedBLE;   // Baseline BLE devices
DeviceSet alertedWiFi;      // WiFi devices we've already alerted about
DeviceSet alertedBLE;       // BLE devices we've already alerted about

// ------------ Scan State ------------
bool baselineMode = true;

// ------------ Rate-limited messages ------------
void sendAlertWithRateLimit(String message) {
  unsigned long currentTime = millis();
  if (currentTime - lastSerialSendTime >= MIN_SEND_INTERVAL) {
    Serial1.println(message);
    lastSerialSendTime = currentTime;
  }
}

// ------------ BLE Scanning Callback ------------
class BLEScanner : public NimBLEScanCallbacks {
private:
  static BLEScanner* instance;
  DeviceSet* targetSet;
  int newDevicesFound;

  BLEScanner()
    : targetSet(nullptr), newDevicesFound(0) {}

public:
  static BLEScanner* getInstance() {
    if (!instance) {
      instance = new BLEScanner();
    }
    return instance;
  }

  void setTargetSet(DeviceSet* set) {
    targetSet = set;
    newDevicesFound = 0;
  }

  int getNewDevicesFound() {
    return newDevicesFound;
  }

  void onResult(const NimBLEAdvertisedDevice* device) override {
    if (!targetSet) return;

    // Get standardized MAC address
    std::string macStr = device->getAddress().toString();
    std::transform(macStr.begin(), macStr.end(), macStr.begin(), ::toupper);
    String mac = String(macStr.c_str());

    // Add to result set
    bool isNewDevice = targetSet->insert(mac).second;

    if (isNewDevice) {
      newDevicesFound++;

      if (baselineMode) {
        Serial.println("Found baseline BLE device: " + mac);
      } else {
        // Check if this is a non-whitelisted device we haven't alerted about
        if (whitelistedBLE.find(mac) == whitelistedBLE.end() && alertedBLE.find(mac) == alertedBLE.end()) {
          // New non-whitelisted device found!
          Serial.println("Detected non-whitelisted BLE device: " + mac);
          sendAlertWithRateLimit("New device alert: BLE " + mac);
          alertedBLE.insert(mac);
        }
      }
    }
  }
};

BLEScanner* BLEScanner::instance = nullptr;

// ------------ Function Declarations ------------
void scanWiFiDevices(DeviceSet& resultSet, uint32_t durationMs);
void scanBLEDevices(DeviceSet& resultSet, uint32_t durationSec);

// ------------ Setup Function ------------
void setup() {
  delay(2000);  // Boot stability delay

  // Initialize communication channels
  Serial.begin(115200);
  Serial.println("USB Serial started.");

  Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);
  Serial.println("Serial1 started.");

  Serial2.begin(115200);
  Serial.println("Serial2 started.");

  // Initialize Bluetooth
  NimBLEDevice::init("");
  Serial.println("NimBLE initialized.");

  // Start in baseline mode
  baselineMode = true;
  Serial.println("Starting baseline collection (10 minutes)...");

  // Run baseline scan for 10 minutes
  DeviceSet tempWiFi, tempBLE;
  unsigned long startTime = millis();
  unsigned long endTime = startTime + (10 * 60 * 1000);  // 10 minutes

  while (millis() < endTime) {
    // Alternate between WiFi and BLE scans
    scanWiFiDevices(tempWiFi, 20000);  // 20-second WiFi scan
    scanBLEDevices(tempBLE, 30);       // 20-second BLE scan
    delay(1000);                       // Short delay between scan cycles
  }

  // Store baseline results
  whitelistedWiFi = std::move(tempWiFi);
  whitelistedBLE = std::move(tempBLE);

  // Ensure baseline sets don't exceed max size
  trimSetToMaxSize(whitelistedWiFi, MAX_WHITELIST_SIZE);
  trimSetToMaxSize(whitelistedBLE, MAX_WHITELIST_SIZE);

  // Exit baseline mode
  baselineMode = false;
  Serial.printf("Baseline complete: %d WiFi and %d BLE devices whitelisted\n",
                whitelistedWiFi.size(), whitelistedBLE.size());
}

// ------------ Loop Function ------------
void loop() {
  // Create temporary sets for this scan cycle
  DeviceSet currentWiFi, currentBLE;

  // Run WiFi scan (5 seconds)
  Serial.println("Starting WiFi detection scan...");
  scanWiFiDevices(currentWiFi, 5000);

  // Run BLE scan (5 seconds)
  Serial.println("Starting BLE detection scan...");
  scanBLEDevices(currentBLE, 5);

  // Report scan completion
  Serial.printf("Detection scan complete: found %d WiFi and %d BLE devices\n",
                currentWiFi.size(), currentBLE.size());

  // Wait before next scan cycle
  delay(3000);
}

// ------------ Scan WiFi Networks ------------
void scanWiFiDevices(DeviceSet& resultSet, uint32_t durationMs) {
  WiFi.mode(WIFI_MODE_STA);
  WiFi.disconnect(true);

  // Capture size before scan
  size_t initialSize = resultSet.size();

  // Start scan
  int networkCount = WiFi.scanNetworks(false, true, false, durationMs);

  if (networkCount == -1) {
    Serial.println("WiFi scan failed.");
    return;
  }

  // Process scan results
  for (int i = 0; i < networkCount; i++) {
    String mac = WiFi.BSSIDstr(i);

    // Add to result set
    bool isNewDevice = resultSet.insert(mac).second;

    if (isNewDevice) {
      if (baselineMode) {
        Serial.println("Found baseline WiFi device: " + mac);
      } else {
        // Check if this is a non-whitelisted device we haven't alerted about
        if (whitelistedWiFi.find(mac) == whitelistedWiFi.end() && alertedWiFi.find(mac) == alertedWiFi.end()) {
          // New non-whitelisted device found!
          Serial.println("Detected non-whitelisted WiFi device: " + mac);
          sendAlertWithRateLimit("New device alert: WiFi " + mac);
          alertedWiFi.insert(mac);
        }
      }
    }
  }

  // Log summary
  size_t newDevices = resultSet.size() - initialSize;
  if (baselineMode || newDevices > 0) {
    Serial.printf("WiFi scan summary: %d new devices, %d total unique devices\n",
                  newDevices, resultSet.size());
  }

  // Clean up
  WiFi.scanDelete();
}

// ------------ Scan BLE Devices ------------
void scanBLEDevices(DeviceSet& resultSet, uint32_t durationSec) {
  // Capture size before scan
  size_t initialSize = resultSet.size();

  // Setup BLE scan
  NimBLEScan* scanner = NimBLEDevice::getScan();
  BLEScanner* callbacks = BLEScanner::getInstance();
  callbacks->setTargetSet(&resultSet);

  scanner->setScanCallbacks(callbacks);
  scanner->setActiveScan(true);  // Active scanning for better device detection
  scanner->setInterval(100);     // Scan interval in ms
  scanner->setWindow(99);        // Scan window in ms

  // Run blocking scan
  scanner->start(durationSec, true);

  // Let any pending callbacks complete
  delay(100);

  // Log summary
  int newDevices = callbacks->getNewDevicesFound();
  if (baselineMode || newDevices > 0) {
    Serial.printf("BLE scan summary: %d new devices, %d total unique devices\n",
                  newDevices, resultSet.size());
  }

  // Clean up
  scanner->clearResults();
}

// ------------ Memory Management Function ------------
// Function to trim a set down to max size if needed
void trimSetToMaxSize(DeviceSet& set, size_t maxSize) {
  if (set.size() <= maxSize) return;

  // Calculate how many to remove
  size_t toRemove = set.size() - maxSize;

  // Simple approach: just remove oldest entries
  // (In this implementation, we just remove some random entries since
  // unordered_set doesn't have a concept of oldest/newest)
  auto it = set.begin();
  for (size_t i = 0; i < toRemove && it != set.end(); i++) {
    it = set.erase(it);
  }

  Serial.printf("Memory management: Trimmed %d devices from set\n", toRemove);
}