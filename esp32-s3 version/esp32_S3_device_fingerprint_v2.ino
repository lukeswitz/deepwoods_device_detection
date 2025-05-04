/*
*******************************************************************************
fingerprint and detect BLE/Wi‑Fi devices concurrently on separate cores,
with a 7 minute baseline phase and continuous detection thereafter,
reporting only non‑baseline detections over USB, plus baseline progress
counts every 30 s during the baseline.
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
#include <freertos/queue.h>
#include <stdarg.h>

// ================================
// Pin Definitions
// ================================
#define SERIAL1_RX_PIN D5
#define SERIAL1_TX_PIN D4

// Signal strength cutoff
#define RSSI_THRESHOLD -70

// ------------ Timing & Flags ------------
static const uint32_t BASELINE_MS = 420000UL; // 7 minutes
static uint32_t baselineStart = 0;
static volatile bool isBaseline = true, wifiDone = false, bleDone = false;

// ------------ Baseline Progress Counts ------------
static volatile size_t currBLECount = 0, currWiFiCount = 0;

// ------------ Baseline Storage ------------
static std::vector<String> baselineWiFi, baselineBLE;

// ------------ Print Message Queue ------------
struct PrintMsg { char buf[128]; };
static QueueHandle_t printQ = nullptr;

static void enqueueFmt(const char *fmt, ...) {
  PrintMsg m;
  va_list ap; va_start(ap, fmt);
  vsnprintf(m.buf, sizeof(m.buf), fmt, ap);
  va_end(ap);
  xQueueSend(printQ, &m, portMAX_DELAY);
}

static void enqueueMsg(const char *s) {
  PrintMsg m;
  strncpy(m.buf, s, sizeof(m.buf) - 1);
  m.buf[sizeof(m.buf) - 1] = '\0';
  xQueueSend(printQ, &m, portMAX_DELAY);
}

// ------------ PrintTask (core 0) ------------
void PrintTask(void*) {
  PrintMsg m;
  while (true) {
    if (xQueueReceive(printQ, &m, portMAX_DELAY) == pdTRUE) {
      size_t len = strnlen(m.buf, sizeof(m.buf));
      Serial.write((const uint8_t*)m.buf, len);
      Serial.write((const uint8_t*)"\r\n", 2);
      Serial.flush();
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

// ------------ BLE Scan Globals & Callback ------------
static NimBLEScan* pBLEScan = nullptr;
std::vector<String>* bleVec = nullptr;

class MyScanCallbacks : public NimBLEScanCallbacks {
public:
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    int32_t rssi = dev->getRSSI();
    if (rssi < RSSI_THRESHOLD) {
      return;
    }
    
    std::string macStd = dev->getAddress().toString();
    std::transform(macStd.begin(), macStd.end(), macStd.begin(), ::toupper);
    String addr(macStd.c_str());
    auto &v = *bleVec;
    if (std::find(v.begin(), v.end(), addr) == v.end()) {
      v.push_back(addr);
      currBLECount = v.size();
      if (!isBaseline && std::find(baselineBLE.begin(), baselineBLE.end(), addr) == baselineBLE.end()) {
        enqueueFmt("Detected non‑baseline BLE: %s (RSSI: %d dBm)", addr.c_str(), rssi);
        Serial1.println("New device alert: BLE " + addr + " " + String(rssi) + " dBm)");
      }
    }
  }
};


// ------------ BLETask (core 1) ------------
void BLETask(void*) {
  std::vector<String> seen;
  while (true) {
    bleVec = &seen;
    pBLEScan->start(2000, false, true);
    pBLEScan->clearResults();
    vTaskDelay(pdMS_TO_TICKS(10));

    if (isBaseline && !bleDone && millis() - baselineStart >= BASELINE_MS) {
      baselineBLE = seen;
      bleDone = true;
      enqueueFmt("BLE baseline done: %u devices", (unsigned)seen.size());
      if (wifiDone) {
        isBaseline = false;
        enqueueMsg("=== Baseline complete ===");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ------------ WiFiTask (core 1) ------------
void WiFiTask(void*) {
  std::vector<String> seen;
  while (true) {
    WiFi.scanNetworks(true, true, false, 110);
    unsigned long dl = millis() + 1470; // estimated scan time
    int n;
    while ((n = WiFi.scanComplete()) < 0 && millis() < dl) {
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (n > 0) {
      for (int i = 0; i < n; i++) {
        String b = WiFi.BSSIDstr(i);
        if (std::find(seen.begin(), seen.end(), b) == seen.end()) {
          seen.push_back(b);
          currWiFiCount = seen.size();
          if (!isBaseline && std::find(baselineWiFi.begin(), baselineWiFi.end(), b) == baselineWiFi.end()) {
            enqueueFmt("Detected non‑baseline WiFi: %s", b.c_str());
            Serial1.println("New device alert: WiFi " + b);
          }
        }
      }
    }
    WiFi.scanDelete();

    if (isBaseline && !wifiDone && millis() - baselineStart >= BASELINE_MS) {
      baselineWiFi = seen;
      wifiDone = true;
      enqueueFmt("WiFi baseline done: %u BSSIDs", (unsigned)seen.size());
      if (bleDone) {
        isBaseline = false;
        enqueueMsg("=== Baseline complete ===");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ------------ StatusTask (core 1) ------------
void StatusTask(void*) {
  while (isBaseline) {
    enqueueFmt("Baseline progress: %u BLE, %u WiFi", (unsigned)currBLECount, (unsigned)currWiFiCount);
    vTaskDelay(pdMS_TO_TICKS(30000));  // every 30 s
  }
  vTaskDelete(NULL);
}

// ------------ setup() ------------
void setup() {
  delay(5000);  // 5 second boot delay
  baselineStart = millis();

  Serial.begin(115200);
  while (!Serial) vTaskDelay(pdMS_TO_TICKS(10));
  printQ = xQueueCreate(20, sizeof(PrintMsg));
  xTaskCreatePinnedToCore(PrintTask, "PrintTask", 4096, NULL, 2, NULL, 0);
  enqueueMsg("USB Serial started.");

  Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);
  enqueueMsg("Serial1 started.");

  NimBLEDevice::init("");
  pBLEScan = NimBLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(60);
  pBLEScan->setWindow(30);
  static MyScanCallbacks cb;
  pBLEScan->setScanCallbacks(&cb, false);
  enqueueMsg("NimBLE initialized.");

  xTaskCreatePinnedToCore(BLETask,    "BLETask",    8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(WiFiTask,   "WiFiTask",   8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(StatusTask, "StatusTask", 4096, NULL, 1, NULL, 1);

  enqueueMsg("Scanning tasks launched (7 min baseline)...");
}

// ------------ loop() idle ------------
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}