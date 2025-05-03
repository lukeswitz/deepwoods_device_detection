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
#include <cstring>  // memcpy, strnlen, strncmp

// ================================
// Pin Definitions
// ================================
#define SERIAL1_RX_PIN D5
#define SERIAL1_TX_PIN D4

// ------------ Timing & Flags ------------
static const uint32_t BASELINE_MS = 300000UL; // 5 minutes
static uint32_t baselineStart = 0;
static volatile bool isBaseline = true, wifiDone = false, bleDone = false;

// ------------ Progress Counts & Baseline Storage ------------
static volatile size_t currBLECount = 0, currWiFiCount = 0;
static std::vector<String> baselineBLE, baselineWiFi;

// ------------ Print Queue & Enqueue Function ------------
struct PrintMsg { char buf[128]; };
static QueueHandle_t printQ = nullptr;
static const char *DETECT_PREFIX = "Detected non‑baseline";

static void enqueueFmt(const char *fmt, ...) {
  PrintMsg m;
  va_list ap; 
  va_start(ap, fmt);
  vsnprintf(m.buf, sizeof(m.buf), fmt, ap);
  va_end(ap);
  xQueueSend(printQ, &m, portMAX_DELAY);
}

// ------------ PrintTask on core 0 ------------
void PrintTask(void*) {
  PrintMsg m;
  char outbuf[130];         // message + CRLF
  static char usbBuf[192];  // up to 3×64B packets
  const size_t EP = 64;

  while (true) {
    if (xQueueReceive(printQ, &m, portMAX_DELAY) == pdTRUE) {
      size_t len = strnlen(m.buf, sizeof(m.buf));
      memcpy(outbuf, m.buf, len);
      outbuf[len]   = '\r';
      outbuf[len+1] = '\n';
      size_t total = len + 2;

      // — USB CDC: pad with NULs to full 64B frames —
      size_t padded = ((total + EP - 1) / EP) * EP;
      if (padded > sizeof(usbBuf)) padded = sizeof(usbBuf);
      memcpy(usbBuf, outbuf, total);
      memset(usbBuf + total, 0, padded - total);
      Serial.write((const uint8_t*)usbBuf, padded);
      if (total % EP == 0) Serial.write((const uint8_t*)"", 0);
      Serial.flush();

      // — UART1: only non‑baseline alerts —
      if (strncmp(m.buf, DETECT_PREFIX, strlen(DETECT_PREFIX)) == 0) {
        Serial1.write((const uint8_t*)outbuf, total);
        Serial1.flush();
      }
    }
  }
}

// ------------ BLE Scan Callback & Task ------------
static NimBLEScan* pBLEScan = nullptr;
static std::vector<String>* bleVec = nullptr;

class MyScanCallbacks : public NimBLEScanCallbacks {
public:
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    std::string mac = dev->getAddress().toString();
    std::transform(mac.begin(), mac.end(), mac.begin(), ::toupper);
    String addr(mac.c_str());
    auto &v = *bleVec;
    if (std::find(v.begin(), v.end(), addr) == v.end()) {
      v.push_back(addr);
      currBLECount = v.size();
      if (!isBaseline &&
          std::find(baselineBLE.begin(), baselineBLE.end(), addr) == baselineBLE.end()) {
        enqueueFmt("Detected non‑baseline BLE: %s", addr.c_str());
      }
    }
  }
};

void BLETask(void*) {
  std::vector<String> seen;
  while (true) {
    bleVec = &seen;
    pBLEScan->start(1000, false, true);
    pBLEScan->clearResults();
    vTaskDelay(pdMS_TO_TICKS(10));

    if (isBaseline && !bleDone && millis() - baselineStart >= BASELINE_MS) {
      baselineBLE = seen;
      bleDone = true;
      if (wifiDone) {
        isBaseline = false;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ------------ WiFi Scan Task ------------
void WiFiTask(void*) {
  std::vector<String> seen;
  while (true) {
    WiFi.scanNetworks(true, true, false, 1000);
    unsigned long deadline = millis() + 1000;
    int n;
    while ((n = WiFi.scanComplete()) < 0 && millis() < deadline) {
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (n > 0) {
      for (int i = 0; i < n; i++) {
        String bssid = WiFi.BSSIDstr(i);
        if (std::find(seen.begin(), seen.end(), bssid) == seen.end()) {
          seen.push_back(bssid);
          currWiFiCount = seen.size();
          if (!isBaseline &&
              std::find(baselineWiFi.begin(), baselineWiFi.end(), bssid) == baselineWiFi.end()) {
            enqueueFmt("Detected non‑baseline WiFi: %s", bssid.c_str());
          }
        }
      }
    }
    WiFi.scanDelete();

    if (isBaseline && !wifiDone && millis() - baselineStart >= BASELINE_MS) {
      baselineWiFi = seen;
      wifiDone = true;
      if (bleDone) {
        isBaseline = false;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ------------ Baseline Progress Task ------------
void StatusTask(void*) {
  size_t lastMinuteReported = 0;
  while (isBaseline) {
    enqueueFmt("Baseline progress: %u BLE, %u WiFi",
               (unsigned)currBLECount, (unsigned)currWiFiCount);

    uint32_t elapsed = millis() - baselineStart;
    size_t minutesElapsed = elapsed / 60000;
    if (minutesElapsed > lastMinuteReported) {
      size_t remMin = (BASELINE_MS - elapsed + 59999) / 60000;
      if (remMin > 0) {
        enqueueFmt("Baseline Scan has %u minutes remaining.", (unsigned)remMin);
      }
      lastMinuteReported = minutesElapsed;
    }

    vTaskDelay(pdMS_TO_TICKS(15000));  // every 15 s
  }

  // combined final summary:
  enqueueFmt("Baseline complete: %u BLE, %u WiFi",
             (unsigned)currBLECount, (unsigned)currWiFiCount);
  enqueueFmt("=== Baseline complete ===");
  vTaskDelete(NULL);
}

// ------------ setup() & loop() ------------
void setup() {
  delay(5000);
  baselineStart = millis();

  Serial.begin(115200);
  while (!Serial) vTaskDelay(pdMS_TO_TICKS(10));
  Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);

  printQ = xQueueCreate(20, sizeof(PrintMsg));
  xTaskCreatePinnedToCore(PrintTask, "PrintTask", 4096, NULL, 2, NULL, 0);

  enqueueFmt("Deepwoods Device Detection");
  enqueueFmt("5 minute Baseline Scan Started");

  NimBLEDevice::init("");
  pBLEScan = NimBLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(60);
  pBLEScan->setWindow(30);
  static MyScanCallbacks bleCb;
  pBLEScan->setScanCallbacks(&bleCb, false);

  xTaskCreatePinnedToCore(BLETask,    "BLETask",    8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(WiFiTask,   "WiFiTask",   8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(StatusTask, "StatusTask", 4096, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
