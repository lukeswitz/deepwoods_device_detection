#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <NimBLEDevice.h>
#include <NimBLEScan.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <stdarg.h>
#include <cstring>            // memcpy, strcmp, strcpy, snprintf
#include <soc/rtc_cntl_reg.h> // RTC_CNTL_BROWN_OUT_REG
#include <soc/soc.h>          // REG_CLR_BIT, RTC_CNTL_BROWN_OUT_ENA

// ================================
// Pin Definitions
// ================================
#define SERIAL1_RX_PIN D5
#define SERIAL1_TX_PIN D4

// ------------ Timing & Flags ------------
static const uint32_t BASELINE_MS = 300000UL; // 5 minutes
static uint32_t baselineStart = 0;
static volatile bool isBaseline = true, bleDone = false, wifiDone = false;

// ------------ Counts ------------
static volatile size_t currBLECount   = 0,
                      currWiFiCount  = 0,
                      currProbeCount = 0;

// ------------ Capacity Constants ------------
#define MAX_BLE   200
#define MAX_WIFI  200
#define MAX_PROBE 500

// ------------ Static Storage Arrays ------------
static char bleSeenArr[MAX_BLE][18];
static int  bleSeenCount = 0;

static char wifiSeenArr[MAX_WIFI][18];
static int  wifiSeenCount = 0;

static char probeSeenArr[MAX_PROBE][18];
static int  probeSeenCount = 0;

static char baselineBleArr[MAX_BLE][18];
static int  baselineBleCount = 0;

static char baselineWifiArr[MAX_WIFI][18];
static int  baselineWifiCount = 0;

static char baselineProbeArr[MAX_PROBE][18];
static int  baselineProbeCount = 0;

// ------------ Print Queue & Function ------------
struct PrintMsg { char buf[128]; };
static QueueHandle_t printQ = nullptr;
static const char *DETECT_PREFIX = "Detected non-baseline";

static void enqueueFmt(const char *fmt, ...) {
  PrintMsg m;
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(m.buf, sizeof(m.buf), fmt, ap);
  va_end(ap);
  xQueueSend(printQ, &m, portMAX_DELAY);
}

// ------------ Probe Event & Queue ------------
struct ProbeEvent { char mac[18]; };
static QueueHandle_t probeQ = nullptr;

// ------------ Sniffer Callback (ISR) ------------
static void IRAM_ATTR snifferCallback(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;
  auto* p = (wifi_promiscuous_pkt_t*)buf;
  const uint8_t* hdr = p->payload;
  uint8_t subtype = (hdr[0] >> 4) & 0xF;
  if (subtype != 4) return; // probe request only

  ProbeEvent ev;
  snprintf(ev.mac, sizeof(ev.mac),
           "%02X:%02X:%02X:%02X:%02X:%02X",
           hdr[10],hdr[11],hdr[12],
           hdr[13],hdr[14],hdr[15]);

  xQueueSendFromISR(probeQ, &ev, NULL);
}

// ------------ ProbeTask (core 1) ------------
void ProbeTask(void*) {
  ProbeEvent ev;
  while (xQueueReceive(probeQ, &ev, portMAX_DELAY)) {
    bool found = false;
    for (int i = 0; i < probeSeenCount; i++) {
      if (strcmp(probeSeenArr[i], ev.mac) == 0) { found = true; break; }
    }
    if (!found && probeSeenCount < MAX_PROBE) {
      strcpy(probeSeenArr[probeSeenCount], ev.mac);
      probeSeenCount++;
      currProbeCount = probeSeenCount;
      if (!isBaseline) {
        bool inBase = false;
        for (int j = 0; j < baselineProbeCount; j++) {
          if (strcmp(baselineProbeArr[j], ev.mac) == 0) { inBase = true; break; }
        }
        if (!inBase) {
          enqueueFmt("Detected non-baseline ProbeReq: %s", ev.mac);
        }
      }
    }
  }
}

// ------------ ChannelHopTask (core 1) ------------
void ChannelHopTask(void*) {
  uint8_t ch = 1;
  while (true) {
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
    ch = (ch % 13) + 1;
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ------------ PrintTask (core 0) ------------
void PrintTask(void*) {
  PrintMsg m;
  char outbuf[130];
  static char usbBuf[192];
  const size_t EP = 64;

  while (xQueueReceive(printQ, &m, portMAX_DELAY)) {
    size_t len = strnlen(m.buf, sizeof(m.buf));
    memcpy(outbuf, m.buf, len);
    outbuf[len]   = '\r';
    outbuf[len+1] = '\n';
    size_t total = len + 2;

    size_t padded = ((total + EP - 1) / EP) * EP;
    if (padded > sizeof(usbBuf)) padded = sizeof(usbBuf);
    memcpy(usbBuf, outbuf, total);
    memset(usbBuf + total, 0, padded - total);

    Serial.write((uint8_t*)usbBuf, padded);
    if (total % EP == 0) Serial.write((uint8_t*)"", 0);
    Serial.flush();

    if (strncmp(m.buf, DETECT_PREFIX, strlen(DETECT_PREFIX)) == 0) {
      Serial1.write((uint8_t*)outbuf, total);
      Serial1.flush();
    }
  }
}

// ------------ BLE Scan Callback & Task ------------
static NimBLEScan* pBLEScan = nullptr;
class MyScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    if (!dev) return;
    NimBLEAddress addr = dev->getAddress();
    addr.reverseByteOrder();
    const uint8_t* raw = addr.getVal();

    char mac[18];
    snprintf(mac, sizeof(mac),
             "%02X:%02X:%02X:%02X:%02X:%02X",
             raw[0], raw[1], raw[2],
             raw[3], raw[4], raw[5]);

    bool found = false;
    for (int i = 0; i < bleSeenCount; i++) {
      if (strcmp(bleSeenArr[i], mac) == 0) { found = true; break; }
    }
    if (!found && bleSeenCount < MAX_BLE) {
      strcpy(bleSeenArr[bleSeenCount], mac);
      bleSeenCount++;
      currBLECount = bleSeenCount;

      if (!isBaseline) {
        bool inBase = false;
        for (int j = 0; j < baselineBleCount; j++) {
          if (strcmp(baselineBleArr[j], mac) == 0) { inBase = true; break; }
        }
        if (!inBase) {
          enqueueFmt("Detected non-baseline BLE: %s", mac);
        }
      }
    }
  }
};

void BLETask(void*) {
  bleSeenCount = 0;
  while (true) {
    pBLEScan->start(1000, false, true);
    pBLEScan->clearResults();
    // small delay to prevent tight-loop timing issues
    vTaskDelay(pdMS_TO_TICKS(20));

    if (isBaseline && !bleDone && millis() - baselineStart >= BASELINE_MS) {
      baselineBleCount = bleSeenCount;
      for (int i = 0; i < bleSeenCount; i++) {
        memcpy(baselineBleArr[i], bleSeenArr[i], 18);
      }
      bleDone = true;
      if (wifiDone) {
        baselineProbeCount = probeSeenCount;
        for (int i = 0; i < probeSeenCount; i++) {
          memcpy(baselineProbeArr[i], probeSeenArr[i], 18);
        }
        isBaseline = false;
      }
    }
  }
}

// ------------ Wi-Fi Scan Task ------------
void WiFiTask(void*) {
  wifiSeenCount = 0;
  while (true) {
    WiFi.scanNetworks(true, true, false, 1000);
    unsigned long dl = millis() + 1000;
    int n;
    while ((n = WiFi.scanComplete()) < 0 && millis() < dl) {
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (n > 0) {
      for (int i = 0; i < n; i++) {
        const uint8_t* raw = WiFi.BSSID(i);
        char mac[18];
        snprintf(mac, sizeof(mac),
                 "%02X:%02X:%02X:%02X:%02X:%02X",
                 raw[0], raw[1], raw[2],
                 raw[3], raw[4], raw[5]);

        bool found = false;
        for (int j = 0; j < wifiSeenCount; j++) {
          if (strcmp(wifiSeenArr[j], mac) == 0) { found = true; break; }
        }
        if (!found && wifiSeenCount < MAX_WIFI) {
          strcpy(wifiSeenArr[wifiSeenCount], mac);
          wifiSeenCount++;
          currWiFiCount = wifiSeenCount;

          if (!isBaseline) {
            bool inBase = false;
            for (int k = 0; k < baselineWifiCount; k++) {
              if (strcmp(baselineWifiArr[k], mac) == 0) { inBase = true; break; }
            }
            if (!inBase) {
              enqueueFmt("Detected non-baseline WiFi: %s", mac);
            }
          }
        }
      }
    }
    WiFi.scanDelete();
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&snifferCallback);
    // small delay to prevent tight-loop timing issues
    vTaskDelay(pdMS_TO_TICKS(20));

    if (isBaseline && !wifiDone && millis() - baselineStart >= BASELINE_MS) {
      baselineWifiCount = wifiSeenCount;
      for (int i = 0; i < wifiSeenCount; i++) {
        memcpy(baselineWifiArr[i], wifiSeenArr[i], 18);
      }
      wifiDone = true;
      if (bleDone) {
        baselineProbeCount = probeSeenCount;
        for (int i = 0; i < probeSeenCount; i++) {
          memcpy(baselineProbeArr[i], probeSeenArr[i], 18);
        }
        isBaseline = false;
      }
    }
  }
}

// ------------ Baseline Progress Task ------------
void StatusTask(void*) {
  size_t lastMin = 0;
  while (isBaseline) {
    enqueueFmt("Baseline progress: %u BLE, %u WiFi, %u ProbeReq",
               (unsigned)currBLECount,
               (unsigned)currWiFiCount,
               (unsigned)currProbeCount);
    uint32_t elapsed = millis() - baselineStart;
    size_t mins = elapsed / 60000;
    if (mins > lastMin) {
      size_t rem = (BASELINE_MS - elapsed + 59999) / 60000;
      if (rem > 0) {
        enqueueFmt("Baseline Scan has %u minutes remaining.", (unsigned)rem);
      }
      lastMin = mins;
    }
    vTaskDelay(pdMS_TO_TICKS(15000));
  }
  enqueueFmt("Baseline complete: %u BLE, %u WiFi, %u ProbeReq",
             (unsigned)currBLECount,
             (unsigned)currWiFiCount,
             (unsigned)currProbeCount);
  enqueueFmt("=== Baseline complete ===");
  vTaskDelete(NULL);
}

// ------------ setup() & loop() ------------
void setup() {
  REG_CLR_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);
  delay(5000);
  baselineStart = millis();

  Serial.begin(115200);
  while (!Serial) vTaskDelay(pdMS_TO_TICKS(10));
  Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);

  printQ = xQueueCreate(20, sizeof(PrintMsg));
  probeQ = xQueueCreate(50, sizeof(ProbeEvent));

  xTaskCreatePinnedToCore(PrintTask,     "PrintTask",   4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(ChannelHopTask,"ChHop",       2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(ProbeTask,     "ProbeTask",   4096, NULL, 1, NULL, 1);

  enqueueFmt("Deepwoods Device Detection");
  enqueueFmt("5 minute Baseline Scan Started");

  NimBLEDevice::init("");
  pBLEScan = NimBLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(60);
  pBLEScan->setWindow(30);
  static MyScanCallbacks bleCb;
  pBLEScan->setScanCallbacks(&bleCb, false);

  WiFi.mode(WIFI_MODE_STA);
  WiFi.disconnect(true);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&snifferCallback);
  esp_wifi_set_ps(WIFI_PS_NONE);

  xTaskCreatePinnedToCore(BLETask,     "BLETask",     8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(WiFiTask,    "WiFiTask",    8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(StatusTask,  "StatusTask",  4096, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
