/*
*******************************************************************************
fingerprint and detect BLE/Wi‑Fi devices concurrently on separate cores,
with a 7 minute baseline phase and continuous detection thereafter,
reporting only non‑baseline detections
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

// ================================
// Pin Definitions
// ================================
#define SERIAL1_RX_PIN D5
#define SERIAL1_TX_PIN D4

// ------------ Timing & Flags ------------
static const uint32_t BASELINE_MS = 420000UL;
static uint32_t baselineStart = 0;
static volatile bool isBaseline = true, wifiDone = false, bleDone = false;

// ------------ Baseline Storage ------------
static std::vector<String> baselineWiFi, baselineBLE;

// ------------ Print Queue ------------
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
  PrintMsg m; strncpy(m.buf, s, sizeof(m.buf)-1); m.buf[127]=0;
  xQueueSend(printQ, &m, portMAX_DELAY);
}

void PrintTask(void*){
  PrintMsg m;
  while(xQueueReceive(printQ, &m, portMAX_DELAY)==pdTRUE){
    Serial.println(m.buf);
  }
}

// ------------ BLE Scan Globals & Callback ------------
static NimBLEScan* pBLEScan = nullptr;
std::vector<String>* bleVec = nullptr;

class MyScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    std::string mac = dev->getAddress().toString();
    std::transform(mac.begin(), mac.end(), mac.begin(), ::toupper);
    String addr(mac.c_str());
    auto &v = *bleVec;
    if (std::find(v.begin(), v.end(), addr)==v.end()) {
      v.push_back(addr);
      // **only** non‑baseline
      if (!isBaseline && std::find(baselineBLE.begin(), baselineBLE.end(), addr)==baselineBLE.end()) {
        enqueueFmt("Detected non‑baseline BLE: %s", addr.c_str());
        enqueueFmt("Alert over UART1: BLE %s", addr.c_str());
        Serial1.println("New device alert: BLE " + addr);
      }
    }
  }
};

// ------------ BLETask (Core 1) ------------
void BLETask(void*) {
  std::vector<String> seen;
  while(true){
    // 1 s chunked scan
    uint32_t w = (1000+999)/1000;
    bleVec = &seen;
    for(uint32_t i=0;i<w;i++){
      pBLEScan->start(1000,false,true);
      pBLEScan->clearResults();
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    // baseline complete?
    if (isBaseline && !bleDone && millis()-baselineStart>=BASELINE_MS) {
      baselineBLE = seen;
      bleDone = true;
      enqueueFmt("BLE baseline done: %u devs", (unsigned)seen.size());
      if (wifiDone) { isBaseline=false; enqueueMsg("=== Baseline complete ==="); }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ------------ WiFiTask (Core 1) ------------
void WiFiTask(void*) {
  std::vector<String> seen;
  while(true){
    // async 1 s scan
    WiFi.scanNetworks(true,true,false,1000);
    unsigned long dl = millis()+1000;
    int n;
    while((n=WiFi.scanComplete())<0 && millis()<dl) vTaskDelay(pdMS_TO_TICKS(50));
    if (n>0) {
      for(int i=0;i<n;i++){
        String b = WiFi.BSSIDstr(i);
        if (std::find(seen.begin(),seen.end(),b)==seen.end()){
          seen.push_back(b);
          // only non‑baseline
          if (!isBaseline && std::find(baselineWiFi.begin(),baselineWiFi.end(),b)==baselineWiFi.end()){
            enqueueFmt("Detected non‑baseline WiFi: %s", b.c_str());
            enqueueFmt("Alert over UART1: WiFi %s", b.c_str());
            Serial1.println("New device alert: WiFi " + b);
          }
        }
      }
    }
    WiFi.scanDelete();
    // baseline complete?
    if (isBaseline && !wifiDone && millis()-baselineStart>=BASELINE_MS) {
      baselineWiFi = seen;
      wifiDone = true;
      enqueueFmt("WiFi baseline done: %u BSSIDs", (unsigned)seen.size());
      if (bleDone) { isBaseline=false; enqueueMsg("=== Baseline complete ==="); }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ------------ setup() ------------
void setup(){
  delay(2000);
  baselineStart = millis();

  // USB + PrintTask (core 0)
  Serial.begin(115200);
  while(!Serial) delay(10);
  printQ = xQueueCreate(50,sizeof(PrintMsg));
  xTaskCreatePinnedToCore(PrintTask,"Printer",4096,NULL,2,NULL,0);
  enqueueMsg("USB ready.");

  // UART1 alerts
  Serial1.begin(115200,SERIAL_8N1,SERIAL1_RX_PIN,SERIAL1_TX_PIN);
  enqueueMsg("UART1 ready.");

  // BLE init
  NimBLEDevice::init("");
  pBLEScan = NimBLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(60);
  pBLEScan->setWindow(30);
  static MyScanCallbacks cb;
  pBLEScan->setScanCallbacks(&cb,false);
  enqueueMsg("BLE inited.");

  // launch tasks on core 1
  xTaskCreatePinnedToCore(BLETask, "BLETask",8192,NULL,1,NULL,1);
  xTaskCreatePinnedToCore(WiFiTask,"WiFiTask",8192,NULL,1,NULL,1);
  enqueueMsg("Scanning launched.");
}

// ------------ idle loop ------------
void loop(){
  vTaskDelay(pdMS_TO_TICKS(1000));
}
