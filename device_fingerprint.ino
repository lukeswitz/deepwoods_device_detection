/*
*******************************************************************************
fingerprint and detect ble/wifi devices and send detection over serial
* 
*******************************************************************************
*/

#include <M5Core2.h>
#include <FastLED.h>
#include <WiFi.h>             // For WiFi scanning
#include <NimBLEDevice.h>     // For BLE scanning
#include <vector>
#include <algorithm>

// ------------ LED Strip Settings (M5GO Base) ------------
#define NUM_LEDS 10
#define DATA_PIN 25  // GPIO pin connected to the LED strip
CRGB leds[NUM_LEDS];

// ------------ LED States ------------
enum LEDState {
    STARTUP,
    WAITING_BASELINE,
    FLASHING_PURPLE,
    SCANNING,
    DETECTED
};

volatile LEDState currentLEDState = STARTUP;

// ------------ Baseline Data ------------
static std::vector<String> baselineWiFi;
static std::vector<String> baselineBLE;

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

// ------------ Flag: Baseline Set? ------------
bool baselineSet = false;

// ------------ Button Press Timing ------------
static const unsigned long LONG_PRESS_MS = 2000; // 2 seconds

// Button 'A' (Set Baseline) Press Tracking
unsigned long pressStartA = 0;
bool isAPressed = false;
bool longPressA = false;

// Button 'C' (Clear Baseline) Press Tracking
unsigned long pressStartC = 0;
bool isCPressed = false;
bool longPressC = false;

// ------------ Define Colors ------------
CRGB colorBaselineCaptured = CRGB::Green;
CRGB colorBaselineCleared = CRGB::Blue;
CRGB colorNewDevice = CRGB::Red;
CRGB colorPurple = CRGB::Purple;

// ------------ Function Declarations ------------
void handleSetBaseline(bool isLongPress);
void handleClearBaseline(bool isLongPress);
void displayDevices();
void flashScreenRed();
void scanWiFiNetworks(std::vector<String> &results, uint32_t duration_ms = 0); 
void scanBLEDevices(std::vector<String> &results, uint32_t durationSec);
void flashLEDColor(CRGB color, int times, int durationMs = 300);
bool isInVector(const std::vector<String> &vec, const String &val);
bool updateDetectedDevices();
void LEDtask(void *arg);

// ------------ Vibration Functions ------------
void vibrate(int durationMs){
    Serial.printf("Vibrating for %d ms\n", durationMs);
    M5.Axp.SetVibration(true);   // Turn on vibration at full duty
    delay(durationMs);           // Vibrate for the specified duration
    M5.Axp.SetVibration(false);  // Turn off vibration
    Serial.println("Vibration stopped");
}

void singlePulse(){
    Serial.println("Single Pulse");
    vibrate(200); // Vibrate for 200 ms
    delay(300);   // Pause for 300 ms
}

void doublePulse(){
    Serial.println("Double Pulse");
    for(int i = 0; i < 2; i++){
        vibrate(200);
        delay(300);
    }
}

void triplePulse(){
    Serial.println("Triple Pulse");
    for(int i = 0; i < 3; i++){
        vibrate(200);
        delay(300);
    }
}

// ------------ Bluetooth Scanning Callbacks ------------
class MyAdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks {
public:
    void onResult(NimBLEAdvertisedDevice* advertisedDevice) override {
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

// NOTE: We do NOT declare `HardwareSerial Serial2(2);` 
// because M5Stack Core2 board definitions already do that for us.

// ------------ Setup Function ------------
void setup(){
    // Initialize Serial (USB) for debugging
    Serial.begin(115200);
    while (!Serial) {
        ; // Wait for serial port to connect. Needed for native USB
    }
    Serial.println("Setup Started");

    // Initialize M5Core2
    M5.begin();
    M5.Lcd.clear(BLACK);
    M5.Lcd.setTextSize(1); 
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.println("M5Core2 Touch + WiFi/BLE Scanner");

    // Initialize LED strip (M5GO Base)
    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(100);
    FastLED.clear();
    FastLED.show();

    // Initialize the built-in Serial2 (Port C) on M5Core2
    // Port C is typically TX on GPIO 13, RX on GPIO 14
    Serial2.begin(115200, SERIAL_8N1, 14, 13);
    Serial.println("Serial2 (Port C) initialized.");

    // Initialize NimBLE
    NimBLEDevice::init("");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9); // Adjust BLE TX power if desired

    // Perform 2 pulses at bootup
    doublePulse();

    // Set LEDs to blue during startup
    fill_solid(leds, NUM_LEDS, CRGB::Blue);
    FastLED.show();

    // Transition to WAITING_BASELINE state
    currentLEDState = WAITING_BASELINE;

    // Create LED Task
    xTaskCreatePinnedToCore(LEDtask, "LEDTask", 4096, NULL, 2, NULL, 0);

    Serial.println("Setup Complete");
}

// ------------ Loop Function ------------
void loop(){
    // Update M5Core2 (handles button states, etc.)
    M5.update();

    // Handle Button 'A' (Set Baseline)
    if (M5.BtnA.isPressed()) {
        if (!isAPressed) {
            isAPressed = true;
            pressStartA = millis();
            longPressA = false;
            Serial.println("Button A Pressed");
            singlePulse(); // Emit single pulse on press down
        } else {
            // Check if pressed long enough for long press
            if (!longPressA && (millis() - pressStartA >= LONG_PRESS_MS)) {
                longPressA = true;
                Serial.println("Button A Long Press Detected");
            }
        }
    } else {
        if (isAPressed) {
            unsigned long pressDuration = millis() - pressStartA;
            if (pressDuration >= LONG_PRESS_MS) {
                handleSetBaseline(true);
            } else {
                handleSetBaseline(false);
            }
            isAPressed = false;
            longPressA = false;
            Serial.println("Button A Released");
        }
    }

    // Handle Button 'C' (Clear Baseline)
    if (M5.BtnC.isPressed()) {
        if (!isCPressed) {
            isCPressed = true;
            pressStartC = millis();
            longPressC = false;
            Serial.println("Button C Pressed");
            singlePulse(); 
        } else {
            if (!longPressC && (millis() - pressStartC >= LONG_PRESS_MS)) {
                longPressC = true;
                Serial.println("Button C Long Press Detected");
            }
        }
    } else {
        if (isCPressed) {
            unsigned long pressDuration = millis() - pressStartC;
            if (pressDuration >= LONG_PRESS_MS) {
                handleClearBaseline(true);
            } else {
                handleClearBaseline(false);
            }
            isCPressed = false;
            longPressC = false;
            Serial.println("Button C Released");
        }
    }

    // ------------ Conditional Continuous Scanning ------------
    if (baselineSet) {
        // Perform WiFi scan
        currentWiFi.clear();
        Serial.println("Starting WiFi scan for 30 seconds...");
        scanWiFiNetworks(currentWiFi, 2000); // Adjust if you want a real 30s scan
        Serial.println("WiFi scan completed.");

        // Perform BLE scan
        currentBLE.clear();
        Serial.println("Starting BLE scan for 30 seconds...");
        scanBLEDevices(currentBLE, 2); // Adjust to 30 if you want a real 30s scan
        Serial.println("BLE scan completed.");

        // Update detected devices list and check for new devices
        bool newDeviceDetected = updateDetectedDevices();

        // If any new devices were detected, trigger feedback
        if(newDeviceDetected){
            Serial.println("Non-whitelisted device detected!");

            // Provide vibration feedback
            triplePulse();

            // Provide LED feedback (blink red 3 times)
            currentLEDState = DETECTED;

            // Optionally, flash the screen red
            flashScreenRed();
        }

        // Update display
        displayDevices();
    }

    delay(3000); // Wait before next scan cycle
}

// ------------ Handle Set Baseline ------------
void handleSetBaseline(bool isLongPress){
    if (isLongPress){
        Serial.println("Handling Set Baseline - Long Press");
        M5.Lcd.clear();
        M5.Lcd.setCursor(0, 10);
        M5.Lcd.setTextColor(TFT_WHITE);
        M5.Lcd.println("Capturing baseline (60s)...");

        doublePulse(); // Provide vibration feedback

        // Clear previous baseline
        baselineWiFi.clear();
        baselineBLE.clear();

        // Perform WiFi scan
        Serial.println("Starting baseline WiFi scan for 30 seconds...");
        scanWiFiNetworks(baselineWiFi, 30000);
        Serial.println("Baseline WiFi scan completed.");

        // Perform BLE scan
        Serial.println("Starting baseline BLE scan for 30 seconds...");
        scanBLEDevices(baselineBLE, 30);
        Serial.println("Baseline BLE scan completed.");

        // Mark baseline as set
        baselineSet = true;

        // Provide LED feedback (flash purple 3 times)
        currentLEDState = FLASHING_PURPLE;

        // Confirmation message
        M5.Lcd.clear();
        M5.Lcd.setCursor(0, 10);
        M5.Lcd.setTextColor(TFT_GREEN);
        M5.Lcd.println("Baseline captured!");
        M5.Lcd.setTextColor(TFT_WHITE);
        M5.Lcd.println("Monitoring for new devices...");
        delay(1000);
    }
    else{
        Serial.println("Handling Set Baseline - Short Press");
        // Additional short-press logic if desired
    }
}

// ------------ Handle Clear Baseline ------------
void handleClearBaseline(bool isLongPress){
    if (isLongPress){
        Serial.println("Handling Clear Baseline - Long Press");
        M5.Lcd.clear();
        M5.Lcd.setCursor(0, 10);
        M5.Lcd.setTextColor(TFT_WHITE);
        M5.Lcd.println("Clearing baseline...");

        doublePulse();

        // Clear baseline data
        baselineWiFi.clear();
        baselineBLE.clear();
        baselineSet = false;

        // Clear detected devices lists
        detectedNonWhitelistedWiFi.clear();
        detectedNonWhitelistedBLE.clear();
        previousDetectedNonWhitelistedWiFi.clear();
        previousDetectedNonWhitelistedBLE.clear();

        // Flash LED color (blue) 2 times
        flashLEDColor(colorBaselineCleared, 2);

        // Transition to WAITING_BASELINE
        currentLEDState = WAITING_BASELINE;

        // Confirmation message
        M5.Lcd.clear();
        M5.Lcd.setCursor(0, 10);
        M5.Lcd.setTextColor(TFT_CYAN);
        M5.Lcd.println("Baseline cleared!");
        M5.Lcd.setTextColor(TFT_WHITE);
        M5.Lcd.println("Awaiting baseline...");
        delay(1000);
    }
    else{
        Serial.println("Handling Clear Baseline - Short Press");
        // Additional short-press logic if desired
    }
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
            // Send to Serial2 (Port C)
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
            // Send to Serial2 (Port C)
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

// ------------ Display Devices on LCD ------------
void displayDevices(){
    M5.Lcd.clear();
    M5.Lcd.setTextSize(1.5);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.println("Scanning for devices...");

    // Display Non-Whitelisted WiFi
    if(!detectedNonWhitelistedWiFi.empty()){
        M5.Lcd.setTextColor(TFT_RED);
        M5.Lcd.println("Non-Whitelisted WiFi Devices:");
        for(auto &dev : detectedNonWhitelistedWiFi){
            M5.Lcd.println("  " + dev.macAddress);
        }
    }

    // Display Non-Whitelisted BLE
    if(!detectedNonWhitelistedBLE.empty()){
        M5.Lcd.setTextColor(TFT_RED);
        M5.Lcd.println("Non-Whitelisted BLE Devices:");
        for(auto &dev : detectedNonWhitelistedBLE){
            M5.Lcd.println("  " + dev.macAddress);
        }
    }
}

// ------------ Flash Screen Red ------------
void flashScreenRed(){
    Serial.println("Flashing screen red");
    for(int i = 0; i < 3; i++){
        M5.Lcd.fillScreen(TFT_RED);
        delay(200);
        M5.Lcd.fillScreen(BLACK);
        delay(200);
    }
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

// ------------ Flash LED Strip ------------
void flashLEDColor(CRGB color, int times, int durationMs){
    Serial.printf("Flashing LEDs %d times with color RGB(%d, %d, %d)\n",
                  times, color.r, color.g, color.b);
    for(int t = 0; t < times; t++){
        fill_solid(leds, NUM_LEDS, color);
        FastLED.show();
        delay(durationMs);

        fill_solid(leds, NUM_LEDS, CRGB::Black);
        FastLED.show();
        delay(durationMs);
    }
}

// ------------ Check if a Value Exists in a Vector ------------
bool isInVector(const std::vector<String> &vec, const String &val){
    return (std::find(vec.begin(), vec.end(), val) != vec.end());
}

// ------------ LED Task (Independent LED Control) ------------
void LEDtask(void *arg){
    while(1){
        switch(currentLEDState){
            case STARTUP:
                // Handled in setup()
                break;

            case WAITING_BASELINE:
                fill_solid(leds, NUM_LEDS, CRGB::Blue);
                FastLED.show();
                break;

            case FLASHING_PURPLE:
                // Flash purple 3 times
                for(int i = 0; i < 3; i++){
                    fill_solid(leds, NUM_LEDS, colorPurple);
                    FastLED.show();
                    delay(300);
                    fill_solid(leds, NUM_LEDS, CRGB::Black);
                    FastLED.show();
                    delay(300);
                }
                currentLEDState = SCANNING;
                break;

            case SCANNING:
                fill_solid(leds, NUM_LEDS, CRGB::Green);
                FastLED.show();
                break;

            case DETECTED:
                // Blink red 3 times
                for(int i = 0; i < 3; i++){
                    fill_solid(leds, NUM_LEDS, colorNewDevice);
                    FastLED.show();
                    delay(300);
                    fill_solid(leds, NUM_LEDS, CRGB::Black);
                    FastLED.show();
                    delay(300);
                }
                currentLEDState = SCANNING;
                break;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}