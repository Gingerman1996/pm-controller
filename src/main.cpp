#include <ArduinoJson.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <HTTPClient.h>
#include <NTPClient.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

#include "MyLog.h"

#include "Calculator.h"
#include "PMS.h"

// Pin for monitoring loop
int Trig1 = 2;
#define _SR 4 // speed range
#define NP 2  // number of TACH pulse per revolution

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Multi-task parameter
TaskHandle_t Fan_controller_Handle = NULL;
TaskHandle_t Http_request_Handle = NULL;
TaskHandle_t Http_postRoon_Handle = NULL;
TaskHandle_t Http_postInlet_Handle = NULL;

// Delay task
const TickType_t xDelay100m = pdMS_TO_TICKS(100);
const TickType_t xDelay1000m = pdMS_TO_TICKS(1000);
const TickType_t xDelay1min = pdMS_TO_TICKS(1000 * 60);

// Task Function
void Fan_controller(void *parameter);
void Http_request(void *parameter);
void Http_postRoom(void *parameter);
void Http_postInlet(void *parameter);

void printLocalPM(bool localPmsDataValid, PMS::DATA localPmsData);
void startPMcontrol();

// Network credentials
const char *ssid = "ag-diamond_2.4GHz";
const char *password = "0505563014466";

// NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0,
                     60000); // Update time every 60 seconds

// API URL
const char *apiUrl =
    "https://api.airgradient.com/public/api/v1/locations/measures/"
    "current?token=dadf4720-44da-4906-bd4f-9a57d6e073a4";

unsigned long bootTime =
    millis(); // gives time since startup in ms. Overflow after 50 days.
unsigned long t = millis();

// Loop variables
const int D = 50;             // delay between loops in ms
unsigned long intervalTs = 0; // timestamp for controlling interval
unsigned long durationTs = 0; // timestamp for controlling fan duration
unsigned long pre2 = 0;       // timestamp for controlling print interval
unsigned long pre2_1 = 0;     // timestamp for printing board data
unsigned long pre2_11 = 0;    // timestamp for printing weighted average
unsigned long pmsReadTs = 0;  // timestamp for controlling PMS reading interval
unsigned long printTimestamp =
    0; // timestamp for controlling print interval (3 seconds)
unsigned int fanSpeedPercent = 0;
float duration = 0;
int InletConcentration = 0;
float interval_s = 0;
bool fanIsOn = false;
float meanpm02 = 0;
float ref[12];

// LocalPmsData
bool localPmsDataValid = false;
PMS::DATA localPmsData;

// PM values - Not used anymore since we only fetch data from sensor #5
// float pmValues[5] = {0};
// float weights[5] = {1, 1, 1, 1, 1};
// int numSensors = 0;

// PMS sensor
PMS local_pms(Serial1);

// ## MAX31790 ("4pin fan" controller - 6 channel) address
const byte MAX31790 = 0x23;
byte data, data0, data1;

#define INIT_INTERVAL_SECONDS                                                  \
  150 // initial interval between 1st pulse and next pulse (longer for
      // distribution of aerosol)
#define REG_INTERVAL_SECONDS                                                   \
  90 // regulation interval between each pulse after the 1st to maintain pm2.5
     // concentration
#define PMS_READ_INTERVAL_SECONDS 1

// === PM2.5 EXPONENTIAL RISE CONFIGURATION ===
// 
// AUTO_START_DELAY_SECONDS: Wait time before starting automatic system after boot
// - Default value: 60 seconds (1 minute)
// - For testing: Reduce to 10 seconds
//
// EXPONENTIAL_RISE_DURATION_SECONDS: Total time for PM to rise from 0 to 50 μg/m³
// - Default value: 7200 seconds (2 hours)
// - For testing: Reduce to 300 seconds (5 minutes)
//
// Total cycle time = AUTO_START_DELAY + EXPONENTIAL_RISE_DURATION + MAINTENANCE_DURATION
// Default: 1 minute + 2 hours + 20 minutes = 2 hours 21 minutes
//
#define AUTO_START_DELAY_SECONDS 60        // Auto start after boot (60 seconds = 1 minute)
#define EXPONENTIAL_RISE_DURATION_SECONDS 7200  // 2 hours exponential rise (7200 seconds = 2 hours)
#define MAINTENANCE_DURATION_SECONDS 1200       // 20 minutes maintenance at target (1200 seconds = 20 minutes)

// For testing purposes, uncomment these lines for faster cycles:
// #define AUTO_START_DELAY_SECONDS 10      // 10 seconds for testing
// #define EXPONENTIAL_RISE_DURATION_SECONDS 300  // 5 minutes for testing
// #define MAINTENANCE_DURATION_SECONDS 60  // 1 minute for testing

unsigned int TARGET_PM02 =
    0; // target pm2.5 concentration in micrograms per cubic meter

// Remove old step-based targets, now using exponential rise
// const int targetValues[] = {5, 10, 20, 40, 100, 150};
// const float targetValuesWorldStandard[] = {2, 5, 10, 25, 35, 50};
const float MAX_PM_TARGET = 50.0;  // Maximum PM2.5 target for exponential rise

unsigned long exponentialStartTime = 0;
bool exponentialRiseActive = false;
unsigned long maintenanceStartTime = 0;
bool maintenanceActive = false;
unsigned long currentNtpTime = 0;

unsigned long lastUpdateTime = 0;
bool isAutoMode = false;
bool isRunning = false;

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

void set_I2C_register(byte ADDRESS, byte REGISTER, byte VALUE) {
  Wire.beginTransmission(ADDRESS);
  Wire.write(REGISTER);
  Wire.write(VALUE);
  Wire.endTransmission();
}

byte get_I2C_register(byte ADDRESS, byte REGISTER) {
  Wire.beginTransmission(ADDRESS);
  Wire.write(REGISTER);
  Wire.endTransmission();
  Wire.requestFrom(ADDRESS, 1); // read 1 byte
  byte x = Wire.read();
  return x;
}

// Function calculateWeightedAverage removed because it's no longer needed
// since we only fetch data from sensor #5

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    if (value == "1") {
      // isRunning = true;
      // isAutoMode = false;
      // TARGET_PM02 = 0;
      // currentHour = 0;
      // MyLog::info(
      //     "System ready for operation, waiting for mode (auto or manual)");
      // xTaskCreatePinnedToCore(Fan_controller, "Fan_controller_task", 4096,
      // NULL,
      //                         1, &Fan_controller_Handle, 1);
    } else if (value == "0") {
      // TARGET_PM02 = 0;
      // isRunning = false;
      // MyLog::info("Auto mode completed, TARGET_PM02 set to 0");
      // vTaskDelete(Fan_controller_Handle);
    }
  }
};

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 26, 25);
  delay(1000);
  while (!Serial) {
    // wait for serial monitor to be monitoring
  }

  // Set and scan I²C bus
  Wire.begin();
  pinMode(13, INPUT_PULLUP);

  // BLEDevice::init("PM Controller");
  // BLEServer *pServer = BLEDevice::createServer();
  // BLEService *pService = pServer->createService(SERVICE_UUID);

  // pCharacteristic = pService->createCharacteristic(
  //     CHARACTERISTIC_UUID,
  //     BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  // pCharacteristic->setCallbacks(new MyCallbacks());
  // pService->start();

  // BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  // pAdvertising->addServiceUUID(SERVICE_UUID);
  // pAdvertising->start();

  // Reset MAX31790 (POR)
  set_I2C_register(MAX31790, 0x00, B01010000); // reset
  delay(100);

  // PWM Frequency       first 4 bits 0111 (0-10V), next 4 bits 1011 (PWM)
  set_I2C_register(MAX31790, 0x01, B01101010); // 1.25kHz
  delay(1);

  // Configuration
  data = B00001000; // PWM mode - 'Spin-up' OFF - control ON - Tach count- PWM
                    // yeah
  for (int i = 0x02; i <= 0x07; i++) {
    set_I2C_register(MAX31790, i, data);
    delay(1);
  }

  // Pin for monitoring loop
  pinMode(Trig1, OUTPUT);
  digitalWrite(Trig1, LOW);
  MyLog::debug("Setup Done");

  local_pms.passiveMode();
  local_pms.wakeUp();

  // Connect to WiFi first
  WiFi.begin(ssid, password);
  MyLog::info("Connecting to WiFi...");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    MyLog::info(".");
  }
  
  MyLog::info("WiFi connected!");
  MyLog::info("IP address: %s", WiFi.localIP().toString().c_str());

  // Initialize NTP client
  timeClient.begin();
  timeClient.update();

  // Get and print MAC Address
  String macAddress = WiFi.macAddress();
  Serial.println("MAC Address: " + macAddress);

  // Wait a bit before starting HTTP tasks
  delay(5000);

  // xTaskCreatePinnedToCore(Fan_controller, "Fan_controller_task", 4096, NULL,
  // 1,
  //                         &Fan_controller_Handle, 1);

  xTaskCreatePinnedToCore(Http_request, "Http_request_task", 4096, NULL, 1,
                          &Http_request_Handle, 0);
  xTaskCreatePinnedToCore(Http_postRoom, "Http_postRoom_task", 4096, NULL, 1,
                          &Http_postRoon_Handle, 0);

  xTaskCreatePinnedToCore(Http_postInlet, "Http_postInlet_task", 4096, NULL, 1,
                          &Http_postInlet_Handle, 0);
}

int MultiplicationCombine(unsigned int x_high, unsigned int x_low) {
  int combined;
  combined = x_high;
  combined = combined * 256;
  combined |= x_low;
  return combined;
}

void loop() {
  timeClient.update();
  currentNtpTime = timeClient.getEpochTime();

  // Serial command handling
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.equalsIgnoreCase("start")) {
      startPMcontrol();
    } else if (command.equalsIgnoreCase("stop")) {
      isRunning = false;
      exponentialRiseActive = false;
      maintenanceActive = false;
      TARGET_PM02 = 0;
      MyLog::info("System stopped");
      fanIsOn = false;
      set_I2C_register(MAX31790, 0x40, 0);
      set_I2C_register(MAX31790, 0x41, 0);
      vTaskDelete(Fan_controller_Handle);
    } else if (command.equalsIgnoreCase("status")) {
      if (exponentialRiseActive) {
        unsigned long currentTime = millis();
        float elapsedTime = (currentTime - exponentialStartTime) / 1000.0;
        float progress = (elapsedTime / EXPONENTIAL_RISE_DURATION_SECONDS) * 100.0;
        MyLog::info("Exponential rise active: %.1f%% complete, TARGET: %d μg/m³", 
                    progress, TARGET_PM02);
      } else if (maintenanceActive) {
        unsigned long currentTime = millis();
        float elapsedTime = (currentTime - maintenanceStartTime) / 1000.0;
        float remaining = (MAINTENANCE_DURATION_SECONDS - elapsedTime) / 60.0;
        MyLog::info("Maintenance phase: %.1f minutes remaining at %d μg/m³", 
                    remaining, TARGET_PM02);
      } else if (isRunning) {
        MyLog::info("System running, maintaining TARGET: %d μg/m³", TARGET_PM02);
      } else {
        MyLog::info("System stopped");
      }
    } else if (command.equalsIgnoreCase("help")) {
      MyLog::info("Available commands:");
      MyLog::info("  start  - Start exponential PM rise (0 to 50 μg/m³ over 2 hours)");
      MyLog::info("          Then maintain 50 μg/m³ for 20 minutes and shutdown");
      MyLog::info("  stop   - Stop system and set fan to 0");
      MyLog::info("  status - Show current system status");
      MyLog::info("  help   - Show this help message");
    }
  }
  // Update system state based on current phase
  if (isAutoMode && isRunning) {
    unsigned long currentTime = millis();
    
    if (exponentialRiseActive) {
      // Phase 1: Exponential rise from 0 to 50 μg/m³
      float elapsedTime = (currentTime - exponentialStartTime) / 1000.0; // Convert to seconds
      
      if (elapsedTime >= EXPONENTIAL_RISE_DURATION_SECONDS) {
        // Exponential rise completed, start maintenance phase
        TARGET_PM02 = MAX_PM_TARGET;
        exponentialRiseActive = false;
        maintenanceActive = true;
        maintenanceStartTime = currentTime;
        MyLog::info("Exponential rise completed. Starting maintenance phase at %.1f μg/m³ for %.1f minutes", 
                    (float)TARGET_PM02, MAINTENANCE_DURATION_SECONDS / 60.0);
      } else {
        // Calculate current exponential target
        float currentTarget = Calculator::calculateExponentialTarget(currentTime);
        TARGET_PM02 = (unsigned int)currentTarget;
        
        // Log progress every 30 seconds to avoid spam
        static unsigned long lastProgressLog = 0;
        if (currentTime - lastProgressLog >= 30000) { // 30 seconds
          lastProgressLog = currentTime;
          MyLog::info("Exponential rise progress: %.1f/%.1f μg/m³ (%.1f%% complete)", 
                      currentTarget, MAX_PM_TARGET, (elapsedTime / EXPONENTIAL_RISE_DURATION_SECONDS) * 100.0);
        }
      }
    } else if (maintenanceActive) {
      // Phase 2: Maintain target for specified duration
      float elapsedTime = (currentTime - maintenanceStartTime) / 1000.0; // Convert to seconds
      
      if (elapsedTime >= MAINTENANCE_DURATION_SECONDS) {
        // Maintenance completed, shutdown system
        TARGET_PM02 = 0;
        isRunning = false;
        exponentialRiseActive = false;
        maintenanceActive = false;
        MyLog::info("Maintenance phase completed. System shutdown - PM will naturally decay");
        fanIsOn = false;
        set_I2C_register(MAX31790, 0x40, 0);
        set_I2C_register(MAX31790, 0x41, 0);
        vTaskDelete(Fan_controller_Handle);
      } else {
        // Continue maintaining target
        TARGET_PM02 = MAX_PM_TARGET;
        
        // Log maintenance progress every 2 minutes
        static unsigned long lastMaintenanceLog = 0;
        if (currentTime - lastMaintenanceLog >= 120000) { // 2 minutes
          lastMaintenanceLog = currentTime;
          float remaining = (MAINTENANCE_DURATION_SECONDS - elapsedTime) / 60.0;
          MyLog::info("Maintenance phase: %.1f minutes remaining at %d μg/m³", 
                      remaining, TARGET_PM02);
        }
      }
    }
  } else if (!isAutoMode && !isRunning &&
             currentNtpTime - lastUpdateTime >= AUTO_START_DELAY_SECONDS) {
    // Auto-start the exponential rise mode
    startPMcontrol();
  }

  // // Debug output to monitor status
  MyLog::debug("Current TARGET_PM02: %d", TARGET_PM02);
  MyLog::debug("Current Average_PM02: %.2f", meanpm02);
  delay(1000);
}

void startPMcontrol() {
  isAutoMode = true;
  isRunning = true;
  exponentialRiseActive = true;
  maintenanceActive = false;
  exponentialStartTime = millis();
  maintenanceStartTime = 0;
  TARGET_PM02 = 0;  // Start from 0
  lastUpdateTime = currentNtpTime;
  
  // Initialize exponential rise in Calculator
  Calculator::initExponentialRise(MAX_PM_TARGET, EXPONENTIAL_RISE_DURATION_SECONDS);
  
  MyLog::info("Exponential PM rise mode started - 0 to %.1f μg/m³ over %.1f minutes", 
              MAX_PM_TARGET, EXPONENTIAL_RISE_DURATION_SECONDS / 60.0);
  MyLog::info("After reaching target, will maintain for %.1f minutes then shutdown", 
              MAINTENANCE_DURATION_SECONDS / 60.0);
  xTaskCreatePinnedToCore(Fan_controller, "Fan_controller_task", 4096, NULL, 1,
                          &Fan_controller_Handle, 1);
}

void printLocalPM(bool localPmsDataValid, PMS::DATA localPmsData) {
  if (localPmsDataValid) {
    MyLog::info("PMS data is valid.");
    MyLog::info("Local PM 1.0 (ug/m3): %d", localPmsData.PM_AE_UG_1_0);
    MyLog::info("Local PM 2.5 (ug/m3): %d", localPmsData.PM_AE_UG_2_5);
    MyLog::info("Local PM 10.0 (ug/m3): %d", localPmsData.PM_AE_UG_10_0);
  }
}

void Fan_controller(void *parameter) {
  while (true) {
    // Setting interval length based on time
    interval_s = millis() - bootTime > INIT_INTERVAL_SECONDS * 1000
                     ? REG_INTERVAL_SECONDS
                     : INIT_INTERVAL_SECONDS;

    // Read smth
    data0 = get_I2C_register(MAX31790, 0x22); // Tach1 count
    data1 = get_I2C_register(MAX31790, 0x23);

    uint16_t tach_out = ((((int16_t)data0) << 8) | data1) >> 5;

    // convert to RPM
    int tach1 = 60 * _SR * 8192 / (NP * tach_out);

    if (millis() > pmsReadTs + (PMS_READ_INTERVAL_SECONDS * 1000)) {
      pmsReadTs = millis();
      while (Serial1.available()) {
        Serial1.read();
      }
      local_pms.requestRead();
      localPmsDataValid = local_pms.readUntil(localPmsData);
      printLocalPM(localPmsDataValid, localPmsData);
    }

    if (!fanIsOn) {
      fanSpeedPercent = 0;
      InletConcentration = Calculator::calculateInletConcentration(TARGET_PM02);
      if (InletConcentration > 900) {
        InletConcentration = 900;
      }
    }

    //////////////////////////////////////////////////// print
    if (millis() > pre2 + 2000) {
      pre2 = millis();
      digitalWrite(Trig1, HIGH);
      // Serial.println("/////////////////////////////////////////////////////");
      // MyLog::info("Average PM2.5: \t\t\t%.2f", meanpm02);
      // MyLog::info("pm2.5_target = \t\t\t%u", TARGET_PM02);
      // MyLog::info("Fan is running: \t\t\t%s", fanIsOn ? "true" : "false");
      // MyLog::info("LocalPmsDataValid is: \t\t%s",
      //             localPmsDataValid ? "true" : "false");
      // MyLog::info("Fan speed is: \t\t\t%d %", fanSpeedPercent);
      // MyLog::info("Fan speed: \t\t\t%d RPM",
      //             Calculator::convertPercentageToRPM(fanSpeedPercent));
      // MyLog::info("InletConcentration: \t\t%d ug/m3", InletConcentration);
      // Serial.println("/////////////////////////////////////////////////////");
    }
    unsigned long now = millis();

    if (WiFi.status() == WL_CONNECTED) {
      if (meanpm02 < TARGET_PM02 && localPmsDataValid) {
        fanSpeedPercent = Calculator::getFanRunSpeed(meanpm02, TARGET_PM02);
        duration = Calculator::getFanRunningIntervalV2(meanpm02, TARGET_PM02,
                                                       fanSpeedPercent);

        fanIsOn = true;
        intervalTs = now;
        durationTs = millis();
        uint16_t pwm_bit = Calculator::scaleDutyCycle(fanSpeedPercent);

        pwm_bit = pwm_bit << 7;
        byte pwm0 = pwm_bit >> 8;
        byte pwm1 = pwm_bit;
        set_I2C_register(MAX31790, 0x40, pwm0); // Channel 1 --> Fan
        set_I2C_register(MAX31790, 0x41, pwm1);
      } else {
        fanIsOn = false;
        set_I2C_register(MAX31790, 0x40, 0);
        set_I2C_register(MAX31790, 0x41, 0);
      }

      // if close sensor find reach the target will stop the fan
      if (localPmsDataValid &&
          localPmsData.PM_AE_UG_2_5 >= InletConcentration) {
        fanIsOn = false;
        set_I2C_register(MAX31790, 0x40, 0);
        set_I2C_register(MAX31790, 0x41, 0);
      }
    }
    vTaskDelay(xDelay100m);
  }
}

void Http_request(void *parameter) {
  // No longer need numSensors since we only fetch data from sensor #5
  while (true) {
    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient https;
      https.begin(apiUrl); // Specify the URL
      // Perform the GET request
      int httpCode = https.GET();
      vTaskDelay(xDelay1000m); // TODO: slow down the server polling
      if (httpCode > 0) {
        // Check if the GET request was successful
        if (httpCode == HTTP_CODE_OK) {
          String payload = https.getString();

          // Parse the JSON response
          DynamicJsonDocument doc(49152);
          DeserializationError error = deserializeJson(doc, payload);

          if (error) {
            https.end(); // Close connection
            MyLog::error("%s", String("deserializeJson() failed: ").c_str());
            MyLog::error("%s", String(error.c_str()).c_str());
            vTaskDelay(xDelay1000m);
            continue;
          }

          // Fetch data only from sensor #5 (index 5)
          meanpm02 = float(doc[5]["pm02"]);

          // MyLog::info("%s%s", String("Mean pm02: ").c_str(),
          //             String(meanpm02).c_str());
        }
      } else {
        https.end(); // Close connection
        Serial.printf("GET request failed, error: %s\n",
                      https.errorToString(httpCode).c_str());
        vTaskDelay(xDelay1000m);
        continue;
      }
      https.end(); // Close connection
    }
    vTaskDelay(xDelay1000m); // Delay before the next request
  }
}

void Http_postRoom(void *parameter) {
  // Variable to track last time data was sent
  unsigned long previousMillis = 0;
  const long interval = 60000; // 1 minute interval

  while (true) {
    unsigned long currentMillis = millis(); // Get the current time

    // Check if 1 minute has passed
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis; // Update the last send time

      if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;

        String serverURL =
            "http://hw.airgradient.com/sensors/ffffffffffff/measures";
        http.begin(serverURL);
        http.addHeader("Content-Type",
                       "application/json"); // Set header to JSON

        // Create JSON document
        StaticJsonDocument<200> jsonDoc;
        jsonDoc["wifi"] = WiFi.RSSI();
        jsonDoc["pm02"] = meanpm02;

        // Convert JSON document to a string
        String jsonData;
        serializeJson(jsonDoc, jsonData);

        // Send HTTP POST request
        int httpResponseCode = http.POST(jsonData);

        if (httpResponseCode > 0) {
          String response = http.getString();
          // MyLog::debug("HTTP Response code: %s", String(httpResponseCode));
          // MyLog::debug("Response: %s", response);
        } else {
          // MyLog::error("Error in sending POST request");
        }

        http.end(); // Close HTTP connection
      } else {
        MyLog::debug("WiFi Disconnected");
      }
    }

    vTaskDelay(xDelay1000m);
  }
}

void Http_postInlet(void *parameter) {
  unsigned long previousMillis = 0;
  const long interval = 60000; // 1 minute interval
  while (true) {
    unsigned long currentMillis = millis(); // Get the current time

    // Check if 1 minute has passed
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis; // Update the last send time
      if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        String serverURL =
            "http://hw.airgradient.com/sensors/94e6868d45b0/measures";
        http.begin(serverURL);
        http.addHeader("Content-Type",
                       "application/json"); // Set header to JSON
        int tacho = Calculator::convertPercentageToRPM(fanSpeedPercent);
        local_pms.requestRead();
        localPmsDataValid = local_pms.readUntil(localPmsData);
        printLocalPM(localPmsDataValid, localPmsData);
        // Create JSON document
        StaticJsonDocument<200> jsonDoc;
        jsonDoc["wifi"] = WiFi.RSSI();
        jsonDoc["pm02"] = localPmsData.PM_AE_UG_2_5;
        jsonDoc["tacho"] = tacho;

        // Convert JSON document to a string
        String jsonData;
        serializeJson(jsonDoc, jsonData);

        // Send HTTP POST request
        int httpResponseCode = http.POST(jsonData);

        if (httpResponseCode > 0) {
          String response = http.getString();
        } else {
          // MyLog::error("Error in sending POST request");
        }

        http.end(); // Close HTTP connection
      } else {
        // MyLog::debug("WiFi Disconnected");
      }
    }

    vTaskDelay(xDelay1000m);
  }
}