#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <NTPClient.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <esp_now.h>

#include "MyLog.h"

// #define USE_ESP_NOW

#include "Calculator.h"
#include "ESPNowMaster.h"
#include "PMS.h"

// Pin for monitoring loop
int Trig1 = 2;
#define _SR 4  // speed range
#define NP 2   // number of TACH pulse per revolution

// Multi-task parameter
TaskHandle_t Fan_controller_Handle = NULL;
TaskHandle_t Data_acquisition_Handle = NULL;
TaskHandle_t Http_request_Handle = NULL;
const TickType_t xDelay100m = pdMS_TO_TICKS(100);

void Fan_controller(void *parameter);
void Data_acquisition(void *parameter);
void Http_request(void *parameter);
void printLocalPM(bool localPmsDataValid, PMS::DATA localPmsData);

// Network credentials
const char *ssid = "ag-diamond_2.4GHz";
const char *password = "0505563014466";
const char *APSSID = "ESP32_Master_AP";
ESPNowMaster espNowMaster(ssid, password, APSSID);

// NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0,
                     60000);  // Update time every 60 seconds

// API URL
const char *apiUrl =
    "https://api.airgradient.com/public/api/v1/locations/measures/"
    "current?token=dadf4720-44da-4906-bd4f-9a57d6e073a4";

unsigned long bootTime =
    millis();  // gives time since startup in ms. Overflow after 50 days.
unsigned long t = millis();

// Loop variables
const int D = 50;              // delay between loops in ms
unsigned long intervalTs = 0;  // timestamp for controlling interval
unsigned long durationTs = 0;  // timestamp for controlling fan duration
unsigned long pre2 = 0;        // timestamp for controlling print interval
unsigned long pre2_1 = 0;      // timestamp for printing board data
unsigned long pre2_11 = 0;     // timestamp for printing weighted average
unsigned long pmsReadTs = 0;   // timestamp for controlling PMS reading interval
unsigned long printTimestamp =
    0;  // timestamp for controlling print interval (3 seconds)
unsigned int fanSpeedPercent = 0;
float duration = 0;
float interval_s = 0;
bool fanIsOn = false;
float meanpm02 = 0;
float ref[12];

// PM values and weights
uint64_t boardId;
float pm25, temp, humi;
unsigned long timestamp;
float pmValues[128] = {0};
float weights[128] = {1.0};
int numSensors = 0;

// PMS sensor
PMS local_pms(Serial1);

// ## MAX31790 ("4pin fan" controller - 6 channel) address
const byte MAX31790 = 0x23;
byte data, data0, data1;

#define INIT_INTERVAL_SECONDS \
  150  // initial interval between 1st pulse and next pulse (longer for
       // distribution of aerosol)
#define REG_INTERVAL_SECONDS \
  90  // regulation interval between each pulse after the 1st to maintain pm2.5
      // concentration
#define PMS_READ_INTERVAL_SECONDS 1
const unsigned int TARGET_PM02 =
    40;  // target pm2.5 concentration in micrograms per cubic meter

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
  Wire.requestFrom(ADDRESS, 1);  // read 1 byte
  byte x = Wire.read();
  return x;
}

// Function to calculate the weighted average
float calculateWeightedAverage(float values[], float weights[],
                               int numSensors) {
  float weightedSum = 0.0;
  float totalWeight = 0.0;

  // Calculate the total weighted sum and total weight
  for (int i = 0; i < numSensors; i++) {
    weightedSum += values[i] * weights[i];
    totalWeight += weights[i];
  }

  // Return the weighted average
  if (totalWeight == 0) {
    return 0;  // Prevent division by zero
  } else {
    return weightedSum / totalWeight;
  }
}

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

  // Reset MAX31790 (POR)
  set_I2C_register(MAX31790, 0x00, B01010000);  // reset
  delay(100);

  // PWM Frequency       first 4 bits 0111 (0-10V), next 4 bits 1011 (PWM)
  set_I2C_register(MAX31790, 0x01, B01101010);  // 1.25kHz
  delay(1);

  // Configuration
  data = B00001000;  // PWM mode - 'Spin-up' OFF - control ON - Tach count- PWM
                     // yeah
  for (int i = 0x02; i <= 0x07; i++) {
    set_I2C_register(MAX31790, i, data);
    delay(1);
  }

  // Pin for monitoring loop
  pinMode(Trig1, OUTPUT);
  digitalWrite(Trig1, LOW);
  MyLog::debug("Setup Done");

  espNowMaster.begin();

  local_pms.passiveMode();
  local_pms.wakeUp();

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    MyLog::error("Error initializing ESP-NOW");
    return;
  }

  // Initialize NTP client
  timeClient.begin();
  timeClient.update();

  xTaskCreatePinnedToCore(Fan_controller, "Fan_controller_task", 4096, NULL, 1,
                          &Fan_controller_Handle, 1);

#ifdef USE_ESP_NOW
  xTaskCreatePinnedToCore(Data_acquisition, "Data_acquisition_task", 4096, NULL,
                          1, &Data_acquisition_Handle, 1);
#endif

#ifndef USE_ESP_NOW
  xTaskCreatePinnedToCore(Http_request, "Http_request_task", 4096, NULL, 1,
                          &Http_request_Handle, 0);
#endif
}

int MultiplicationCombine(unsigned int x_high, unsigned int x_low) {
  int combined;
  combined = x_high;
  combined = combined * 256;
  combined |= x_low;
  return combined;
}

void loop() {
  // Empty loop as tasks are being handled separately
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
  bool localPmsDataValid = false;

  uint16_t oldLocalPM25;
  uint16_t newLocalPM25;

  while (true) {
    // Setting interval length based on time
    interval_s = millis() - bootTime > INIT_INTERVAL_SECONDS * 1000
                     ? REG_INTERVAL_SECONDS
                     : INIT_INTERVAL_SECONDS;

    // Read smth
    data0 = get_I2C_register(MAX31790, 0x22);  // Tach1 count
    data1 = get_I2C_register(MAX31790, 0x23);

    uint16_t tach_out = ((((int16_t)data0) << 8) | data1) >> 5;

    // convert to RPM
    int tach1 = 60 * _SR * 8192 / (NP * tach_out);

    PMS::DATA localPmsData;
    if (millis() > pmsReadTs + (PMS_READ_INTERVAL_SECONDS * 1000)) {
      pmsReadTs = millis();
      while (Serial1.available()) {
        Serial1.read();
      }
      local_pms.requestRead();
      localPmsDataValid = local_pms.readUntil(localPmsData);
      printLocalPM(localPmsDataValid, localPmsData);
    }

    //////////////////////////////////////////////////// print
    if (millis() > pre2 + 2000) {
      pre2 = millis();
      digitalWrite(Trig1, HIGH);
      MyLog::info("Average PM2.5: %.2f", meanpm02);
      MyLog::info("pm2.5_target = %u", TARGET_PM02);
      MyLog::info("Fan is running: \t\t\t%s", fanIsOn ? "true" : "false");
      MyLog::info("LocalPmsDataValid is: %s",
                  localPmsDataValid ? "true" : "false");
      MyLog::info("Fan speed is: %d", fanSpeedPercent);
    }
    unsigned long now = millis();

    if (WiFi.status() == WL_CONNECTED) {
      if (meanpm02 < TARGET_PM02 && localPmsDataValid) {
        fanSpeedPercent = Calculator::getFanRunSpeed(meanpm02, TARGET_PM02);
        duration = Calculator::getFanRunningIntervalV2(meanpm02, TARGET_PM02,
                                                       fanSpeedPercent);

        // double requiredRPM = Calculator::determineFanRPMToAchieveTarget(
        //     meanpm02, TARGET_PM02, localPmsData.PM_AE_UG_2_5);

        // fanSpeedPercent = Calculator::convertRPMToPercentage(requiredRPM);

        fanIsOn = true;
        intervalTs = now;
        durationTs = millis();
        uint16_t pwm_bit = Calculator::scaleDutyCycle(fanSpeedPercent);

        pwm_bit = pwm_bit << 7;
        byte pwm0 = pwm_bit >> 8;
        byte pwm1 = pwm_bit;
        set_I2C_register(MAX31790, 0x40, pwm0);  // Channel 1 --> Fan
        set_I2C_register(MAX31790, 0x41, pwm1);
      } else {
        fanIsOn = false;
        set_I2C_register(MAX31790, 0x40, 0);
        set_I2C_register(MAX31790, 0x41, 0);
      }
      newLocalPM25 = localPmsData.PM_AE_UG_2_5;
      // if close sensor find reach the target will stop the fan
      if (localPmsDataValid && localPmsData.PM_AE_UG_2_5 >= 300 &&
          oldLocalPM25 < newLocalPM25) {
        fanIsOn = false;
        set_I2C_register(MAX31790, 0x40, 0);
        set_I2C_register(MAX31790, 0x41, 0);
        oldLocalPM25 = newLocalPM25;
      } 
      // else if (localPmsDataValid && localPmsData.PM_AE_UG_2_5 <= 500 &&
      //            oldLocalPM25 > newLocalPM25 && meanpm02 < TARGET_PM02) {
      //   fanIsOn = true;
      //   uint16_t pwm_bit = Calculator::scaleDutyCycle(fanSpeedPercent);

      //   pwm_bit = pwm_bit << 7;
      //   byte pwm0 = pwm_bit >> 8;
      //   byte pwm1 = pwm_bit;
      //   set_I2C_register(MAX31790, 0x40, pwm0);  // Channel 1 --> Fan
      //   set_I2C_register(MAX31790, 0x41, pwm1);
      //   oldLocalPM25 = newLocalPM25;
      // }
    }
    vTaskDelay(xDelay100m);
  }
}

void Data_acquisition(void *parameter) {
  // Update NTP time
  timeClient.update();
  unsigned long currentNtpTime = timeClient.getEpochTime();
  while (true) {
    numSensors = 0;  // Reset sensor count for each loop
    unsigned long currentNtpTime = timeClient.getEpochTime();
    for (int i = 0; i < 128; i++) {
      if (espNowMaster.getBoardId(i, boardId)) {
        if (espNowMaster.getData(boardId, pm25, temp, humi, timestamp)) {
          // Check if data is newer than 5 seconds using NTP timestamp
          if ((currentNtpTime - timestamp) <= 5) {
            pmValues[numSensors] = pm25;
            weights[numSensors] =
                1.0;  // Set weight to 1.0 for equal weighting, can be changed
            numSensors++;
          }
          MyLog::info("Board ID: %llX", boardId);
          MyLog::info("PM2.5: %.2f", pm25);
          MyLog::info("Temperature: %.2f", temp);
          MyLog::info("Humidity: %.2f", humi);
          MyLog::info("Last Updated: %lu", timestamp);
        }
      }
    }

    // Calculate the weighted average of PM2.5 values
    if (numSensors > 0) {
      meanpm02 = calculateWeightedAverage(pmValues, weights, numSensors);
      if (millis() > pre2_11 + 2000) {
        pre2_11 = millis();
        MyLog::info("Weighted Average PM2.5: %.2f", meanpm02);
      }
    } else {
      MyLog::warn("No data available to calculate weighted average.");
    }

    // Check for timeout of boards every 30 seconds
    espNowMaster.checkBoardTimeout();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void Http_request(void *parameter) {
  numSensors = 6;
  while (true) {
    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient https;
      https.begin(apiUrl);  // Specify the URL
      // Perform the GET request
      int httpCode = https.GET();
      MyLog::info("%s%s",
                  String("Get data from server with response code: ").c_str(),
                  String(httpCode).c_str());
      delay(1000);  // TODO: slow down the server polling
      if (httpCode > 0) {
        // Check if the GET request was successful
        if (httpCode == HTTP_CODE_OK) {
          String payload = https.getString();

          // Parse the JSON response
          DynamicJsonDocument doc(49152);
          DeserializationError error = deserializeJson(doc, payload);

          if (error) {
            https.end();  // Close connection
            MyLog::info("%s", String("deserializeJson() failed: ").c_str());
            MyLog::info("%s", String(error.c_str()).c_str());
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
          }

          for (int i = 0; i < 5; i++) {
            ref[i] = float(doc[i]["pm02"]);
            Serial.printf("Ref #%d: %.2f\n", i, ref[i]);
            pmValues[i] = doc[i]["pm02"];
          }
          Serial.printf("Ref #11: %.2f\n", float(doc[12]["pm02"]));
          pmValues[5] = doc[12]["pm02"];

          meanpm02 = calculateWeightedAverage(pmValues, weights, numSensors);

          MyLog::info("%s", String("Mean pm02: ").c_str());
          MyLog::info("%s", String(meanpm02).c_str());
        }
      } else {
        https.end();  // Close connection
        Serial.printf("GET request failed, error: %s\n",
                      https.errorToString(httpCode).c_str());
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }
      https.end();  // Close connection
    }
    vTaskDelay(pdMS_TO_TICKS(1000));  // Delay before the next request
  }
}

void localPMControl(int *localPM) {}
