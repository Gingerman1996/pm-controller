#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>

#include "Calculator.h"
#include "PMS.h"

// Pin for monitoring loop
int Trig1 = 2;
#define _SR 4  // speed range
#define NP 2   // number of TACH pulse per revolution
// Wire and functions

// Multi-task parameter
TaskHandle_t Fan_controller_Handle = NULL;
const TickType_t xDelay100m = pdMS_TO_TICKS(100);

void Fan_controller(void* parameter);
void printLocalPM(bool localPmsDataValid, PMS::DATA localPmsData);

// Network credentials
const char* ssid = "ag-diamond_2.4GHz";
const char* password = "0505563014466";

// API URL
const char* apiUrl =
    "https://api.airgradient.com/public/api/v1/locations/measures/"
    "current?token=dadf4720-44da-4906-bd4f-9a57d6e073a4";

unsigned long bootTime =
    millis();  // gives time since startup in ms. Overflow after 50 days.
unsigned long t = millis();
// uint16_t loop_cnt = 0; //loop count
const int D = 50;              // delay between loops in ms
unsigned long intervalTs = 0;  // timestamp for controlling interval
unsigned long durationTs = 0;  // timestamp for controlling fan duration
unsigned long pre2 = 0;        // timestamp for controlling print interval
unsigned long pmsReadTs = 0;   // timestamp for controlling PMS reading interval
unsigned int fanSpeedPercent = 0;
float duration = 0;
float interval_s = 0;
bool fanIsOn = false;
float meanpm02 = 0;
float ref[12];

float pmValues[6] = {};

// Weights used for each sensor
float weights[] = {1.5, 1.5, 1.5, 1.5, 1.5, 1.0};

// Number of sensors
int numSensors = 6;


PMS local_pms(Serial1);

// ## MAX31790 ("4pin fan" controler - 6 channel) address
// Address
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
    5;  // target pm2.5 concentration in micrograms per cubic meter

// MQTT Config
const char* mqtt_server = "192.168.100.67";
const int mqtt_port = 1883;
const char* mqtt_user = "pmController_esp32";
const char* mqtt_password = "TanapaT41894";

// Create WiFi and MQTT client objects
WiFiClient espClient;
PubSubClient client(espClient);

// Define the structure for the message
typedef struct struct_message {
  char message[32];  // Message to send
  float pm25;        // PM2.5 value
} struct_message;

struct_message myData;

uint8_t peerMACAddress[] = {0x34, 0xB7, 0xDA, 0xBD, 0x94, 0xF4};

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
float calculateWeightedAverage(float values[], float weights[], int numSensors) {
  float weightedSum = 0.0;
  float totalWeight = 0.0;

  // Calculate the total weighted sum and total weight
  for (int i = 0; i < numSensors; i++) {
    weightedSum += values[i] * weights[i];
    totalWeight += weights[i];
  }

  // Return the weighted average
  if (totalWeight == 0) {
    return 0; // Prevent division by zero
  } else {
    return weightedSum / totalWeight;
  }
}

typedef struct ESPNow_message {
    uint64_t id;   // Serial number of ESP (MAC address)
    float pm25;    // PM2.5 value
    float humi;    // Humidity value
    float temp;    // Temperature value
} ESPNow_message;

// Create a struct_message called ESPNowData
ESPNow_message ESPNowData;

// Create a structure to hold the readings from each board
ESPNow_message boardsStruct[128];

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);

  // Copy the incoming data into ESPNowData
  memcpy(&ESPNowData, incomingData, sizeof(ESPNowData));
  Serial.printf("Board ID (dec): %u, Board ID (hex): %X: %u bytes\n", ESPNowData.id, ESPNowData.id, len);

  bool found = false;

  // Loop through the array to find an empty slot or a matching ID
  for (int i = 0; i < 128; i++) {
    if (boardsStruct[i].id == 0) {
      // Empty slot, add new board data here
      boardsStruct[i] = ESPNowData;
      Serial.printf("New board added at index %d with ID (hex): %X\n", i, ESPNowData.id);
      found = true;
      break;
    } else if (boardsStruct[i].id == ESPNowData.id) {
      // ID matches, update existing board data
      boardsStruct[i].pm25 = ESPNowData.pm25;
      boardsStruct[i].temp = ESPNowData.temp;
      boardsStruct[i].humi = ESPNowData.humi;
      Serial.printf("Updated board at index %d with ID (hex): %X\n", i, ESPNowData.id);
      found = true;
      break;
    }
  }

  if (!found) {
    // No empty slot and no matching ID found
    Serial.println("No available slot or matching ID to store data.");
  }

  // Print the values of the updated/added board
  for (int i = 0; i < 3; i++) {
    if (boardsStruct[i].id != 0) {
      Serial.printf("Index %d - Board ID (hex): %X\n", i, boardsStruct[i].id);
      Serial.printf("PM2.5: %.2f\n", boardsStruct[i].pm25);
      Serial.printf("Temp: %.2f\n", boardsStruct[i].temp);
      Serial.printf("Humi: %.2f\n", boardsStruct[i].humi);
      Serial.println();
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 26, 25);
  delay(1000);
  while (!Serial);  // wait for serial monitor to be monitoring ^^

  // Set and scan I²C bus
  Wire.begin();
  pinMode(13, INPUT_PULLUP);
  // Reset MAX31790 (POR)
  set_I2C_register(MAX31790, 0x00, B01010000);  // reset
  delay(100);
  // set_I2C_register(MAX31790, 0x00, B00110000); //no I�Ctime-out

  // PWM Frequency       first 4 bits 0111 (0-10V), next 4 bits 1011 (PWM)
  set_I2C_register(MAX31790, 0x01, B01101010);  // 1.25kHz.  0x
  delay(1);

  // Configuration
  data = B00001000;  // PWM mode - 'Spin-up' OFF - control ON - Tach count- PWM
                     // yeah
  set_I2C_register(MAX31790, 0x02, data);
  delay(1);
  set_I2C_register(MAX31790, 0x03, data);
  delay(1);
  set_I2C_register(MAX31790, 0x04, data);
  delay(1);
  set_I2C_register(MAX31790, 0x05, data);
  delay(1);
  set_I2C_register(MAX31790, 0x06, data);
  delay(1);
  set_I2C_register(MAX31790, 0x07, data);
  delay(1);

  // Pin for monitoring loop
  pinMode(Trig1, OUTPUT);
  digitalWrite(Trig1, LOW);
  Serial.println("Setup Done");

  WiFi.mode(WIFI_STA);
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  WiFi.mode(WIFI_STA);

  local_pms.passiveMode();
  local_pms.wakeUp();

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  xTaskCreatePinnedToCore(Fan_controller, "Fan_controller_task", 2048, NULL, 1,
                          &Fan_controller_Handle, 1);
}

int MultiplicationCombine(unsigned int x_high, unsigned int x_low) {
  int combined;
  combined = x_high;
  combined = combined * 256;
  combined |= x_low;
  return combined;
}

void loop() {
  unsigned long now = millis();
  bool localPmsDataValid = false;
  // Make HTTPS request
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient https;
    https.begin(apiUrl);  // Specify the URL
    // Perform the GET request
    int httpCode = https.GET();
    Serial.print("Get data from server with response code: ");
    Serial.println(httpCode);
    delay(1000);  // TODO: slow donw the sever polling
    if (httpCode > 0) {
      // Check if the GET request was successful
      if (httpCode == HTTP_CODE_OK) {
        String payload = https.getString();

        // Parse the JSON response
        DynamicJsonDocument doc(49152);
        DeserializationError error = deserializeJson(doc, payload);

        // serializeJsonPretty(doc[12], Serial);
        // Serial.println();

        if (error) {
          https.end();  // Close connection
          Serial.print("deserializeJson() failed: ");
          Serial.println(error.c_str());
          delay(10000);
          return;
        }

        for (int i = 0; i < 5; i++) {
          ref[i] = float(doc[i]["pm02"]);
          Serial.printf("Ref #%d: %.2f\n", i, ref[i]);
          pmValues[i] = doc[i]["pm02"];
        }
        Serial.printf("Ref #11: %.2f\n", float(doc[12]["pm02"]));
        pmValues[5] = doc[12]["pm02"];

        meanpm02 = calculateWeightedAverage(pmValues, weights, numSensors);
        
        Serial.print("Mean pm02: ");
        Serial.println(meanpm02);
        // setFanSpeed(10, float(doc[0]["pm02"]));
      }
    } else {
      https.end();  // Close connection
      Serial.printf("GET request failed, error: %s\n",
                    https.errorToString(httpCode).c_str());
      delay(10000);
      return;
    }
    https.end();  // Close connection

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

    //////////////////////////////////////////////////// print
    if (millis() > pre2 + 2000) {
      pre2 = millis();
      digitalWrite(Trig1, HIGH);
      Serial.println();

      // Print log
      Serial.print("Average PM2.5: ");
      Serial.println(meanpm02);

      Serial.print(" pm2.5_target=");
      // Serial.print(loop_cnt,DEC);
      Serial.print("  ");
      Serial.print(TARGET_PM02);
      Serial.print("  ");

      Serial.print("Fan is running: ");
      Serial.println(fanIsOn);

      Serial.print("Fan speed is: ");
      Serial.print(fanSpeedPercent);
      Serial.println(" %");

      // PMS::DATA localPmsData;
      // if (millis() > pmsReadTs + (PMS_READ_INTERVAL_SECONDS * 1000)) {
      //   pmsReadTs = millis();
      //   while (Serial1.available()) {
      //     Serial1.read();
      //   }
      //   Serial.println("Send read request...");
      //   local_pms.requestRead();
      //   localPmsDataValid = local_pms.readUntil(localPmsData);
      // }
      // printLocalPM(localPmsDataValid, localPmsData);
      // Serial.printf("WiFi Channel: %d\n", WiFi.channel());

      // void printLocalPM(bool localPmsDataValid, PMS::DATA localPmsData);

      // t = millis();
      // Serial.print("t[s]= ");
      // Serial.print(t / 1000.0, 3);
      // Serial.print("\t calculated pulse duration: ");
      // Serial.print(duration / 1000);
      // Serial.println(" seconds");
      // digitalWrite(Trig1, LOW);
      // Serial.print("\t curent interval[s]: ");
      // Serial.print(interval_s);
      // Serial.println(" seconds");
      // Serial.print("\t next pulse in ");
      // Serial.println(interval_s - (t / 1000 % int(interval_s)));
    }
  }
}

void printLocalPM(bool localPmsDataValid, PMS::DATA localPmsData)
{
  if (localPmsDataValid) {
        Serial.println("PMS data is valid.");
        Serial.print("Local PM 1.0 (ug/m3): ");
        Serial.println(localPmsData.PM_AE_UG_1_0);

        Serial.print("Local PM 2.5 (ug/m3): ");
        Serial.println(localPmsData.PM_AE_UG_2_5);

        Serial.print("Local PM 10.0 (ug/m3): ");
        Serial.println(localPmsData.PM_AE_UG_10_0);

        Serial.println();
      }
}

void Fan_controller(void* parameter) {
  unsigned long now = millis();
  bool localPmsDataValid = false;

  while (true) {
    if (WiFi.status() == WL_CONNECTED) {
      PMS::DATA localPmsData;
      if (millis() > pmsReadTs + (PMS_READ_INTERVAL_SECONDS * 1000)) {
        pmsReadTs = millis();
        while (Serial1.available()) {
          Serial1.read();
        }
        // Serial.println("Send read request...");
        local_pms.requestRead();
        localPmsDataValid = local_pms.readUntil(localPmsData);
        printLocalPM(localPmsDataValid, localPmsData);
      }

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
        set_I2C_register(MAX31790, 0x40, pwm0);  // Channel 1 --> Fan
        set_I2C_register(MAX31790, 0x41, pwm1);

        // Serial.print("Turn On fan: ");
        // Serial.println(millis());
        // Serial.print("PWM bit: ");
        // Serial.println(pwm_bit);
      } else {
        fanIsOn = false;
        // Serial.println("Force turn Off fan");
        set_I2C_register(MAX31790, 0x40, 0);
        set_I2C_register(MAX31790, 0x41, 0);
        // Serial.print("Mean PM2.5: ");
        // Serial.print(meanpm02);
        // Serial.println("μg/m³");
        // Serial.print("Local PMS status: ");
        // if (localPmsDataValid) {
        //   Serial.println("TRUE");
        // } else {
        //   Serial.println("FALSE");
        // }
      }

      // if close sensor find reach the target will stop the fan
      if (localPmsDataValid && localPmsData.PM_AE_UG_2_5 >= TARGET_PM02 * 8.5) {
        fanIsOn = false;
        // Serial.println("Force turn Off fan");
        set_I2C_register(MAX31790, 0x40, 0);
        set_I2C_register(MAX31790, 0x41, 0);
      }
      else if (localPmsDataValid && localPmsData.PM_AE_UG_2_5 < TARGET_PM02 * 8.5 && fanIsOn == true) {
        
      }
    }
    vTaskDelay(xDelay100m);
  }
}
