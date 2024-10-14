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
    10;  // target pm2.5 concentration in micrograms per cubic meter

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

// Callback function for receiving MQTT messages
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

// Function to connect to the MQTT broker
void reconnect() {
  // Loop until successfully connected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Connect to MQTT using username and password
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("connected");
      // Subscribe to a topic if needed
      client.subscribe("test/topic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);  // Wait 5 seconds before retrying
    }
  }
}

// Callback function when data is sent
void onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  Serial.print("Request sent: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}

// Callback function when data is received
void onDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  if (strcmp(myData.message, "FidasDataResponse") ==
      0) {  // Check if it is a response message
    Serial.println("Received response:");
    Serial.print("PM2.5: ");
    Serial.println(myData.pm25);
    Serial.println();
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

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
    return;
  }

  // Register callbacks for sending and receiving data
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  // Remove existing peer if any
  esp_now_del_peer(peerMACAddress);

  // Configure the peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, peerMACAddress,
         6);                          // Set the MAC Address of the receiver
  peerInfo.channel = WiFi.channel();  // Set to the same channel as Wi-Fi
  peerInfo.encrypt = false;           // Disable encryption

  // Add the peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
  } else {
    Serial.println("Peer added successfully");
  }

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

        meanpm02 = (float(doc[0]["pm02"]) + float(doc[1]["pm02"]) +
                    float(doc[2]["pm02"]) + float(doc[3]["pm02"]) +
                    float(doc[4]["pm02"])) /
                   5;
        for (int i = 0; i < 5; i++) {
          ref[i] = float(doc[i]["pm02"]);
          Serial.printf("Ref #%d: %.2f\n", i, ref[i]);
        }
        Serial.printf("Ref #11: %.2f\n", float(doc[12]["pm02"]));
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
      if (localPmsDataValid && localPmsData.PM_AE_UG_2_5 >= 75) {
        fanIsOn = false;
        // Serial.println("Force turn Off fan");
        set_I2C_register(MAX31790, 0x40, 0);
        set_I2C_register(MAX31790, 0x41, 0);
      }
      else if (localPmsDataValid && localPmsData.PM_AE_UG_2_5 < 75 && fanIsOn == true) {
        printLocalPM(localPmsDataValid, localPmsData);
      }
    }
    vTaskDelay(xDelay100m);
  }
}
