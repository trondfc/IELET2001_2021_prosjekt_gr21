/*
   Code for IoT prodject in IELET2002 2021

   node mesuring water level of a plant, as well as ligt and temperature and sending it to IoT

   uses:
      esp32
      bme280
      capasative soil moisture
      LDR
      lipo batery

   To be done:
      integrate all sensors in code     X
      fix OTA button and reset button   X
      OTA duble click reset?            X
      led flash on OTA                  X
      integrate power transistor        X
      messure power consumtion
      integrate warning led             X
      Test with sensors                 X
      ubidots event mesages
      make circuit
      design 3D moddel
      print 3D moddel
      calibrate mesurments

*/
#include <WiFi.h>               // libary for wifi
#include <ESPmDNS.h>            // libary for wifi
#include <WiFiUdp.h>            // libary for wifi
#include <ArduinoOTA.h>         // libary for Over The Air updates
#include <Adafruit_Sensor.h>    // libary for bme sensor
#include <Adafruit_BME280.h>    // libary for bme sensor
#include <Wire.h>               // libary for i2c comunication
#include "UbidotsEsp32Mqtt.h"   // libary for ubidots mqtt
#include <EEPROM.h>             // libary for EEPROM storage

#define UBIDOTS_TOKEN "BBFF-3YgdHdFOLx4ZLhV3WdXJwOs1izpWXL"   // Put here your Ubidots TOKEN
#define WIFI_SSID "Get-D62F31"                                // Put here your Wi-Fi SSID
#define WIFI_PASS "HRNKLK8LLJ"                                // Put here your Wi-Fi password
#define DEVICE_LABEL "test_device"                            // Put here your Device label to which data  will be published

#define TEMP_LABEL "Temp"                             // Ubidots label for temperature
#define HUMIDITY_LABEL "Humidity"                     // Ubidots label for air humidity
#define LIGHT_LABEL "Light"                           // Ubidots label for light level
#define BATERY_LABEL "Batery"                         // Ubidots label for batery voltage
#define BATERY_PERCENT_LABEL "Batery_percent"         // Ubidots label for batery percentage
#define MOISTURE_LABEL "Moisture"                     // Ubidots label for soil moisture
#define MOISTURE_WARNING_LABEL "Moisture_warning"     // Ubidots label for moisture warning

#define uS_TO_S_FACTOR 1000000ULL   // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP  300           // Time ESP32 will go to sleep (in seconds)

#define EEPROM_SIZE 1               // Number of bits stored in EEPROM
#define OTA_TIME 60000              // Time OTA checker runs (in uS)

#define AirValue 2627               // value for soil moisture calibration
#define WaterValue 1285             // value for soil moisture calibration
#define LIGHT_MIN 0                 // value for light level calibration
#define LIGHT_MAX 4095              // value for light level calibration

#define BATERY_CONST_1 114.8            // constant for batery persentage formula
#define BATERY_CONST_2 98248312502245   // constant for batery persentage formula
#define BATERY_CONST_3 -8.08            // constant for batery persentage formula

const int soilPin = 32;         // Pin used to read data from soil moisture
const int lightPin = 33;        // Pin used to read data from LDR
const int batteryPin = 34;      // Pin used to read voltage of batery
const int ledPin = 26;          // Pin used to control warning led
const int transistorPin = 19;   // Pin used to control power transistor

Adafruit_BME280 bme;              // Bme instance
Ubidots ubidots(UBIDOTS_TOKEN);   // Ubidots instance

void callback(char *topic, byte *payload, unsigned int length)    //message from ubidots
{
  Serial.print("Message arrived [");      // Serial info
  Serial.print(topic);                    // Serial info
  Serial.print("] ");                     // Serial info
  bool led_state = false;                 // create boolean state
  for (int i = 0; i < length; i++)        // loop through message
  {
    Serial.print((char)payload[i]);       // Serial info
    if ((char)payload[0] == '1')          // if value = 1
    {
      led_state = true;                     // set boolean state
    }
    else                                  // if value = 0
    {
      led_state = false;                    // set boolean state
    } 
  }
  Serial.println("");                     // Serial info
  Serial.println(led_state);              // Serial info
  digitalWrite(ledPin, led_state);        // Control led
}

void setup() {
  pinMode(ledPin, OUTPUT);                                          // Set pin Mode
  pinMode(transistorPin, OUTPUT);                                   // Set pin Mode
  Serial.begin(115200);                                             // Start serial comunication
  EEPROM.begin(EEPROM_SIZE);                                        // Start EEPROM storage
  int Flag = EEPROM.read(0);                                        // Read EEPROM storage
  if (Flag) {                                                       // Test reading from EEPROM
    Serial.println("OTA Flag are high, going to OTA");                // Serial info
    handle_OTA();                                                     // Call handle_OTA function
  }
  EEPROM.write(0, 1);                                               // Write EEPROM storage
  EEPROM.commit();                                                  // Write EEPROM storage
  delay(1000);                                                      // Wait for 1s. reseting during this makes esp go into OTA mode
  EEPROM.write(0, 0);                                               // Write EEPROM storage
  EEPROM.commit();                                                  // Write EEPROM storage
  Serial.println("Booting");                                        // Serial info
  Serial.println("Setup ESP32 to sleep for " +                      // Serial info
                 String(TIME_TO_SLEEP) + " Seconds");               // Serial info

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);    // Enable deepsleep

  digitalWrite(transistorPin, HIGH);                                // Turn sensors on

  WiFi.mode(WIFI_STA);                                              // WiFi mode
  WiFi.begin(WIFI_SSID, WIFI_PASS);                                 // Start WiFi
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {             // Reboot if failed
    Serial.println("WiFi connection Failed! Rebooting...");           // Serial info
    delay(2000);                                                      // Wait for 2s      
    ESP.restart();                                                    // Reboot esp
  }
  if (! bme.begin(0x76, &Wire)) {                                   // Start bme, reebot if failed
    Serial.println("bme280 connection Failed! Rebooting...");         // Serial info
    delay(2000);                                                      // Wait for 2s 
    ESP.restart();                                                    // Reboot esp
  }
  gpio_hold_dis((gpio_num_t)26);                                    // Disable gpio holding
  //ubidots.setDebug(true);
  ubidots.setCallback(callback);                                    // Ubidots setup
  ubidots.setup();                                                  // Ubidots setup
  ubidots.reconnect();                                              // Ubidots conection
  ubidots.subscribeLastValue(DEVICE_LABEL, MOISTURE_WARNING_LABEL); // Ubidots subscribe to moisture warning

  publish_to_label(TEMP_LABEL, get_air_temp());                     // Publish value to ubidots
  publish_to_label(HUMIDITY_LABEL, get_air_humidity());             // Publish value to ubidots
  publish_to_label(LIGHT_LABEL, get_light_level());                 // Publish value to ubidots
  publish_to_label(BATERY_LABEL, get_battery_voltage());            // Publish value to ubidots
  publish_to_label(MOISTURE_LABEL, get_soil_moisture());            // Publish value to ubidots
  publish_to_label(BATERY_PERCENT_LABEL, get_battery_persentage()); // Publish value to ubidots
  ubidots.publish(DEVICE_LABEL);                                    // Publish values to ubidots
  for (int i = 0; i < 20; i++) {                                    // Loop 20 times
    ubidots.loop();                                                   // Try to get Ubidots updates
    delay(100);                                                       // Wait for 200 mS
  }
  ubidots.disconnect();                                             // disconect from ubidots 
  gpio_hold_en((gpio_num_t)26);                                     // Enable gpio holding. Nesesary for led to stay high during deepsleep
  Serial.println("done, going to sleep");                           // Serial info
  digitalWrite(transistorPin, LOW);                                 // turn off sensors
  esp_deep_sleep_start();                                           // Send to deepsleep
}

void loop() {
}

void handle_OTA() {                                       // Handle OTA function
  bool led_state = true;                                  // Create boolean state
  digitalWrite(ledPin, led_state);                        // Control led
  EEPROM.write(0, 0);                                     // Write EEPROM storage
  EEPROM.commit();                                        // Write EEPROM storage
  Serial.println("in OTA mode");                          // Serial info
  WiFi.mode(WIFI_STA);                                    // Set WiFi mode
  WiFi.begin(WIFI_SSID, WIFI_PASS);                       // Start WiFi conection
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {   // Wait for conection
    Serial.println("Connection Failed! Rebooting...");      // Serial info
    delay(2000);                                            // Wait for 2s 
    ESP.restart();                                          // Reboot esp
  }
  ArduinoOTA.setHostname("myesp32");                      // Set OTA lable

  ArduinoOTA                                              // Configure OTA
  .onStart([]() {                                         // Configure OTA
    String type;                                          // Configure OTA
    if (ArduinoOTA.getCommand() == U_FLASH)               // Configure OTA
      type = "sketch";                                    // Configure OTA
    else // U_SPIFFS                                      // Configure OTA
      type = "filesystem";                                // Configure OTA

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);             // Serial info
  })
  .onEnd([]() {                                                               // Configure OTA
    Serial.println("\nEnd");                                                  // Configure OTA
  })                                                                          // Configure OTA
  .onProgress([](unsigned int progress, unsigned int total) {                 // Configure OTA
    Serial.printf("Progress: %u%%\r\n", (progress / (total / 100)));          // Serial info
  })
  .onError([](ota_error_t error) {                                            // Configure OTA
    Serial.printf("Error[%u]: ", error);                                      // Configure OTA
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");               // Configure OTA
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");        // Configure OTA
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");    // Configure OTA
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");    // Configure OTA
    else if (error == OTA_END_ERROR) Serial.println("End Failed");            // Configure OTA
  });

  ArduinoOTA.begin();                         // Start OTA

  Serial.println("Ready");                    // Serial info
  Serial.print("IP address: ");               // Serial info
  Serial.println(WiFi.localIP());             // Serial info
  unsigned long startTime = millis();         // Set start time to curent time
  while (millis() - startTime < OTA_TIME) {   // Run for predefined time
    led_state = !led_state;                   // Toggle boolean state
    digitalWrite(ledPin, led_state);          // Control led
    delay(500);                               // Wait for 0.5s
    ArduinoOTA.handle();                      // Try OTA conection
  }
  digitalWrite(ledPin, LOW);                  // Control led
  ESP.restart();                              // Reboot esp
}

void publish_to_label(char* VARIABLE_LABEL, float value_to_publish) {       // Create function
  Serial.println("sending " + String(value_to_publish) +                    // Serial info
                 " to " + String(VARIABLE_LABEL));                          // Serial info
  ubidots.add(VARIABLE_LABEL, value_to_publish);                            // Publish value to ubidots
}

int get_random() {                        // Create function
  int r_int = random(-100, 100);          // Get random number
  return r_int;                           // Return number
}

float get_soil_moisture() {                                                 // Create function
  float MoistureVal = analogRead(soilPin);                                  // Read value
  float MoisturePercent = map(MoistureVal, AirValue, WaterValue, 0, 100);   // Calculate percentage
  if (MoisturePercent > 100) {                                              // Clamp persentage
    MoisturePercent = 100;
  }
  if (MoisturePercent < 0) {                                                // Clamp persentage
    MoisturePercent = 0;
  }
  return MoisturePercent;                                                   // Return persentage
}

float get_air_temp() {                    // Create function
  float temp = bme.readTemperature();     // Read value
  if ( temp > 90) {                       // Test if value unreasonable
    temp = get_air_temp();                // Run function again
  }
  return temp;                            // Return value
}

float get_air_humidity() {                  // Create function
  float humidity = bme.readHumidity();      // Read value
  if ( humidity > 90) {                     // Test if value unreasonable
    humidity = get_air_humidity();          // Run function again
  }
  return humidity;                          // Return value
}

float get_light_level() {                                       // Create function
  float raw = analogRead(lightPin);                             // Read value
  float LightPercent = map(raw, LIGHT_MIN, LIGHT_MAX, 0, 100);  // Calculate percentage
  if (LightPercent > 100) {                                     // Clamp persentage
    LightPercent = 100;
  }
  if (LightPercent < 0) {                                       // Clamp persentage
    LightPercent = 0;
  }
  return LightPercent;                                          // Return persentage
}

float get_battery_voltage() {                   // Create function
  float raw = analogRead(batteryPin);           // Read value
  float voltage = raw * 2.0 * (3.3 / 4095.0);   // Calculate voltage
  return voltage;                               // Return voltage
}

float get_battery_persentage() {                                                                    // Create function
  float batery = get_battery_voltage();                                                             // Get batery voltage
  float BateryPercent = (BATERY_CONST_1 / ( 1 + BATERY_CONST_2 * exp(BATERY_CONST_3 * batery)));    // Calculate batery persentage
  return BateryPercent;                                                                             // Return batery persentage
}
