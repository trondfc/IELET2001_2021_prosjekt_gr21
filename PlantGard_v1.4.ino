/*
   Code for IoT prodject in IELET2002 2021

   node mesuring water level of a plant, as well as ligt and temperature and sending it to IoT

   uses:
      esp-01
      bme280
      capasative soil moisture
      asd1015
      LDR
      lipo batery

   To be done:
      integrate all sensors in code     X
      fix OTA button and reset button   X
      OTA duble click reset?            X
      led flash on OTA                  X
      integrate power transistor        X
      messure power consumtion          X   80mA + 0.35mA
      integrate warning led             X
      Test with sensors                 X
      move to esp-01                    X
        deepsleep rutine for led        X
      ubidots event mesages
      make circuit
      design 3D moddel
      print 3D moddel
      calibrate mesurments

*/

#include <Adafruit_Sensor.h>    // libary for i2c sensors
#include <Adafruit_BME280.h>    // libary for bme280 sensor
#include <Adafruit_ADS1X15.h>   // libary for ads1015 sensor
#include <Wire.h>               // libary for i2c comunication
#include "UbidotsESPMQTT.h"     // libary for ubidots
#include <ESP8266WiFi.h>        // libary for wifi
#include <ESP8266mDNS.h>        // libary for wifi
#include <WiFiUdp.h>            // libary for wifi
#include <ArduinoOTA.h>         // libary for OTA
#include <EEPROM.h>             // libary for EEPROM storage

#define TOKEN "BBFF-3YgdHdFOLx4ZLhV3WdXJwOs1izpWXL"   // Your Ubidots TOKEN
#define WIFINAME "Foss"                         // Your SSID
#define WIFIPASS "7N437RJGDH"                         // Your Wifi Pass
#define DEVICE_LABEL "PlantGuard"                     // ubidots device lable

#define TEMP_LABEL "Temp"                             // Ubidots label for temperature
#define HUMIDITY_LABEL "Humidity"                     // Ubidots label for air humidity
#define LIGHT_LABEL "Light"                           // Ubidots label for light level
#define BATERY_LABEL "Batery"                         // Ubidots label for batery voltage
#define BATERY_PERCENT_LABEL "Batery_percent"         // Ubidots label for batery percentage
#define MOISTURE_LABEL "Moisture"                     // Ubidots label for soil moisture
#define MOISTURE_WARNING_LABEL "Moisture_warning"     // Ubidots label for moisture warning

#define uS_TO_S_FACTOR 1000000ULL     // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP  1800            // Time ESP32 will go to sleep (in seconds)
#define WARNING_TIME_TO_SLEEP 60      // Time betwen blinks in warning mode (in seconds)
#define OTA_TIME 60000                // Time OTA checker runs (in mS)

#define EEPROM_SIZE 512               // Number of bits stored in EEPROM
#define OTA_FLAG_ADDRESS 0            // EEPROM address for OTA flag
#define WATER_FLAG_ADDRESS 2          // EEPROM address for water flag
#define WATER_WARNING_ADDRESS 4       // EEPROM address for water warning counter

#define BATERY_CONST_1 114.8            // constant for batery persentage formula
#define BATERY_CONST_2 98248312502245   // constant for batery persentage formula
#define BATERY_CONST_3 -8.08            // constant for batery persentage formula

#define AirValue 1251               // value for soil moisture calibration
#define WaterValue 2500             // value for soil moisture calibration
#define LIGHT_MIN 0                 // value for light level calibration
#define LIGHT_MAX 1200              // value for light level calibration
#define VOLTAGE_OFSET 0.4


#define led 1             // GIPO for LED
#define transistor 3      // GPIO for power transistor


Ubidots client(TOKEN);    // start ubidots instance
Adafruit_ADS1015 ads;     // start ads1015 instance
Adafruit_BME280 bme;      // start bme280 instance

void callback(char* topic, byte* payload, unsigned int length) {    // When receving value from ubidots
  bool led_state = false;                                             // Create boolean state
  for (int i = 0; i < length; i++) {                                  // Loop trough message
    if ((char)payload[0] == '1')                                      // if value = 1
    {
      led_state = true;                                                 // set boolean state
    }
    else                                                              // if value = 0
    {
      led_state = false;                                                // set boolean state
    }
  }
  EEPROM.put(WATER_FLAG_ADDRESS, led_state);                          // Store boolean state in EEPROM
  EEPROM.commit();                                                    // Store boolean state in EEPROM
  if (led_state) {                                // If boolean value is 1
    water_warning();                              // Run water_warning function
  }
}

void setup() {                                    // Run on boot
  pinMode(led, OUTPUT);                           // Set pin mode
  pinMode(transistor, OUTPUT);                    // Set pin mode
  digitalWrite(led, HIGH);                        // Turn off led (revese logic)

  Wire.begin(2, 0);                               // Start i2c
  EEPROM.begin(EEPROM_SIZE);                      // Start EEPROM
  bool flag;                                      // Create boolean
  EEPROM.get(OTA_FLAG_ADDRESS, flag);             // Set boolean to value from EEPROM
  if (flag) {                                     // If boolean value is 1
    handle_OTA();                                   // Run handle_OTA function
  }
  EEPROM.put(OTA_FLAG_ADDRESS, true);             // Store true in EEPROM
  EEPROM.commit();                                // Store true in EEPROM
  delay(1000);                                    // Wait for 1s
  EEPROM.put(OTA_FLAG_ADDRESS, false);            // Store false in EEPROM
  EEPROM.commit();                                // Store false in EEPROM
  bool water_flag;                                // Create boolean
  EEPROM.get(WATER_FLAG_ADDRESS, water_flag);     // Set boolean to value from EEPROM
  if (water_flag) {                               // If boolean value is 1
    water_warning();                              // Run water_warning function
  }

  digitalWrite(transistor, HIGH);                                 // Turn sensors on
  delay(100);

  WiFi.mode(WIFI_STA);                                            // Set wifi mode
  WiFi.begin(WIFINAME, WIFIPASS);                                 // Start wifi
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {           // try to conect to wifi
    digitalWrite(led, LOW);                                         // Turn led on
    delay(500);                                                     // wait for 0.5s
    ESP.restart();                                                  // restart esp
  }
  client.begin(callback);                                         // Configure ubidots
  client.reconnect();                                             // Conect to ubidots
  client.ubidotsSubscribe(DEVICE_LABEL, MOISTURE_WARNING_LABEL);  // Subscribe to label from ubidots

  if (!ads.begin()) {                  // Try to start ADS
    bool state = false;                 // Create boolean
    for (int i = 0; i < 50; i++) {
      state = !state;                     // Togle boolean
      digitalWrite(led, state);           // Control led
      delay(100);                         // Wait
    }
    ESP.restart();
  }
  if (! bme.begin(0x76, &Wire)) {     // Try to start ADS
    bool state = false;                 // Create boolean
    for (int i = 0; i < 50; i++) {
      state = !state;                     // Togle boolean
      digitalWrite(led, state);           // Control led
      delay(500);                         // Wait
    }
    ESP.restart();
  }

  client.add(TEMP_LABEL, get_air_temp());                         // Publish value to ubidots
  client.add(HUMIDITY_LABEL, get_air_humidity());                 // Publish value to ubidots
  client.add(LIGHT_LABEL, get_light_level());                     // Publish value to ubidots
  client.add(BATERY_LABEL, get_battery_voltage());                // Publish value to ubidots
  client.add(MOISTURE_LABEL, get_soil_moisture());                // Publish value to ubidots
  client.add(BATERY_PERCENT_LABEL, get_battery_persentage());     // Publish value to ubidots
  client.ubidotsPublish(DEVICE_LABEL);                            // Publish value to ubidots
  for (int i = 0; i < 10; i++) {                                    // Loop 10 times
    client.loop();                                                   // Try to get Ubidots updates
    delay(100);                                                       // Wait for 100 mS
  }
  digitalWrite(transistor, LOW);                                  // Turn off sensors
  EEPROM.end();                                                   // Turn off EEPROM
  ESP.deepSleep(TIME_TO_SLEEP * uS_TO_S_FACTOR);                  // Start deepsleep
}

void loop() {
}

void water_warning() {                                        // Create function
  bool led_state = true;                                      // Create boolean
  for (int i = 0; i < 4; i++) {                               // Loop 6 times
    led_state = !led_state;                                     // Toggle boolean
    digitalWrite(led, led_state);                               // controll led
    delay(300);                                                 // wait
  }
  int warning_loops;                                          // Create integer
  EEPROM.get(WATER_WARNING_ADDRESS, warning_loops);           // Set integer to value from EEPROM
  if (warning_loops <= 30) {                                  // If integer >= 30
    warning_loops++;                                            // Increase integer
    EEPROM.put(WATER_WARNING_ADDRESS, warning_loops);           // Store integer in EEPROM
    EEPROM.commit();                                            // Store integer in EEPROM
  }
  else {                                                      // If integer < 30
    EEPROM.put(WATER_WARNING_ADDRESS, 0);                       // Store 0 as loop counter in EEPROM
    EEPROM.commit();                                            // Store in EEPROM
    EEPROM.put(WATER_FLAG_ADDRESS, false);                      // Store false as water flag in EEPROM
    EEPROM.commit();                                            // Store in EEPROM
    delay(100);                                                 // Wait
  }
  EEPROM.end();                                               // Close EEPROM
  ESP.deepSleep(WARNING_TIME_TO_SLEEP * uS_TO_S_FACTOR);      // Start deepsleep
}

void handle_OTA() {                                     // Handle OTA function
  bool led_state = false;                               // Create boolean state
  digitalWrite(led, led_state);                         // Control led
  WiFi.mode(WIFI_STA);                                  // Set WiFi mode
  WiFi.begin(WIFINAME, WIFIPASS);                       // Start WiFi conection
  while (WiFi.waitForConnectResult() != WL_CONNECTED) { // Wait for conection
    digitalWrite(led, LOW);                               // Control led
    delay(500);                                           // Wait
    digitalWrite(led, HIGH);                              // Control led
    ESP.restart();                                        // Reboot esp
  }

  EEPROM.put(OTA_FLAG_ADDRESS, false);                    // Store boolean in EEPROM
  EEPROM.commit();                                        // Store in EEPROM
  EEPROM.end();                                           // Close EEPROM

  ArduinoOTA.setHostname("PlantGuard");                   // Set OTA lable

  ArduinoOTA.onStart([]() {                               // Configure OTA
    String type;                                          // Configure OTA
    if (ArduinoOTA.getCommand() == U_FLASH) {             // Configure OTA
      type = "sketch";                                    // Configure OTA
    } else { // U_FS                                      // Configure OTA
      type = "filesystem";                                // Configure OTA
    }
  });
  ArduinoOTA.onEnd([]() {                                                 // Configure OTA
    Serial.println("\nEnd");                                              // Configure OTA
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {   // Configure OTA
    Serial.printf("Progress: %u%%\r\n", (progress / (total / 100)));      // Configure OTA
  });
  ArduinoOTA.onError([](ota_error_t error) {        // Configure OTA
    Serial.printf("Error[%u]: ", error);            // Configure OTA
    if (error == OTA_AUTH_ERROR) {                  // Configure OTA
      Serial.println("Auth Failed");                // Configure OTA
    } else if (error == OTA_BEGIN_ERROR) {          // Configure OTA
      Serial.println("Begin Failed");               // Configure OTA
    } else if (error == OTA_CONNECT_ERROR) {        // Configure OTA
      Serial.println("Connect Failed");             // Configure OTA
    } else if (error == OTA_RECEIVE_ERROR) {        // Configure OTA
      Serial.println("Receive Failed");             // Configure OTA
    } else if (error == OTA_END_ERROR) {            // Configure OTA
      Serial.println("End Failed");                 // Configure OTA
    }
  });
  ArduinoOTA.begin();                         // Start OTA
  unsigned long startTime = millis();         // Set start time to curent time
  while (millis() - startTime < OTA_TIME) {   // Run for predefined time
    led_state = !led_state;                     // Toggle boolean state
    digitalWrite(led, led_state);               // Control led
    delay(500);                                 // Wait for 0.5s
    ArduinoOTA.handle();                        // Try OTA conection
  }
  digitalWrite(led, HIGH);                    // Control led
  ESP.restart();                              // Reboot esp
}

float get_air_temp() {                    // Create function
  float temp = bme.readTemperature();     // Read value
  if ( temp > 90 || temp < 5) {                       // Test if value unreasonable
    temp = get_air_temp();                // Run function again
  }
  return temp;                            // Return value

}
float get_air_humidity() {                  // Create function
  float humidity = bme.readHumidity();      // Read value
  if ( humidity > 90 || humidity < 5) {                     // Test if value unreasonable
    humidity = get_air_humidity();          // Run function again
  }
  return humidity;                          // Return value

}

float get_soil_moisture() {                                         // Create function
  float total = 0;
  for (int i = 0; i < 5; i++) {
    int16_t adc2 = ads.readADC_SingleEnded(2);                        // Read analog value
//    float MoisturePercent = map(adc2, AirValue, WaterValue, 0, 100);  // Calculate percentage
    //if (MoisturePercent > 100) {                                    // Clamp persentage
    //  MoisturePercent = 100;
    //}
    //if (MoisturePercent < 0) {                                      // Clamp persentage
    //  MoisturePercent = 0;
    // }
//    total += MoisturePercent;
    total += adc2;
  }
  float avg = total / 5;
  return avg;                                           // Return persentage
}

float get_light_level() {                                       // Create function
  float total = 0;
  for (int i = 0; i < 5; i++) {
    int16_t adc1 = ads.readADC_SingleEnded(1);                    // Read analog value
    float LightPercent = map(adc1, LIGHT_MIN, LIGHT_MAX, 0, 100);
    if (LightPercent > 100) {                                     // Clamp persentage
      LightPercent = 100;
    }
    if (LightPercent < 0) {                                       // Clamp persentage
      LightPercent = 0;
    }
    total += LightPercent;
  }
  float avg_light = total / 5;
  return avg_light;                                          // Return persentage
}

float get_battery_voltage() {                     // Create function
  float total = 0;
  for (int i = 0; i < 5; i++) {
    int16_t adc0 = ads.readADC_SingleEnded(0);      // Read analog value
    float volts = (ads.computeVolts(adc0) + VOLTAGE_OFSET);           // Calculate voltage
    if (volts < 2) {
      volts = get_battery_voltage();
    }
    total += volts;
  }
  float avg = total / 5;
  return avg;                                   // Return voltage
}

float get_battery_persentage() {                                                                          // Create function
  float battery = get_battery_voltage();                                                                  // Get voltage
  float BatteryPercent = (BATERY_CONST_1 / ( 1 + BATERY_CONST_2 * exp(BATERY_CONST_3 * battery)));        // Calculate percentage
  return BatteryPercent;                                                                                  // Return percentage
}
