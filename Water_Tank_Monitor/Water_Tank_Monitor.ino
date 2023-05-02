#include "DebugPrint.h"
#include <WiFi.h>
#include <ThingSpeak.h>

// Sensor pins
#define SENSOR_VCC_PIN GPIO_NUM_12  // Pin 12 (strapping pin) can cause trouble while uploading the code, disconnect it from the sensor if needed
#define SENSOR_GND_PIN GPIO_NUM_15  // "15" is the same as "GPIO_NUM_15" (enum) with arduino functions
#define RX2 GPIO_NUM_14             // Avoid default Serial2 pins (16, 17) linked to RGB LED (16, 17 & 18)
#define TX2 GPIO_NUM_13             // TX pin is used to choose between processed value (default - HIGH) and real-time value (LOW)

// Sensor distance parameters
#define DISTANCE_THRESHOLD -1  // Minimum required distance change to upload, expressed in mm
#define DIST_MIN 150           // Expressed in mm (minimum recommended distance for the A02YYUW : 30mm)
#define DIST_MAX 1700          // Expressed in mm (maximum recommended distance for the A02YYUW : 4500mm)
#define DIST_INLET 1470        // Distance to the pump inlet (used to compute the remaining volume)
#define DIST_ERROR -1          // Value to return if distance reading failed
#define DIST_OVERSAMPLING 5

// LED parameters
#define LED_ON_TIME 80  // Expressed in ms
#define LED_R_PIN 16
#define LED_G_PIN 17
#define LED_B_PIN 18

// Water tank parameters
#define LITERS_PER_MM 3.464  // Diameter of 2m10, surface of 3.464m²
#define DISTANCE_MAX 1600    // Distance from the sensor to the pump inlet

// Deep sleep parameters
#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor from seconds to micro seconds
#define TIME_TO_SLEEP 900          // Time ESP32 will go into deepsleep (in seconds)

#define FORCED_UPDATE_FREQUENCY 86400 / TIME_TO_SLEEP  // Force to update after 24h (60*60*24) / TIME_TO_SLEEP

// Since the ADC2 module is also used by the Wi-Fi, adc2_get_raw() may get blocked until Wi-Fi stops, and vice versa.
// ADC1 pins : GPIO32 - GPIO39
// GPIO : 34 - 39 are input only (GPI)
// Only pins that support both input & output have integrated pull-up and pull-down resistors.
// Pin 32, 33 are the only one on ADC1 configurable as output
#define VOLTAGE_DIVIDER_GROUND_PIN GPIO_NUM_33  // Vbat - R2 - VOLTAGE_DIVIDER_PIN - R1 - GPIO33 and 100µF decoupling capacitor : VOLTAGE_DIVIDER_PIN - Ground
#define VOLTAGE_DIVIDER_PIN GPIO_NUM_32
#define SETTLING_TIME 50         // Time for the capacitor to be stable (measured at around 20ms)
#define VOLTAGE_OVERSAMPLING 32  // With an average over 32 readings, precision seems to be +- 1mV (precision is different from accuracy)

// WiFi parameters
#define TIMEOUT_WIFI 8000             // Max allowed time to connect to wifi
const char* ssid = "WifiName";  // Enter WiFi SSID
const char* password = "Password";    // Enter WiFi password
// Set your Static IP address
IPAddress local_IP(192, 168, 0, 99);
// Set your Gateway IP address
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(212, 224, 129, 90);    //optional
IPAddress secondaryDNS(212, 224, 129, 94);  //optional
WiFiClient client;

// Thingspeak parameters
const long channelNumber = 1234567;            // Enter your channel number
const char* writeAPIKey = "writeAPIKey";  // Enter your channel Write API Key

int distances[DIST_OVERSAMPLING];
int distance = DIST_ERROR;
int waterLevel = 0;
int volume = 0;
float Vbat = 0;

RTC_DATA_ATTR int lastDistance = 0;
RTC_DATA_ATTR int bootWithoutUpdate = 0;

void setup() {
  unsigned long startTime = millis();
  bootWithoutUpdate++;
  SERIAL_BEGIN(115200);
  Serial2.begin(9600, SERIAL_8N1, RX2, TX2);
  ThingSpeak.begin(client);

  DBG_PRINTLN("\n\n-----------------\nSetting pins mode");
  pinMode(LED_G_PIN, OUTPUT);
  digitalWrite(LED_G_PIN, HIGH);  // Turns LED off
  pinMode(SENSOR_GND_PIN, OUTPUT);
  pinMode(SENSOR_VCC_PIN, OUTPUT);
  pinMode(VOLTAGE_DIVIDER_PIN, INPUT);
  pinMode(VOLTAGE_DIVIDER_GROUND_PIN, OUTPUT);

  digitalWrite(VOLTAGE_DIVIDER_GROUND_PIN, LOW);
  unsigned int voltageSettled = millis() + SETTLING_TIME;
  DBG_PRINTLN("Voltage divider ground pin set to LOW");

  sensorPower(1);

  DBG_PRINTLN("LED indicator");
  blink(LED_G_PIN, LED_ON_TIME);

  while (voltageSettled > millis()) {
    //DBG_PRINTLN("Waiting for the voltage to settle");
    delay(1);
  }

  // Reading Vbat
  Vbat = analogReadOversampling(VOLTAGE_DIVIDER_PIN, VOLTAGE_OVERSAMPLING);
  Vbat = ADCCorrection(Vbat) * 3.3 * 2 / 4095;
  Vbat = round(Vbat * 1000) / 1000;  // Rounding to 3 digits

  // Reading distance
  int attempt = 0;
  int valid = 0;
  while (attempt < 2 * DIST_OVERSAMPLING && valid < DIST_OVERSAMPLING) {
    DBG_PRINTF("Attemp %d\n", attempt);
    int d = readSensor(1000);
    if (isInsideLimits(d, DIST_MIN, DIST_MAX) && d != DIST_ERROR) {
      distances[valid++] = d;
      DBG_PRINTF("\tValid %d\n", valid);
    }
    attempt++;
  }
  sensorPower(0);

  DBG_PRINTF("\nDistance and Vbat time : %dms\n", (millis() - startTime));

  if (valid > 0) {
    distance = average(distances, valid);
    waterLevel = DIST_INLET - distance;
    volume = round(waterLevel * LITERS_PER_MM);
  }

  DBG_PRINTF("Distance : %dmm\n", distance);
  DBG_PRINTF("Water level : %dmm\n", waterLevel);
  DBG_PRINTF("Volume : %dL\n", volume);
  DBG_PRINTF("Battery voltage : %.3fV\n", Vbat);

  // If the distance has been measured successfully, is different from the
  // previous one: upload to thingspeak
  // If the values haven't been updated for x times : upload to thingspeak
  if ((valid > 0 && isOutsideThreshold(lastDistance, distance)) || bootWithoutUpdate >= FORCED_UPDATE_FREQUENCY) {
    DBG_PRINTLN("\nAttempting to connect to SSID: " + String(ssid));

    // Configures static IP address
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
      DBG_PRINTLN("STA Failed to configure");
    }
    unsigned int startWifi = millis();
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED && (startWifi + TIMEOUT_WIFI) > millis()) {
      delay(1);
    }

    if (WiFi.status() == WL_CONNECTED) {
      DBG_PRINTF("Connected : %dms\n", millis() - startWifi);

      // Set the fields with the values
      ThingSpeak.setField(1, waterLevel);
      ThingSpeak.setField(2, volume);
      ThingSpeak.setField(3, Vbat);

      int index = 0;
      char buffer[80];
      for (int x = 0; x < DIST_OVERSAMPLING; x++) {
        index += sprintf(buffer + index, "%d\t", distances[x]);  // tabs makes it easy to copy into excel if needed
      }
      ThingSpeak.setStatus(buffer);                                   // ThingSpeak limits this to 255 bytes (UTF8).
      int code = ThingSpeak.writeFields(channelNumber, writeAPIKey);  // 200 : Success
      if (code == 200) {
        lastDistance = distance;
        bootWithoutUpdate = 0;
      }
      DBG_PRINTF("Upload to ThingSpeak : %d\n", code);
      DBG_PRINTF("Wifi time : %dms\n", (millis() - startWifi));
    } else {
      DBG_PRINTLN("Failed to connect");
    }
  }

  DBG_PRINTF("\nTotal time : %dms\n", (millis() - startTime));

  // First we configure the wake up source
  DBG_PRINTF("Setup ESP32 to sleep for %d seconds\n", TIME_TO_SLEEP);
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

void loop() {
  // Empty
}

int readSensor(int time) {
  byte buffer[4];
  byte index = 0;
  int distance = DIST_ERROR;

  unsigned startTime = millis();
  while (startTime + time > millis() && distance == DIST_ERROR) {
    index = 0;
    if (Serial2.available()) {
      buffer[index++] = Serial2.read();
      delay(2);

      if (buffer[0] == 255) {
        while (Serial2.available() && index < 4) {
          buffer[index++] = Serial2.read();
          delay(1);
        }
        if (index == 4) {
          byte sum = buffer[0] + buffer[1] + buffer[2];
          if (sum == buffer[3]) {
            distance = buffer[1] * 256 + buffer[2];
          }
        }
      }
    }
  }
  DBG_PRINTF("\tReturning distance : %dmm\n", distance);
  return distance;
}

bool isOutsideThreshold(int lastDistance, int distance) {
  return (abs(lastDistance - distance) > DISTANCE_THRESHOLD);
}

bool isInsideLimits(int distance, int min, int max) {
  return distance >= min && distance <= max;
}

int analogReadOversampling(int pin, int oversampling) {
  float adc = 0;

  for (int x = 0; x < oversampling; x++) {
    adc += analogRead(pin);  // Range : 0-4095
    delay(1);                // Reading stability
  }
  return round(adc / oversampling);
}

void sensorPower(int state) {
  digitalWrite(SENSOR_VCC_PIN, state);
  digitalWrite(SENSOR_GND_PIN, LOW);

  if (state) {
    DBG_PRINTLN("Sensor is ON");
  } else {
    //pinMode(RX2, INPUT);  // Avoid leakage current, using RX or TX or both changes nothing
    DBG_PRINTLN("Sensor is OFF");
  }
}

// Compute average of array without erroneous data (DIST_ERROR)
int average(int* array, int len) {
  float sum = 0;
  int addends = 0;
  for (int i = 0; i < len; i++) {
    if (array[i] != DIST_ERROR) {
      sum += array[i];
      addends++;
    }
  }
  if (addends > 0) {
    return round(sum / addends);
  }
  return DIST_ERROR;
}

void blink(int pin, int time) {
  digitalWrite(pin, LOW);  // Switch LED ON
  delay(time);
  digitalWrite(pin, HIGH);
}

float ADCCorrection(int in) {
  return 1.0 * in + 0.0;
}