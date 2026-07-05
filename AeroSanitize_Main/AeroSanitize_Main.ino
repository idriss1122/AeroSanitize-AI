#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include "RTClib.h"
#include "DHT.h"

// --- PIN DEFINITIONS ---
#define I2C_DATA_PIN 8
#define I2C_CLOCK_PIN 9
#define LDR_PIN 3
#define RADAR_PIN 5
#define DOOR_PIN 6
#define DHT_PIN 7

#define DHTTYPE DHT22

// --- HARDWARE OBJECTS ---
Adafruit_AMG88xx thermalCamera;
RTC_DS3231 rtc;
DHT dht(DHT_PIN, DHTTYPE);

float temperaturePixels[AMG88xx_PIXEL_ARRAY_SIZE];

// --- THE 3 STOPWATCHES ---
unsigned long previousFastMillis = 0;
const long fastInterval = 100;    // 10x per second (Security)

unsigned long previousMediumMillis = 0;
const long mediumInterval = 1000; // 1x per second (Visuals & Time)

unsigned long previousSlowMillis = 0;
const long slowInterval = 2000;   // Every 2 seconds (Climate)

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("==========================================");
  Serial.println("Aero-Sanitize AI: THE GRAND FUSION");
  Serial.println("Booting 6-Sensor Array...");

  // Setup Standard Pins
  pinMode(RADAR_PIN, INPUT);
  pinMode(DOOR_PIN, INPUT_PULLUP);
  
  // Start I2C Bus
  Wire.begin(I2C_DATA_PIN, I2C_CLOCK_PIN);

  // Initialize Sensors
  dht.begin();
  
  if (!thermalCamera.begin()) {
    Serial.println("ERROR: Thermal Camera missing!");
    while(1) delay(10);
  }
  
  if (!rtc.begin()) {
    Serial.println("ERROR: RTC missing!");
    while(1) delay(10);
  }

  Serial.println("SUCCESS: All systems nominal. Beginning live monitoring.");
  Serial.println("==========================================");
}

void loop() {
  unsigned long currentMillis = millis();

  // ==========================================
  // STOPWATCH 1: FAST (SECURITY) - 100ms
  // ==========================================
  if (currentMillis - previousFastMillis >= fastInterval) {
    previousFastMillis = currentMillis;

    if (digitalRead(DOOR_PIN) == HIGH) {
      Serial.println("🚨 ALERT: DOOR IS OPEN!");
    }
    
    if (digitalRead(RADAR_PIN) == HIGH) {
      Serial.println("🏃 ALERT: MICRO-MOVEMENT DETECTED (Radar)!");
    }
  }

  // ==========================================
  // STOPWATCH 2: MEDIUM (VISUALS/TIME) - 1000ms
  // ==========================================
  if (currentMillis - previousMediumMillis >= mediumInterval) {
    previousMediumMillis = currentMillis;

    DateTime now = rtc.now();
    int lightLevel = analogRead(LDR_PIN);
    
    // Read Camera and Ambient
    thermalCamera.readPixels(temperaturePixels);
    float ambientTemp = thermalCamera.readThermistor();

    // Find Hottest Pixel
    float maxTemp = 0.0;
    for (int i = 0; i < 64; i++) {
      if (temperaturePixels[i] > maxTemp) {
        maxTemp = temperaturePixels[i];
      }
    }

    // Dynamic Threshold Logic (Summer Heat Fix)
    float dynamicThreshold = ambientTemp + 3.0;
    bool cameraBlind = (ambientTemp >= 33.0);
    bool heatDetected = (!cameraBlind && maxTemp >= dynamicThreshold);

    // Print the Medium Update
    Serial.print("[");
    Serial.print(now.hour()); Serial.print(":");
    if (now.minute() < 10) Serial.print("0");
    Serial.print(now.minute()); Serial.print(":");
    if (now.second() < 10) Serial.print("0");
    Serial.print(now.second());
    Serial.print("] Light: "); Serial.print(lightLevel);
    Serial.print(" | Cam Max: "); Serial.print(maxTemp);
    
    if (cameraBlind) Serial.println("C (BLIND)");
    else if (heatDetected) Serial.println("C (HEAT DETECTED)");
    else Serial.println("C (Clear)");
  }

  // ==========================================
  // STOPWATCH 3: SLOW (CLIMATE) - 2000ms
  // ==========================================
  if (currentMillis - previousSlowMillis >= slowInterval) {
    previousSlowMillis = currentMillis;

    float hum = dht.readHumidity();
    float temp = dht.readTemperature();

    if (!isnan(hum) && !isnan(temp)) {
      Serial.print("   ↳ 🌍 CLIMATE UPDATE: Temp: ");
      Serial.print(temp);
      Serial.print("C | Humidity: ");
      Serial.print(hum);
      Serial.println("%");
    }
  }
}