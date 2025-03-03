#include "BluetoothSerial.h"
BluetoothSerial ESP_BT; // Object for Bluetooth

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include <nvs_flash.h> // Include NVS flash library

Adafruit_MPU6050 mpu;

#define RF 5
#define LB 18
#define LF 19
#define RB 23

int incoming;

// Wi-Fi credentials
const char* ssid = "ESP32-Access-Point";
const char* password = "123456789";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

void setup() {
  Serial.begin(9600); // Start Serial monitor at 9600 baud

  // Initialize NVS (Non-Volatile Storage) for Wi-Fi and Bluetooth coexistence
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Initialize Wi-Fi Access Point before Bluetooth
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  // Initialize Bluetooth after Wi-Fi
  ESP_BT.begin("Car2");
  Serial.println("Bluetooth Device is Ready to Pair");

  // Initialize MPU6050 sensor
  Serial.println("Adafruit MPU6050 test!");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
  }
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
  }
  Serial.println("");

  // Initialize motor control pins
  pinMode(RF, OUTPUT);
  pinMode(LF, OUTPUT);
  pinMode(LB, OUTPUT);
  pinMode(RB, OUTPUT);
  digitalWrite(RF, LOW);
  digitalWrite(LB, LOW);
  digitalWrite(LF, LOW);
  digitalWrite(RB, LOW);

  delay(100);

  // Server handlers
  server.on("/imu", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "application/json", readIMU().c_str());
  });

  // Start server
  server.begin();
}

void loop() {
  if (ESP_BT.available()) // Check if we receive anything from Bluetooth
  {
    incoming = ESP_BT.read(); // Read what we receive
    if (incoming != 83) {
      Serial.print("Received: "); Serial.println(incoming);
    }

    if (incoming == 83) {
      digitalWrite(RF, LOW);
      digitalWrite(LB, LOW);
      digitalWrite(LF, LOW);
      digitalWrite(RB, LOW);
    }
    else if (incoming == 66) {
      digitalWrite(RF, HIGH);
      digitalWrite(LB, LOW);
      digitalWrite(LF, HIGH);
      digitalWrite(RB, LOW);
    }
    else if (incoming == 70) {
      digitalWrite(RF, LOW);
      digitalWrite(LB, HIGH);
      digitalWrite(LF, LOW);
      digitalWrite(RB, HIGH);
    }
    else if (incoming == 82) {
      digitalWrite(RF, HIGH);
      digitalWrite(LB, HIGH);
      digitalWrite(LF, LOW);
      digitalWrite(RB, LOW);
    }
    else if (incoming == 76) {
      digitalWrite(RF, LOW);
      digitalWrite(LB, LOW);
      digitalWrite(LF, HIGH);
      digitalWrite(RB, HIGH);
    }
    else if (incoming == 68) {
      digitalWrite(RF, LOW);
      digitalWrite(LB, LOW);
      digitalWrite(LF, LOW);
      digitalWrite(RB, LOW);
    }
    else if (incoming == 6) {
      digitalWrite(RF, HIGH);
      digitalWrite(LB, HIGH);
      digitalWrite(LF, LOW);
      digitalWrite(RB, LOW);
      delay(300);
      digitalWrite(RF, LOW);
      digitalWrite(LB, LOW);
      digitalWrite(LF, LOW);
      digitalWrite(RB, LOW);
    }
    else if (incoming == 5) {
      digitalWrite(RF, LOW);
      digitalWrite(LB, LOW);
      digitalWrite(LF, HIGH);
      digitalWrite(RB, HIGH);
      delay(300);
      digitalWrite(RF, LOW);
      digitalWrite(LB, LOW);
      digitalWrite(LF, LOW);
      digitalWrite(RB, LOW);
    }
    else if (incoming == 4) {
      digitalWrite(RF, HIGH);
      digitalWrite(LB, LOW);
      digitalWrite(LF, HIGH);
      digitalWrite(RB, LOW);
      delay(400);
      digitalWrite(RF, LOW);
      digitalWrite(LB, LOW);
      digitalWrite(LF, LOW);
      digitalWrite(RB, LOW);
    }
    else if (incoming == 3) {
      digitalWrite(RF, LOW);
      digitalWrite(LB, HIGH);
      digitalWrite(LF, LOW);
      digitalWrite(RB, HIGH);
      delay(400);
      digitalWrite(RF, LOW);
      digitalWrite(LB, LOW);
      digitalWrite(LF, LOW);
      digitalWrite(RB, LOW);
    }
  }
  delay(20); // Small delay to allow other tasks to run
}

String readIMU() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  String json = "{";
  json += "\"accel_x\":" + String(a.acceleration.x) + ",";
  json += "\"accel_y\":" + String(a.acceleration.y) + ",";
  json += "\"accel_z\":" + String(a.acceleration.z) + ",";
  json += "\"gyro_x\":" + String(g.gyro.x) + ",";
  json += "\"gyro_y\":" + String(g.gyro.y) + ",";
  json += "\"gyro_z\":" + String(g.gyro.z);
  json += "}";
  return  json;
}