#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Wi-Fi credentials
const char* ssid = "ESP32-Access-Point";
const char* password = "123456789";

// Server address of the master car's IMU data
const char* serverNameIMU = "http://192.168.4.1/imu";

// Motor control pins
#define RF 5
#define LB 18
#define LF 19
#define RB 23

// Ultrasonic sensor pins
#define TRIG_PIN 4
#define ECHO_PIN 2

// Control parameters
float kp_speed = 0.5;  // Proportional gain for speed control
float kp_yaw = 0.1;    // Proportional gain for yaw control

// Desired distance to maintain (in centimeters)
const float target_distance = 20.0;  // 100 cm (1 meter)

// Leader's IMU data variables
float leader_gyro_z;
float leader_yaw = 0.0;
unsigned long lastLeaderIMUTime = 0;

// Follower's IMU data variables
float follower_gyro_z;
float follower_yaw = 0.0;
unsigned long lastFollowerIMUTime = 0;

// Timing variables
unsigned long previousMillis = 0;
const long interval = 100; // Update interval in milliseconds

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  
  // Initialize Wi-Fi
  WiFi.begin(ssid, password);
  Serial.println("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) { 
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi network with IP Address: ");
  Serial.println(WiFi.localIP());
  
  // Initialize MPU6050 sensor
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Initialize motor control pins
  pinMode(RF, OUTPUT);
  pinMode(LF, OUTPUT);
  pinMode(LB, OUTPUT);
  pinMode(RB, OUTPUT);
  digitalWrite(RF, LOW);
  digitalWrite(LB, LOW);
  digitalWrite(LF, LOW);
  digitalWrite(RB, LOW);

  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  lastLeaderIMUTime = millis();
  lastFollowerIMUTime = millis();
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    // Fetch leader's IMU data
    if (WiFi.status() == WL_CONNECTED) {
      String imuData = httpGETRequest(serverNameIMU);
      if (imuData.length() > 0) {
        // Parse JSON data
        const size_t capacity = JSON_OBJECT_SIZE(6) + 60;
        DynamicJsonDocument doc(capacity);
        DeserializationError error = deserializeJson(doc, imuData);
        if (!error) {
          leader_gyro_z = doc["gyro_z"];
          
          // Calculate leader's yaw
          float deltaTime = (currentMillis - lastLeaderIMUTime) / 1000.0; // in seconds
          leader_yaw += leader_gyro_z * deltaTime * (180.0 / PI); // Convert rad/s to degrees
          leader_yaw = fmod(leader_yaw + 360.0, 360.0); // Normalize to [0, 360)
          lastLeaderIMUTime = currentMillis;
        } else {
          Serial.print("Failed to parse leader JSON: ");
          Serial.println(error.c_str());
        }
      } else {
        Serial.println("Failed to get IMU data from leader");
      }
    } else {
      Serial.println("Wi-Fi Disconnected");
    }
    
    // Read follower's own IMU data
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    follower_gyro_z = g.gyro.z;
    
    // Calculate follower's yaw
    float deltaTimeFollower = (currentMillis - lastFollowerIMUTime) / 1000.0; // in seconds
    follower_yaw += follower_gyro_z * deltaTimeFollower * (180.0 / PI); // Convert rad/s to degrees
    follower_yaw = fmod(follower_yaw + 360.0, 360.0); // Normalize to [0, 360)
    lastFollowerIMUTime = currentMillis;
    
    // Measure distance to the leader using ultrasonic sensor
    float distance = readUltrasonicDistance(TRIG_PIN, ECHO_PIN);
    
    // Compute speed error based on distance
    float speed_error = (distance - target_distance);
    float desired_speed = kp_speed * speed_error; // Since we don't have leader's speed, use proportional control
    
    // Compute throttle command
    float throttle = desired_speed;
    throttle = constrain(throttle, -1.0, 1.0);
    
    // Compute yaw error
    float yaw_error = normalize_angle(leader_yaw - follower_yaw);
    
    // Compute steering command
    float steer = kp_yaw * yaw_error;
    steer = constrain(steer, -1.0, 1.0);
    
    // Apply motor control
    applyMotorControl(throttle, steer);
    
    // Debug output
    Serial.print("Distance: "); Serial.print(distance);
    Serial.print(" cm, Speed Error: "); Serial.print(speed_error);
    Serial.print(" cm, Throttle: "); Serial.print(throttle);
    Serial.print(", Yaw Error: "); Serial.print(yaw_error);
    Serial.print(", Steer: "); Serial.println(steer);
  }
}

float normalize_angle(float angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

void applyMotorControl(float throttle, float steer) {
  // Simplified motor control logic based on throttle and steer
  String dir = "None";
  if (throttle > 0.1 and (steer < 0.1 and steer > -0.1 )) {
    // Move forward
    digitalWrite(RF, LOW);
    digitalWrite(LF, LOW);
    digitalWrite(LB, HIGH);
    digitalWrite(RB, HIGH);
    dir = "Forward";
    delay(20);
  } else if (throttle < -0.1 and (steer < 0.1 and steer > -0.1 )) {
    // Move backward
    digitalWrite(RF, HIGH);
    digitalWrite(LF, HIGH);
    digitalWrite(LB, LOW);
    digitalWrite(RB, LOW);
    dir = "Backward";
    delay(20);
  } else if (throttle < 0.1 and throttle > -0.1) {
    // Stop
    digitalWrite(RF, LOW);
    digitalWrite(LF, LOW);
    digitalWrite(LB, LOW);
    digitalWrite(RB, LOW);
    dir = "Stop";
    delay(20);
  }
  else if (steer > 0.1 ){
    //turn right
    digitalWrite(RF,LOW);
    digitalWrite(LB,LOW);
    digitalWrite(LF,HIGH);
    digitalWrite(RB,HIGH);
    dir = "Right";
    delay(10);
  }
  else if (steer < -0.1) {
    //turn left
    digitalWrite(RF,HIGH);
    digitalWrite(LB,HIGH);
    digitalWrite(LF,LOW);
    digitalWrite(RB,LOW);
    dir = "Left";
    delay(10);
  }
  Serial.print(", Direction: "); Serial.println(dir);
  
  // // Pivot in place if throttle is zero and steer is significant
  // if (abs(throttle) < 0.1 && abs(steer) > 0.1) {
  //   if (steer > 0.1) {
  //     // Pivot right
  //     digitalWrite(RF, HIGH);
  //     digitalWrite(LF, LOW);
  //     digitalWrite(LB, LOW);
  //     digitalWrite(RB, HIGH);
  //   } else {
  //     // Pivot left
  //     digitalWrite(RF, LOW);
  //     digitalWrite(LF, HIGH);
  //     digitalWrite(LB, HIGH);
  //     digitalWrite(RB, LOW);
  //   }
  // }
}
long prev=20;
long readUltrasonicDistance(int triggerPin, int echoPin)
{
  // Clear the trigger pin
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);

  // Send a 10 microsecond pulse to the trigger pin
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  // Read the echo pin. pulseIn returns the duration (length of the pulse) in microseconds
  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout

  // Calculate the distance in centimeters based on the speed of sound
  long distanceCm = duration / 29 / 2;
  if (abs(prev-distanceCm)>20){
    distanceCm= prev;
  }
  prev= distanceCm;
  return distanceCm;
}

String httpGETRequest(const char* serverName) {
  WiFiClient client;
  HTTPClient http;
    
  // Initiate HTTP connection
  http.begin(client, serverName);
  
  // Send HTTP GET request
  int httpResponseCode = http.GET();
  
  String payload = ""; 
  
  if (httpResponseCode > 0) {
    payload = http.getString();
  } else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  http.end();

  return  payload;
}