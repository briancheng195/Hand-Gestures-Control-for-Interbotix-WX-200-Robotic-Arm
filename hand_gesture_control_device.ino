// Program for GY-85 IMU Sensor, HC-SR04 Ultrasonic Sensor and Flex Sensor
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_HMC5883_U.h>
#include "ITG3200.h"
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
ITG3200 gyro;
float pitch = 0.0;
float yaw = 0.0;
float yawOffset = 0.0;
const int trigPin = 9;
const int echoPin = 10;
long duration;
float distance;
float initialDistance = 0.0; // Initial distance from the ultrasonic sensor
const int flexPin = A0; // Flex sensor, pin A0 to read analog input
int angleValue;
int output;

void setup() {
  Serial.begin(19200);
  accel.begin();
  mag.begin();
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  // Calibration Routine
  for (int i = 0; i < 100; i++) {
    sensors_event_t magEvent;
    mag.getEvent(&magEvent);
    float heading = atan2(magEvent.magnetic.y, magEvent.magnetic.x);
    if (heading < 0) {
      heading += 2 * PI;
    }
    yawOffset += heading;
    delay(10);
  }
  yawOffset /= 100;

  // Initial distance measurement
  initialDistance = measureDistance();
}

float measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t event;
  accel.getEvent(&event);
  float pitchAcc = atan2(event.acceleration.y, event.acceleration.z) * RAD_TO_DEG;

  sensors_event_t magEvent;
  mag.getEvent(&magEvent);

  float heading = atan2(magEvent.magnetic.y, magEvent.magnetic.x);
  if (heading < 0) {
    heading += 2 * PI;
  }
  float yawAcc = (heading - yawOffset) * RAD_TO_DEG;
  
  float rollAcc = atan2(event.acceleration.x, event.acceleration.z) * RAD_TO_DEG;

  // Measure the current distance
  float currentDistance = measureDistance();

  // Calculate the difference in distance traveled
  float distanceDifference = currentDistance - initialDistance;
  distanceDifference = round(distanceDifference);

  // Calculate angle for Flex sensor 
  angleValue = analogRead(flexPin); 
  output = 0                  // Indication that the gripper of the arm is opened
  if (angleValue > 0) {       // When flex sensor is bent, angle value increases
    output = 1                // User grips his hand, indication that the gripper of the arm would be closed
  }
  
  Serial.print(distanceDifference);
  Serial.print(",");
  Serial.print(pitchAcc);
  Serial.print(",");
  Serial.print(yawAcc);
  Serial.print(",");
  Serial.print(rollAcc);
  Serial.print(",");
  Serial.print(output);
  Serial.println(" ");
}
