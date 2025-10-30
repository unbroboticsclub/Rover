// ================================================================
//  Teensy Rover Firmware v1.0
//  Author: Kayla Stafford
//  Description: Base firmware for motor + sensor + UART interface
// ================================================================

#include <Arduino.h>
#include <Encoder.h>
#include <Wire.h>
#include <ArduinoJson.h>

// -------- Pin Assignments --------
#define MOTOR_L_PWM 3
#define MOTOR_L_DIR 4
#define MOTOR_R_PWM 5
#define MOTOR_R_DIR 6

#define TRIG_PIN 7
#define ECHO_PIN 8

// -------- Encoder Setup ----------
Encoder encLeft(2, 9);    // example pins
Encoder encRight(10, 11); // example pins

// -------- Globals ---------------
float imu_roll = 0, imu_pitch = 0, imu_yaw = 0; // placeholders
long lastEncL = 0, lastEncR = 0;
unsigned long lastUpdate = 0;
int motorSpeed = 0;

// -------- UART Settings ---------
#define SERIAL_BAUD 115200

// ================================================================
//  Motor Control
// ================================================================
void setMotorSpeed(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  // Left motor
  digitalWrite(MOTOR_L_DIR, left >= 0 ? HIGH : LOW);
  analogWrite(MOTOR_L_PWM, abs(left));

  // Right motor
  digitalWrite(MOTOR_R_DIR, right >= 0 ? HIGH : LOW);
  analogWrite(MOTOR_R_PWM, abs(right));
}

// ================================================================
//  Ultrasonic Distance Measurement
// ================================================================
float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 20000);
  float distance = duration * 0.034 / 2; // in cm
  return distance;
}

// ================================================================
//  Send Telemetry to Jetson
// ================================================================
void sendTelemetry() {
  StaticJsonDocument<256> doc;
  doc["encL"] = encLeft.read();
  doc["encR"] = encRight.read();
  doc["imu"] = {imu_roll, imu_pitch, imu_yaw};
  doc["ultra"] = readUltrasonic();

  String out;
  serializeJson(doc, out);
  Serial1.println(out); // send to Jetson
}

// ================================================================
//  Parse Incoming Commands
// ================================================================
void parseCommand(String input) {
  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, input);
  if (err)
    return;

  const char* cmd = doc["cmd"];
  int spd = doc["speed"] | 150;

  if (strcmp(cmd, "forward") == 0)
    setMotorSpeed(spd, spd);
  else if (strcmp(cmd, "backward") == 0)
    setMotorSpeed(-spd, -spd);
  else if (strcmp(cmd, "left") == 0)
    setMotorSpeed(-spd, spd);
  else if (strcmp(cmd, "right") == 0)
    setMotorSpeed(spd, -spd);
  else if (strcmp(cmd, "stop") == 0)
    setMotorSpeed(0, 0);
}

// ================================================================
//  Setup
// ================================================================
void setup() {
  Serial.begin(9600);        // USB monitor
  Serial1.begin(SERIAL_BAUD); // UART to Jetson
  Wire.begin();

  pinMode(MOTOR_L_PWM, OUTPUT);
  pinMode(MOTOR_L_DIR, OUTPUT);
  pinMode(MOTOR_R_PWM, OUTPUT);
  pinMode(MOTOR_R_DIR, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.println("Rover firmware started.");
}

// ================================================================
//  Loop
// ================================================================
void loop() {
  // --- Check for new UART commands ---
  if (Serial1.available()) {
    String input = Serial1.readStringUntil('\n');
    parseCommand(input);
  }

  // --- Periodic telemetry ---
  if (millis() - lastUpdate > 100) {
    sendTelemetry();
    lastUpdate = millis();
  }
}
