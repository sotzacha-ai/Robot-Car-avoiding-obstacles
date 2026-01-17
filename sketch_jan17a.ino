/*
  ==== OBSTACLE AVOIDANCE ROBOT (ARDUINO UNO) ====

  Motors:
    M1: IN1=2, IN2=3
    M2: IN1=4, IN2=5
    M3: IN1=6, IN2=7
    M4: IN1=8, IN2=9

  Ultrasonic Sensors:
    FRONT: trig A4, echo A5
    LEFT: trig A2, echo A3
    RIGHT: trig A0, echo A1

  LED: D11
  BUZZER: D12
*/

#include <Arduino.h>

//// ==== MOTOR DIRECTION DEFINITIONS ====
const int STOPPED = 0;
const int FORWARD = 1;
const int BACKWARD = 2;

//// ==== MOTOR PIN DEFINITIONS ====
// Left side motors
const uint8_t M1_IN1 = 2;
const uint8_t M1_IN2 = 3;
const uint8_t M2_IN1 = 4;
const uint8_t M2_IN2 = 5;

// Right side motors
const uint8_t M3_IN1 = 6;
const uint8_t M3_IN2 = 7;
const uint8_t M4_IN1 = 8;
const uint8_t M4_IN2 = 9;

//// ==== ULTRASONIC SENSOR PINS ====
const uint8_t FRONT_TRIG = A4;
const uint8_t FRONT_ECHO = A5;

const uint8_t LEFT_TRIG = A2;
const uint8_t LEFT_ECHO = A3;

const uint8_t RIGHT_TRIG = A0;
const uint8_t RIGHT_ECHO = A1;

//// ==== ALERT OUTPUTS ====
const uint8_t LED_PIN    = 11;
const uint8_t BUZZER_PIN = 12;

//// ==== SYSTEM SETTINGS ====
const int FRONT_STOP_CM = 20; //// Distance threshold (cm)
const int TURN_TIME_MS = 350; //// Turn duration (open-loop)
const int STOP_TIME_MS = 120; //// Stop delay before turning
const int SENSOR_GAP_MS = 25; //// Delay between sensor readings
const unsigned long US_TIMEOUT = 35000UL; //// Ultrasonic timeout (µs)

//// ==== LOW-LEVEL MOTOR CONTROL ====
void setMotor(uint8_t in1, uint8_t in2, int d) {
  //// Controls a single DC motor via H-bridge logic
  if (d == FORWARD) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (d == BACKWARD) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    //// Motor stopped
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

//// ==== SIDE CONTROL FUNCTIONS ====
void leftSide(int d) {
  //// Controls both left motors simultaneously
  setMotor(M1_IN1, M1_IN2, d);
  setMotor(M2_IN1, M2_IN2, d);
}

void rightSide(int d) {
  //// Controls both right motors simultaneously
  setMotor(M3_IN1, M3_IN2, d);
  setMotor(M4_IN1, M4_IN2, d);
}

//// ==== HIGH-LEVEL MOVEMENT COMMANDS ====
void stopAll() {
  //// Stops all motors
  leftSide(STOPPED);
  rightSide(STOPPED);
}

void forwardAll() {
  //// Moves the robot forward
  leftSide(FORWARD);
  rightSide(FORWARD);
}

void turnLeft() {
  //// Pivot turn to the left
  leftSide(BACKWARD);
  rightSide(FORWARD);
}

void turnRight() {
  //// Pivot turn to the right
  leftSide(FORWARD);
  rightSide(BACKWARD);
}

//// ==== ULTRASONIC DISTANCE MEASUREMENT ====
long readDistanceCm(uint8_t trig, uint8_t echo) {
  //// Send trigger pulse
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  //// Measure echo pulse duration
  unsigned long us = pulseIn(echo, HIGH, US_TIMEOUT);

  //// If no echo is received, return a large distance
  if (us == 0) return 999;

  //// Convert time to distance in centimeters
  return us / 58;
}

//// ==== ALERT CONTROL ====
void setAlert(bool on) {
  //// Turn LED and buzzer ON or OFF together
  digitalWrite(LED_PIN, on ? HIGH : LOW);
  digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
}

//// ==== SETUP FUNCTION ====
void setup() {
  //// Motor pin configuration
  pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  pinMode(M3_IN1, OUTPUT); pinMode(M3_IN2, OUTPUT);
  pinMode(M4_IN1, OUTPUT); pinMode(M4_IN2, OUTPUT);

  //// Ultrasonic sensor pins
  pinMode(FRONT_TRIG, OUTPUT); pinMode(FRONT_ECHO, INPUT);
  pinMode(LEFT_TRIG,  OUTPUT); pinMode(LEFT_ECHO,  INPUT);
  pinMode(RIGHT_TRIG, OUTPUT); pinMode(RIGHT_ECHO, INPUT);

  //// Alert outputs
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  //// Initial state
  stopAll();
  setAlert(false);

  //// Serial monitor initialization
  Serial.begin(9600);
}

//// ==== MAIN LOOP (OBSTACLE AVOIDANCE LOGIC) ====
void loop() {
  //// Read distances from ultrasonic sensors
  int front = readDistanceCm(FRONT_TRIG, FRONT_ECHO);
  delay(SENSOR_GAP_MS);
  int left  = readDistanceCm(LEFT_TRIG,  LEFT_ECHO);
  delay(SENSOR_GAP_MS);
  int right = readDistanceCm(RIGHT_TRIG, RIGHT_ECHO);
  delay(SENSOR_GAP_MS);

  //// Obstacle detection using thresholding
  bool frontHit = (front <= FRONT_STOP_CM);
  bool leftHit  = (left  <= FRONT_STOP_CM);
  bool rightHit = (right <= FRONT_STOP_CM);

  //// Debug output via Serial Monitor
  Serial.print("F="); Serial.print(front);
  Serial.print(" L="); Serial.print(left);
  Serial.print(" R="); Serial.println(right);

  //// No obstacle detected
  if (!frontHit && !leftHit && !rightHit) {
    setAlert(false);
    forwardAll();
    delay(20);
    return;
  }

  //// Obstacle detected
  setAlert(true);
  stopAll();
  delay(STOP_TIME_MS);

  //// Decision-making logic
  if (leftHit && !rightHit) {
    turnRight();
  } else if (rightHit && !leftHit) {
    turnLeft();
  } else {
    //// Choose direction with more available space
    if (left > right) turnLeft();
    else turnRight();
  }

  //// Execute turn (open-loop control)
  delay(TURN_TIME_MS);

  //// Stop and reset alert
  stopAll();
  delay(80);
  setAlert(false);
}
