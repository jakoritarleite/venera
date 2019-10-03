#include <QTRSensors.h>

#define Kp 1
#define Kd 4

#define maxSpeed 150
#define baseSpeed 100
#define speedTurn 60

#define motorAs1 8 //right Motor 1
#define motorAs2 9 //right Motor 2

#define motorBs1 12 //left Motor 1
#define motorBs2 13 //left Motor 2

#define pwmA 10
#define pwmB 11

QTRSensors qtr;

const uint8_t sensorsCount = 8;
uint16_t sensorValue[sensorsCount];

int lastError = 0;

unsigned int sensors[8];

void setup() {
  Serial.begin(9600);

  qtr.setTypeAnalog();
  qtr.setSensorPins (( const uint8_t []) {A0, A1, A2, A3, A4, A5, A6, A7}, sensorsCount);

  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  for (uint8_t i = 0; i < sensorsCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i] + " ");
  }
  Serial.println();

  for (uint8_t i = 0; i < sensorsCount; i++) {
    Serial.print(qtr.calibrationOn.maximun[i] + " ");
  }
  Serial.println();Serial.println();delay(1000);
}

void loop() {
  uint16_t position = qtr.readLineBlack(sensorValue);
  Serial.println("Position: " + String(position));

  if (position > 6700) {
    motorsToWork(1, speedTurn, 1);
    motorsToWork(0, speedTurn, 0);
  }

  if (position < 300) {
    motorsToWork(1, speedTurn, 0);
    motorsToWork(0, speedTurn, 0);
  }

  int motorSpeeds[2] = calculatePID(position);

  motorsToWork(1, motorSpeeds[0], 1);
  motorsToWork(0, leftMotorSpeed[1], 1);
}

void calculatePID(uint16_t position) {
  int error = position - 3500;
  int motorSpeed = (Kp * error) + (Kd * (error - lastError))
  lastError = error;

  int rightMotorSpeed = baseSpeed + motorSpeed;
  int leftMotorSpeed = baseSpeed - motorSpeed;

  if (rightMotorSpeed > maxSpeed) {rightMotorSpeed = maxSpeed}
  else if (leftMotorSpeed > maxSpeed) {leftMotorSpeed = maxSpeed}
  else if (rightMotorSpeed < 0) {rightMotorSpeed = 0}
  else if (leftMotorSpeed < 0) {leftMotorSpeed = 0}

  return(rightMotorSpeed, leftMotorSpeed);
}

void motorsToWork(int motor, int speed, int direction) {
  boolean inPin1 = HIGH;
  boolean inPin2 = LOW;

  if (direction == 1) {
    inPin1 = HIGH;
    inPin2 = LOW;
  }
  else if (direction == 0) {
    inPin1 = LOW;
    inPin2 = HIGH;
  }

  if (motor == 0) {
    digitalWrite(motorBs1, inPin1);
    digitalWrite(motorBs2, inPin2);
    analogWrite(pwmB, speed);
  }
  else if (motor == 1) {
    digitalWrite(motorAs1, inPin1);
      digitalWrite(motorAs2, inPin2);
      analogWrite(pwmA, speed);
  }
}
