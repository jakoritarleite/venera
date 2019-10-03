#include <QTRSensors.h>

#define Kp 1 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 4// experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define MaxSpeed 100// max speed of the robot
#define BaseSpeed 100 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define speedturn 60
#define rightMotor1 8
#define rightMotor2 9
#define rightMotorPWM 10
#define leftMotor1 12
#define leftMotor2 13
#define leftMotorPWM 11

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
int lastError = 0;
unsigned int sensors[8];

void setup() {
  Serial.begin(9600);

  qtr.setTypeAnalog();
  qtr.setSensorPins (( const uint8_t []) {A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void loop() {
   uint16_t position = qtr.readLineBlack(sensorValues);
   Serial.println(position);
   
  if(position>6700){
    move(1, speedturn, 1);//motor derecho hacia adelante
    move(0, speedturn, 0);//motor izquierdo hacia adelante
    return;    
  }
  if(position<300){ 
    move(1, speedturn, 0);//motor derecho hacia adelante
    move(0, speedturn, 1);//motor izquierdo hacia adelante
    return;
  }
  
  int error = position - 3500;
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = BaseSpeed + motorSpeed;
  int leftMotorSpeed = BaseSpeed - motorSpeed;
  
  if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > MaxSpeed ) leftMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0)rightMotorSpeed = 0;    
  if (leftMotorSpeed < 0)leftMotorSpeed = 0;
    
  move(1, rightMotorSpeed, 1);//motor derecho hacia adelante
  move(0, leftMotorSpeed, 1);//motor izquierdo hacia adelante
}

void move(int motor, int speed, int direction){
  
  boolean inPin1=HIGH;
  boolean inPin2=LOW;
  
  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }  
  if(direction == 0){
    inPin1 = LOW;
    inPin2 = HIGH;
  }

  if(motor == 0){
    digitalWrite(leftMotor1, inPin1);
    digitalWrite(leftMotor2, inPin2);
    analogWrite(leftMotorPWM, speed);
  }
  if(motor == 1){
    digitalWrite(rightMotor1, inPin1);
    digitalWrite(rightMotor2, inPin2);
    analogWrite(rightMotorPWM, speed);
  } 
} 
