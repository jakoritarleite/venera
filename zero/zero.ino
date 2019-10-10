#include <QTRSensors.h>

#define motorAs1 8
#define motorAs2 7

#define motorBs1 12
#define motorBs2 13

#define pwmA 10
#define pwmB 11

QTRSensors qtr;

const uint8_t sensorCount = 8;
uint16_t sensorValues[sensorCount];

int Kp = 20;
int Kd = 120;

int lastError = 0;
int error;

int speedBase = 100;
int speedMax = 255;

int motorsKey[4] = {HIGH, LOW, HIGH, LOW};

void setup() {
	qtr.setTypeAnalog();
	qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, sensorCount); 

	for (uint16_t i = 0; i < 400; i++) { qtr.calibrate(); }

		Serial.begin(9600);

	Serial.print("Minimum values: ");
	for (uint8_t i = 0; i < sensorCount; i++) {
		Serial.print(qtr.calibrationOn.minimum[i]);
		Serial.print(" ");
	}

	Serial.print("Maximum values: ");
	for (uint8_t i = 0; i < sensorCount; i++) {
		Serial.print(qtr.calibrationOn.maximum[i]); 
		Serial.print(" ");
	} for (int i = 0; i < 2; i++) {Serial.println();}

	delay(1000);
}

void motorsToWork(int A, int B, int value1 = HIGH, int value2 = LOW, int value3 = HIGH, int value4 = LOW) {
  digitalWrite(motorAs1, value1);
  digitalWrite(motorAs2, value2);
  digitalWrite(motorBs1, value3);
  digitalWrite(motorBs2, value4);

  analogWrite(pwmA, A);
  analogWrite(pwmB, B);
}

void loop() {
	readSensors();

}

void readSensors() {
	uint16_t position = qtr.readLineWhite(sensorValues);

//  for (uint8_t i = 0; i < sensorCount; i++) {
//    Serial.print(sensorValues[i]);
//    Serial.print("\t");  
//  } Serial.print(position);// Serial.println();

	calculatePID(position);

}

void calculatePID(uint16_t position) {

	if (position >= 0  && position <= 500 ) { error = -7; }
	else if (position > 500  && position <= 1000) { error = -6; }
	else if (position > 1000 && position <= 1500) { error = -5; }
	else if (position > 1500 && position <= 2000) { error = -4; }
	else if (position > 2000 && position <= 2500) { error = -3; }
	else if (position > 2500 && position <= 3000) { error = -2; }
	else if (position > 3000 && position <= 3300) { error = -1; }
	else if (position > 3300 && position <= 3700) { error = 0; }
	else if (position > 3700 && position <= 4000) { error = 1; }
	else if (position > 4000 && position <= 4500) { error = 2; }
	else if (position > 4500 && position <= 5000) { error = 3; }
	else if (position > 5000 && position <= 5500) { error = 4; }
	else if (position > 5500 && position <= 6000) { error = 5; }
	else if (position > 6000 && position <= 6500) { error = 6; }
	else if (position > 6500 && position <= 7000) { error = 7; }
  error = 7;
	int speedMotor = (Kp * error) + (Kd * (error - lastError));
	lastError = error;
  
	int speedA = speedBase + speedMotor;
	int speedB = speedBase - speedMotor;
  Serial.print(error); Serial.print("\t"); Serial.print(speedA); Serial.print("\t"); Serial.print(speedB); Serial.println();

	if (speedA > speedMax) {
		speedA = speedMax; 
	} else if (speedA < (speedMax * -1)) {
		speedA = speedMax * -1;
	}
	if (speedB > speedMax) {
		speedB = speedMax;
	} else if (speedB < (speedMax * -1)) {
		speedB = speedMax * -1;
	}
	if(speedA < 0) {
		speedA *= -1;
    motorsKey[0] = LOW;
    motorsKey[1] = HIGH;
	}  
	if(speedB < 0) {
		speedB *= -1;
    motorsKey[2] = LOW;
    motorsKey[3] = HIGH;
	}

	if (speedA > 0) {
		motorsKey[0] = HIGH;
		motorsKey[1] = LOW;
	}
	if (speedB > 0) {
    motorsKey[2] = HIGH;
    motorsKey[3] = LOW;
	}
 
	//motorsToWork(speedA, speedB, motorsKey[0], motorsKey[1], motorsKey[2], motorsKey[3]);

  digitalWrite(motorAs1, motorsKey[0]);
  digitalWrite(motorAs2, motorsKey[1]);
  digitalWrite(motorBs1, motorsKey[2]);
  digitalWrite(motorBs2, motorsKey[3]);

  analogWrite(pwmA, speedA);
  analogWrite(pwmB, speedB);

}
