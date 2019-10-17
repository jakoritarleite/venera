#include <Encoder.h>
#include <QTRSensors.h>

//Motors pins
#define motorAs1 8
#define motorAs2 7

#define motorBs1 12
#define motorBs2 13

#define pwmA 10
#define pwmB 11

//Encoders pins
#define encoderA1 2
#define encoderA2 3

//Bluetooth pins
#define pinRX 1
#define pinTX 0

//Other pin
#define CALIBRATE_PIN 6

//Defining functions
QTRSensors qtr;
Encoder myEncA(encoderA1, encoderA2);

/*		Defining Variables		*/

//Sensors
const uint8_t sensorCount = 8;
uint16_t sensorValues[sensorCount];

bool sensorDigital[sensorCount] = {false, false, false, true, true, false, false, false};
int error;

//Encoders
long oldPositionA = NULL;
boolean doBoost = false;

//Boost
boolean doBoost = false; //If it is true, the car is do a boost based on the encoder's value
int startBoostTime[2] = {};
int endBoostTime[2] = {};

//Bluetooth
boolean useBluetooth = false;

void setup() {
	analogWrite(CALIBRATE_PIN, 255);

  qtr.setTypeAnalog();
	qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, sensorCount);

	//Calibrating sensor
	for (uint16_t i = 0; i < 400; i++) { qtr.calibrate(); }

	analogWrite(CALIBRATE_PIN, 0);

	Serial.begin(9600);

  oldPositionA = myEncA.read();

	delay(3000);
}

void motorsToWork(int A, int B, int value1, int value2, int value3, int value4) {
  digitalWrite(motorAs1, value1);
	digitalWrite(motorAs2, value2);
	digitalWrite(motorBs1, value3);
	digitalWrite(motorBs2, value4);

	analogWrite(pwmA, A);
	analogWrite(pwmB, B);
}


void calculateError() {
	uint16_t position = qtr.readLineWhite(sensorValues);

	for (uint8_t i = 0; i < sensorCount; i++) {
		if (sensorValues[i] < 250) {sensorDigital[i] = true;}
		else {sensorDigital[i] = false;}
	}
  
  if (!sensorDigital[0] && !sensorDigital[1] && !sensorDigital[2] && !sensorDigital[3] && !sensorDigital[4] && !sensorDigital[5] && !sensorDigital[6] &&  sensorDigital[7]) { error = -7; motorsToWork(255, 255, HIGH, LOW, LOW, HIGH);}
  else if (!sensorDigital[0] && !sensorDigital[1] && !sensorDigital[2] && !sensorDigital[3] && !sensorDigital[4] && !sensorDigital[5] &&  sensorDigital[6] &&  sensorDigital[7]) { error = -6; motorsToWork(245, 145, HIGH, LOW, LOW, HIGH);}
  else if (!sensorDigital[0] && !sensorDigital[1] && !sensorDigital[2] && !sensorDigital[3] && !sensorDigital[4] &&  sensorDigital[5] &&  sensorDigital[6] &&  sensorDigital[7]) { error = -5; motorsToWork(235, 115, HIGH, LOW,  HIGH, LOW);}
  else if (!sensorDigital[0] && !sensorDigital[1] && !sensorDigital[2] && !sensorDigital[3] && !sensorDigital[4] &&  sensorDigital[5] &&  sensorDigital[6] && !sensorDigital[7]) { error = -4; motorsToWork(235, 115, HIGH, LOW, HIGH, LOW);}
  else if (!sensorDigital[0] && !sensorDigital[1] && !sensorDigital[2] && !sensorDigital[3] &&  sensorDigital[4] &&  sensorDigital[5] &&  sensorDigital[6] && !sensorDigital[7]) { error = -3; motorsToWork(235, 145, HIGH, LOW, HIGH, LOW);}
  else if (!sensorDigital[0] && !sensorDigital[1] && !sensorDigital[2] && !sensorDigital[3] &&  sensorDigital[4] &&  sensorDigital[5] && !sensorDigital[6] && !sensorDigital[7]) { error = -2; motorsToWork(235, 190, HIGH, LOW, HIGH, LOW);}
  else if (!sensorDigital[0] && !sensorDigital[1] && !sensorDigital[2] &&  sensorDigital[3] &&  sensorDigital[4] &&  sensorDigital[5] && !sensorDigital[6] && !sensorDigital[7]) { error = -1; motorsToWork(235, 205, HIGH, LOW, HIGH, LOW);}
  else if (!sensorDigital[0] && !sensorDigital[1] && !sensorDigital[2] &&  sensorDigital[3] &&  sensorDigital[4] && !sensorDigital[5] && !sensorDigital[6] && !sensorDigital[7]) { error = 0; motorsToWork(255, 255, HIGH, LOW, HIGH, LOW);}
  else if (!sensorDigital[0] && !sensorDigital[1] &&  sensorDigital[2] &&  sensorDigital[3] &&  sensorDigital[4] && !sensorDigital[5] && !sensorDigital[6] && !sensorDigital[7]) { error = 1; motorsToWork(205, 255, HIGH, LOW, HIGH, LOW);}
  else if (!sensorDigital[0] && !sensorDigital[1] &&  sensorDigital[2] &&  sensorDigital[3] && !sensorDigital[4] && !sensorDigital[5] && !sensorDigital[6] && !sensorDigital[7]) { error = 2; motorsToWork(190, 255, HIGH, LOW, HIGH, LOW);}
  else if (!sensorDigital[0] &&  sensorDigital[1] &&  sensorDigital[2] &&  sensorDigital[3] && !sensorDigital[4] && !sensorDigital[5] && !sensorDigital[6] && !sensorDigital[7]) { error = 3; motorsToWork(145, 255, HIGH, LOW, HIGH, LOW);}
  else if (!sensorDigital[0] &&  sensorDigital[1] &&  sensorDigital[2] && !sensorDigital[3] && !sensorDigital[4] && !sensorDigital[5] && !sensorDigital[6] && !sensorDigital[7]) { error = 4; motorsToWork(115, 255, HIGH, LOW, HIGH, LOW);}
  else if (sensorDigital[0] &&  sensorDigital[1] &&  sensorDigital[2] && !sensorDigital[3] && !sensorDigital[4] && !sensorDigital[5] && !sensorDigital[6] && !sensorDigital[7]) { error = 5; motorsToWork(115, 255, HIGH, LOW, HIGH, LOW);}
  else if (sensorDigital[0] &&  sensorDigital[1] && !sensorDigital[2] && !sensorDigital[3] && !sensorDigital[4] && !sensorDigital[5] && !sensorDigital[6] && !sensorDigital[7]) { error = 6; motorsToWork(145, 255, LOW, HIGH, HIGH, LOW);}
  else if (sensorDigital[0] && !sensorDigital[1] && !sensorDigital[2] && !sensorDigital[3] && !sensorDigital[4] && !sensorDigital[5] && !sensorDigital[6] && !sensorDigital[7]) { error = 7; motorsToWork(255, 255, LOW, HIGH, HIGH, LOW);}

  //I increased 30, but don't tested yet.
}

void encodersToWork() {
	long newPositionA = myEncA.read();

	if (doBluetooth) {
      Serial.print("A: ");
      Serial.print(newPositionA);
      Serial.println();
	}
  //If you want to know which is the encoder value, just define useBluetooth to true. 
	
  if ((newPositionA - oldPositionA) <= -61500) {
    while (true) {motorsToWork(0, 0, HIGH, HIGH, HIGH, HIGH);}
  }
  
  else if (doBoost && (newPositionA - oldPositionA) <= startBoostTime[0] && (newPositionA - oldPositionA) >= endBoostTime[0]) {
    digitalWrite(CALIBRATE_PIN, HIGH);
  	if (error == 0) {motorsToWork(225, 225, HIGH, LOW, HIGH, LOW);} 
    else if (error == 1) {motorsToWork(175, 225, HIGH, LOW, HIGH, LOW);} 
    else if (error == -1) {motorsToWork(225, 175, HIGH, LOW, HIGH, LOW);}
    else if (error == 2) {motorsToWork(155, 225, HIGH, LOW, HIGH, LOW);}
    else if (error == -2) {motorsToWork(225, 155, HIGH, LOW, HIGH, LOW);}
    else if (error == 3) {motorsToWork(115, 235, HIGH, LOW, HIGH, LOW);}
    else if (error == -3) {motorsToWork(235, 115, HIGH, LOW, HIGH, LOW);}
    digitalWrite(CALIBRATE_PIN, LOW);
  }
  
  else if (doBoost && (newPositionA - oldPositionA) <= startBoostTime[1] && (newPositionA - oldPositionA) >= endBoostTime[1]) {
    digitalWrite(CALIBRATE_PIN, HIGH);
    if (error == 0) {motorsToWork(255, 255, HIGH, LOW, HIGH, LOW);}
    else if (error == 1) {motorsToWork(190, 250, HIGH, LOW, HIGH, LOW);} 
    else if (error == -1) {motorsToWork(250, 190, HIGH, LOW, HIGH, LOW);} 
    else if (error == 2) {motorsToWork(170, 240, HIGH, LOW, HIGH, LOW);}
    else if (error == -2) {motorsToWork(240, 170, HIGH, LOW, HIGH, LOW);}
    else if (error == 3) {motorsToWork(115, 235, HIGH, LOW, HIGH, LOW);}
    else if (error == -3) {motorsToWork(235, 115, HIGH, LOW, HIGH, LOW);}
    digitalWrite(CALIBRATE_PIN, LOW);
  }
} //These two else if is to verify and do the boost

void loop() {
  calculateError();
  encodersToWork();
}