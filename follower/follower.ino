#include <Encoder.h>
#include <QTRSensors.h>
#include <SoftwareSerial.h>

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

#define encoderB1 4
#define encoderB2 5

//Bluetooth pins
#define pinRX 1
#define pinTX 0

//Other pin
#define CALIBRATE_PIN 6

//Defining functions
QTRSensors qtr;
SoftwareSerial Bluetooth(pinRX, pinTX);
Encoder myEncA(encoderA1, encoderA2);
Encoder myEncB(encoderB1, encoderB2);

/*		Defining Variables		*/

//Sensors
const uint8_t sensorCount = 8;
uint16_t sensorValues[sensorCount];

bool sensorDigital[sensorCount] = {false, false, false, true, true, false, false, false};
int error;

//Encoders
long oldPositionA = NULL;
long oldPositionB = NULL;

void setup() {
	analogWrite(CALIBRATE_PIN, 255);

	qtr.setTypeAnalog();
	qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, sensorCount);

	//Calibrating sensor
	for (uint16_t i = 0; i < 400; i++) { qtr.calibrate(); }

	analogWrite(CALIBRATE_PIN, 0);

	Serial.begin(9600);
	Bluetooth.begin(19200);

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
		if (sensorValues[i] < 100) {sensorDigital[i] = true;}
		else {sensorDigital[i] = false;}
	}

	if (!sensorDigital[0] && !sensorDigital[1] && !sensorDigital[2] && !sensorDigital[3] && !sensorDigital[4] && !sensorDigital[5] && !sensorDigital[6] &&  sensorDigital[7]) { error = -7; motorsToWork(255, 255, HIGH, LOW, LOW, HIGH);}
	else if (!sensorDigital[0] && !sensorDigital[1] && !sensorDigital[2] && !sensorDigital[3] && !sensorDigital[4] && !sensorDigital[5] &&  sensorDigital[6] &&  sensorDigital[7]) { error = -6; motorsToWork(200, 100, HIGH, LOW, LOW, HIGH);}
	else if (!sensorDigital[0] && !sensorDigital[1] && !sensorDigital[2] && !sensorDigital[3] && !sensorDigital[4] &&  sensorDigital[5] &&  sensorDigital[6] &&  sensorDigital[7]) { error = -5; motorsToWork(200, 80, HIGH, LOW,  HIGH, LOW);}
	else if (!sensorDigital[0] && !sensorDigital[1] && !sensorDigital[2] && !sensorDigital[3] && !sensorDigital[4] &&  sensorDigital[5] &&  sensorDigital[6] && !sensorDigital[7]) { error = -4; motorsToWork(200, 100, HIGH, LOW, HIGH, LOW);}
	else if (!sensorDigital[0] && !sensorDigital[1] && !sensorDigital[2] && !sensorDigital[3] &&  sensorDigital[4] &&  sensorDigital[5] &&  sensorDigital[6] && !sensorDigital[7]) { error = -3; motorsToWork(200, 120, HIGH, LOW, HIGH, LOW);}
	else if (!sensorDigital[0] && !sensorDigital[1] && !sensorDigital[2] && !sensorDigital[3] &&  sensorDigital[4] &&  sensorDigital[5] && !sensorDigital[6] && !sensorDigital[7]) { error = -2; motorsToWork(200, 160, HIGH, LOW, HIGH, LOW);}
	else if (!sensorDigital[0] && !sensorDigital[1] && !sensorDigital[2] &&  sensorDigital[3] &&  sensorDigital[4] &&  sensorDigital[5] && !sensorDigital[6] && !sensorDigital[7]) { error = -1; motorsToWork(200, 180, HIGH, LOW, HIGH, LOW);}
	else if (!sensorDigital[0] && !sensorDigital[1] && !sensorDigital[2] &&  sensorDigital[3] &&  sensorDigital[4] && !sensorDigital[5] && !sensorDigital[6] && !sensorDigital[7]) { error = 0; motorsToWork(200, 200, HIGH, LOW, HIGH, LOW);}
	else if (!sensorDigital[0] && !sensorDigital[1] &&  sensorDigital[2] &&  sensorDigital[3] &&  sensorDigital[4] && !sensorDigital[5] && !sensorDigital[6] && !sensorDigital[7]) { error = 1; motorsToWork(180, 200, HIGH, LOW, HIGH, LOW);}
	else if (!sensorDigital[0] && !sensorDigital[1] &&  sensorDigital[2] &&  sensorDigital[3] && !sensorDigital[4] && !sensorDigital[5] && !sensorDigital[6] && !sensorDigital[7]) { error = 2; motorsToWork(160, 200, HIGH, LOW, HIGH, LOW);}
	else if (!sensorDigital[0] &&  sensorDigital[1] &&  sensorDigital[2] &&  sensorDigital[3] && !sensorDigital[4] && !sensorDigital[5] && !sensorDigital[6] && !sensorDigital[7]) { error = 3; motorsToWork(120, 200, HIGH, LOW, HIGH, LOW);}
	else if (!sensorDigital[0] &&  sensorDigital[1] &&  sensorDigital[2] && !sensorDigital[3] && !sensorDigital[4] && !sensorDigital[5] && !sensorDigital[6] && !sensorDigital[7]) { error = 4; motorsToWork(100, 200, HIGH, LOW, HIGH, LOW);}
	else if ( sensorDigital[0] &&  sensorDigital[1] &&  sensorDigital[2] && !sensorDigital[3] && !sensorDigital[4] && !sensorDigital[5] && !sensorDigital[6] && !sensorDigital[7]) { error = 5; motorsToWork(80, 200, HIGH, LOW, HIGH, LOW);}
	else if ( sensorDigital[0] &&  sensorDigital[1] && !sensorDigital[2] && !sensorDigital[3] && !sensorDigital[4] && !sensorDigital[5] && !sensorDigital[6] && !sensorDigital[7]) { error = 6; motorsToWork(100, 200, LOW, HIGH, HIGH, LOW);}
	else if ( sensorDigital[0] && !sensorDigital[1] && !sensorDigital[2] && !sensorDigital[3] && !sensorDigital[4] && !sensorDigital[5] && !sensorDigital[6] && !sensorDigital[7]) { error = 7; motorsToWork(255, 255, LOW, HIGH, HIGH, LOW);}
	//else if ( sensorDigital[0] && sensorDigital[1] && sensorDigital[2] && sensorDigital[3] && sensorDigital[4] && sensorDigital[5] && sensorDigital[6] && sensorDigital[7]) { error = 0; motorsToWork(200, 200, HIGH, LOW, HIGH, LOW);}
}

void sendByBluetooth(char data, boolean breakLine = false) {
	if (Bluetooth.available()) {
		if (breakLine) {Bluetooth.println(data);}
		else if (!breakLine) {Bluetooth.print(data);}
	}
}

void encodersToWork() {
	long newPositionA = myEncA.read();
	long newPositionB = myEncB.read();

	if (newPositionA != oldPositionA) {
		//Serial.println(newPositionA);
	}
	if (newPositionB != oldPositionB) {
		oldPositionB = newPositionB;
	}
  if ((newPositionA - oldPositionA) <= -35000 && (newPositionB - oldPositionB) <= -35000 ) {motorsToWork(0, 0, HIGH, HIGH, HIGH, HIGH);}
 
}

void loop() {
	//motorsToWork(80, 80, HIGH, LOW, HIGH, LOW);
  calculateError();
	encodersToWork();
}
