#include <QTRSensors.h>
//#include <RotaryEncoder.h> //https://github.com/mathertel/RotaryEncoder/

QTRSensors qtr;

//Motor Setup
#define motorAs1 8
#define motorAs2 7

#define motorBs1 12
#define motorBs2 13

#define pwmA 10
#define pwmB 11

#define encoderA1 2
#define encoderA2 3
#define encoderB1 4
#define encoderB2 5

//RotaryEncoder encoderA(encoderA1, encoderA2);
//RotaryEncoder encoderB(encoderB1, encoderB2);

#define Kp 1
#define Kd 4

#define speedBase 100
#define speedMax 150

uint16_t sensorValues[8];
unsigned int sensorsArray[8];
const uint8_t pins[8] = { A0, A1, A2, A3, A4, A5, A6, A7 }; //Setting all the pins to an array variable to use it on qtr.setSensorPins();

int lastError = 0;

void setup() {
	pinMode(LED_BUILTIN, OUTPUT);
	
	qtr.setTypeRC(); //Set the sensor type to RC (sensor array with lenght is eight, if you uses the array with 6 sensors the type is Analog) 
	qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, 8);
	//qtr.setEmitterPin(); It set the emitter pin which indicates if the IR Leds are on or off  

	digitalWrite(LED_BUILTIN, HIGH); //Turn on Arduino's LED to indicate we are in calibration mode

	//QTRRC the read timeout (default) is 2,5ms. 2.5ms * 10reads per calibrate() call
	// = ~25ms per calibrate() call
	//Call calibrate() 400 times to make calibration take about 10 seconds.

	for (uint16_t i = 0; i < 400; i++) { //It call calibrate() 400 times to calibrate all sensors, and it takes 10seconds
		qtr.calibrate(); 
	} digitalWrite(LED_BUILTIN, LOW); //Turn off Arduino's LED to indicate we are through with calibration 


	//All the code from here is just see numbers. Not afect the car
	Serial.begin(9600);

	Serial.print("Minimum values:	");
	for (uint8_t i = 0; i < 8; i++) {
		Serial.print(qtr.calibrationOn.minimum[i]); Serial.print(" "); //It will show the minimum calibration values of all 8 sensors
	} Serial.println();

	Serial.print("Maximum values:	");
	for (uint8_t i = 0; i < 8; i++) {
		Serial.print(qtr.calibrationOn.maximum[i]); Serial.print(" "); //It shows the maximum calibration values of each sensor
	} for (int i = 0; i < 2; i++) {Serial.println();}

	delay(1000); 
}

void loop() {
	int position = arrayToWork(sensorValues);
	Serial.println("Position: " + String(position)); //Debugs position

}

int arrayToWork(uint16_t sensors[]) {
	uint16_t position = qtr.readLineWhite(sensors); //read calibrated sensor values and obtain a measure of the line position

	//	print the sensor values as numbers from 0 to 1000, where 0 means maximum
	//	reflectance and 1000 means minumum reflectance, followed by the line
	for (uint8_t i = 0; i < 8; i++) {Serial.print(sensors[i]); Serial.print("\t");}

	calculatePID(position);

	return(position);
}

void calculateDecoders() {
	static int posA, posB = 0;

	int newPosA, newPosB = encoderA.getPosition(), encoderB.getPosition();

	if (posA != newPosA && posB != newPosB) {
		Serial.println("Position A: " + String(newPosA));
		Serial.println("Position B: " + String(newPosB));

		posA, posB = newPosA, newPosB;
	}

	/*
	I need to see how it works, but I already have an idea of it. 
	I will add to stop with interruptor
	*/

}

void motorsToWork(int speedMotorA, int speedMotorB, int valueA1 = HIGH, int valueA2 = LOW, int valueB1 = HIGH, int valueB2 = LOW) {
	digitalWrite(motorAs1, valueA1);
	digitalWrite(motorAs2, valueA2);
	digitalWrite(motorBs1, valueB1);
	digitalWrite(motorBs2, valueB2);

	analogWrite(pwmA, speedMotorA);
	analogWrite(pwmB, speedMotorB);
}

void calculatePID(int pos) {
  int error = pos - 3500;
  int speedMotor = (Kp * error) + (Kd * (error - lastError));
  lastError = error;

  int speedA = speedBase + speedMotor;
  int speedB = speedBase - speedMotor;

  if (speedA > speedMax) { speedA = speedMax; }
  else if (speedA < 0) { speedA = 0; } 
  else if (speedB > speedMax) { speedB = speedMax; }
  else if (speedB < 0) { speedB = 0; }

  if (pos > 6700) { motorsToWork(speedA, speedB, HIGH, LOW, LOW, HIGH); }  //It was at previous code
  else if (pos < 300) { motorsToWork(speedA, speedB, LOW, HIGH, HIGH, LOW); } //It was at previous code
  else { motorsToWork(speedA, speedB, HIGH, LOW, HIGH, LOW); } //Go foward
}