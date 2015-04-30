#include <Arduino.h>
#include <math.h>

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define leftM 11
#define leftDir 13
#define rightM 3
#define rightDir 12
#define button 8

const int LOOP_TIME = 9;
int lastLoopT = LOOP_TIME;
int lastLoopTUSE = LOOP_TIME;
unsigned long loopStartT = 0;

MPU6050 imu;


/* Potentiometer Class
 *
 *     This class takes care of the initialization and
 *     data retrieval of a potentiometer attached to one
 *     of the analog input pins.
 *
 *     It provides the functionality to both return just
 *     the raw value, but also it will return a mapped value
 *     that takes the value read from the potentiometer and
 *     maps it to a range that is more suitable for use.
 *
 */
class Potentiometer{
public:
	Potentiometer(int pin){
		pinMode(pin, INPUT);
		_pot = pin;
		_reading = 0;
	}

	int getRaw(){
		_reading = analogRead(_pot);
		return _reading;
	}

	float getReading(int min, int max){
		return this->getCompensated((float) this->getRaw(), (float) min, (float) max);
	}

private:
	int _pot;
	int _reading;

	float getCompensated(float in, float outMin, float outMax){
		float inMin = 0, inMax = 1024;
		return outMin + (( (in - inMin) * (outMax-outMin) )/(inMax-inMin));
	}

};


/* IMUControl
 *   Responsible for handling data retrieval and calculation
 *   with regards to the MPU6050.  This class and its objects
 *   expect that the MPU6050 has already been properly initialized
 *   and that proper communication has been established.
 *
 *   NOTE that while DMP functionality is included within this class
 *   the data processed by DMP is not being used.  Through testing
 *   it was found that the program executed with more stability when the
 *   DMP was initialized even if data from it was not being used.
 *
 */
class IMUControl{
public:
	bool dmpReady;
	uint16_t packetSize;
	uint16_t fifoCount;
	uint8_t intStat;
	uint8_t fifoBuffer[64];

	Quaternion q;
	VectorInt16 aa;
	VectorInt16 aaReal;
	VectorFloat gravity;

	float euler[3];
	float ypr[3];

	int acc_angle;

	volatile bool data;

	void getMeasure(){
		imu.getMotion6(&_ax, &_ay, &_az, &_gx, &_gy, &_gz);
		data = false;
		intStat = imu.getIntStatus();
		fifoCount = imu.getFIFOCount();
		if((intStat & 0x10) || fifoCount == 1024){
			imu.resetFIFO();
			//overflow
		} else if(intStat & 0x02){
			while(fifoCount < packetSize) fifoCount = imu.getFIFOCount();

			imu.getFIFOBytes(fifoBuffer, packetSize);

			fifoCount -= packetSize;

			//			//---yaw pitch roll ---
			//			imu.dmpGetQuaternion(&q, fifoBuffer);
			//			imu.dmpGetGravity(&gravity, &q);
			//			imu.dmpGetYawPitchRoll(ypr, &q, &gravity);

			//			//---accel compensated for gravity --
			//			imu.dmpGetQuaternion(&q, fifoBuffer);
			//			imu.dmpGetAccel(&aa, fifoBuffer);
			//			imu.dmpGetGravity(&gravity, &q);
			//			imu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

		}

		_gx = _gx * 250.0/32768.0;
		acc_angle = this->arctan2(-_ay, -_az) -2; //-20 for _ay and _az
		Serial.print("acc / gx: \t"); Serial.print(acc_angle);
		Serial.print("\t"); Serial.print(_gx);


	}

	void calibrate(){
		int offset[6] = {0};
		this->mean(500);

		offset[0] = -_mean[0]/8;
		offset[1] = -_mean[1]/8;
		offset[2] = (16384-_mean[2])/4;

		offset[3] = -_mean[3]/4;
		offset[4] = -_mean[4]/4;
		offset[5] = -_mean[5]/4;

		imu.setXAccelOffset(offset[0]);
		imu.setYAccelOffset(offset[1]);
		imu.setZAccelOffset(offset[2]);
		imu.setXGyroOffset(offset[3]);
		imu.setYGyroOffset(offset[4]);
		imu.setZGyroOffset(offset[5]);


	}

	int filter(){
		this->getMeasure();
		return kalmanCalculate(acc_angle, _gx, LOOP_TIME);
	}

private:
	int16_t _ax = 0, _ay = 0, _az = 0, _gx = 0, _gy = 0, _gz = 0;
	float _mean[6];
	float Q_angle  =  0.001; //0.001
	float Q_gyro   =  0.003;  //0.003
	float R_angle  =  0.1;  //0.03

	float x_angle = 0;
	float x_bias = 0;
	float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
	float dt, y, S;
	float K_0, K_1;

	float kalmanCalculate(float newAngle, float newRate,int looptime) {
		dt = float(looptime)/1000;
		x_angle += dt * (newRate - x_bias);
		P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
		P_01 +=  - dt * P_11;
		P_10 +=  - dt * P_11;
		P_11 +=  + Q_gyro * dt;

		y = newAngle - x_angle;
		S = P_00 + R_angle;
		K_0 = P_00 / S;
		K_1 = P_10 / S;

		x_angle +=  K_0 * y;
		x_bias  +=  K_1 * y;
		P_00 -= K_0 * P_00;
		P_01 -= K_0 * P_01;
		P_10 -= K_1 * P_00;
		P_11 -= K_1 * P_01;

		return x_angle;
	}

	void mean(int sampleSize){
		int toss = 0.1 * sampleSize;
		long buff[6] = {0};
		int i = 0;
		while(i < (sampleSize + (toss + 1))){
			imu.getMotion6(&_ax, &_ay, &_az, &_gx, &_gy, &_gz);
			if (i < toss && i<=sampleSize+toss) {
				buff[0] = _ax;
				buff[1] = _ay;
				buff[2] = _az;
				buff[3] = _gx;
				buff[4] = _gy;
				buff[5] = _az;
			}

			if(i==(sampleSize+toss)){
				for(int x = 0; x < 6; x ++){
					_mean[x]= buff[x]/sampleSize;
				}
			}
			i++;
		}
	}

	int arctan2(int y, int x) {          // http://www.dspguru.com/comp.dsp/tricks/alg/fxdatan2.htm
		int coeff_1 = 128;                 // angle in Quids (1024 Quids=360¡) <<<<<<<<<<<<<<
		int coeff_2 = 3*coeff_1;
		float abs_y = abs(y)+1e-10;
		float r, angle;

		if (x >= 0) {
			r = (x - abs_y) / (x + abs_y);
			angle = coeff_1 - coeff_1 * r;
		}  else {
			r = (x + abs_y) / (abs_y - x);
			angle = coeff_2 - coeff_1 * r;
		}
		if (y < 0)      return int(-angle);
		else            return int(angle);
	}


}con;


/* MotorControl
 *   This class is responsible for handling the motor control
 *   and power calculation using PID.  This class works in
 *   conjunction with the Potentiometer class to set the values
 *   for the gain of each control type, kp, ki, and kd.
 *
 *   This class expects that the pins that will drive two motors
 *   for both direction and output, have already been set in the
 *   setup() function.
 *
 */
class MotorControl{
public:

	void checkMotor(){
		int tmp = con.filter();
		int pwm = this->pid(0, tmp);
//		Serial.print("pwm value: "); Serial.println(pwm);
		Serial.print("\t"); Serial.println(tmp);
		this->motor('L', pwm);
		this->motor('R', pwm);

	}

	int pid(int setPoint, int current){
		Potentiometer kpPot(0), kiPot(1), kdPot(2);

		_kp = kpPot.getReading(20, 30);
		_ki = kiPot.getReading(0, 10);
		_kd = kdPot.getReading(10, 20);
//
//		Serial.print(_kp);
//		Serial.print("\t"); Serial.print(_kd);
//		Serial.print("\t"); Serial.println(_ki);

		const int guard = 50;
		_error = setPoint - current;
		int p =_kp * _error;
		int intError = intError + _error;
		int i = _ki * constrain(intError, -guard, guard);
		int d = _kd * (_error - _lastE);
		_lastE = _error;

		int out = -constrain(_k*(p + i + d), -255, 255);
		if(out <= 30 && out >= 0){
			return 30;
		} else if(out < 0 && out >= - 30){
			return -30;
		} else {
			return out;
		}
	}


	/*
	 * Given a particular previous state passed to this function
	 * in chars the Stop will momentarily reverse the direction
	 * of the motors in order to prevent drifting from motor
	 * over spin.
	 *
	 * The chars passed to this function represent the previous
	 * state of the motor.
	 *   -- F ~> forward motor direction i.e. dir pin set HIGH
	 *   -- B ~> backward motor direction i.e. dir pin set LOW
	 *
	 * While these values for each char is hard coded in at the
	 * moment, it could be feasible to design the software to
	 * set the chars itself.
	 */
	void stop(char l, char r){
		int rt = 50;

		//both motors were previously set to forward
		if(l == 'F' && r == 'F'){
			//reverse direction
			digitalWrite(leftDir, LOW);
			digitalWrite(rightDir, LOW);

			analogWrite(leftM, 255);
			analogWrite(rightM, 255);

			delay(rt);

			//motors off
			analogWrite(leftM, 0);
			analogWrite(rightM, 0);

		} else if (l == 'F' && r == 'B'){
			digitalWrite(leftDir, LOW);
			digitalWrite(rightDir, HIGH);

			analogWrite(leftM, 255);
			analogWrite(rightM, 255);

			delay(rt);

			analogWrite(leftM, 0);
			analogWrite(rightM, 0);

		} else if( l == 'B' && r == 'F'){
			digitalWrite(leftDir, HIGH);
			digitalWrite(rightDir, LOW);

			analogWrite(leftM, 255);
			analogWrite(rightM, 255);

			delay(rt);

			analogWrite(leftM, 0);
			analogWrite(rightM, 0);

		}
	}

private:
	float _kp = 1; //value was 2.5
	float _ki = 0; //value was 3
	float _kd = 0; //values was -2.25
	float _k = 1; //value was 2.25

	int _error = 0;
	int _lastE = 0;

	//Motor A is the right motor
	//Motor B is the left motor
	void motor(char m, int pwm){
		//forward motor direction defined as positive power level
		//for reverse direction pass in a negative value
		if(m == 'L'){
			if(pwm > 0){
				digitalWrite(leftDir, HIGH);
				analogWrite(leftM, pwm);

			} else if(pwm < 0){
				digitalWrite(leftDir, LOW);
				analogWrite(leftM, -pwm);

			} else{
				analogWrite(leftM, 0);
			}
		}  // end motor A

		if(m == 'R'){
			if(pwm > 0){  //see motor A above
				digitalWrite(rightDir, HIGH);
				analogWrite(rightM, pwm);

			} else if(pwm < 0){
				digitalWrite(rightDir, LOW);
				analogWrite(rightM, -pwm);

			} else {
				analogWrite(rightM, 0);
			}
		}  // end motor B
	}

}m;


//interrupt for data ready to read
void dataReady(){
	con.data = true;
}


void setup(){
	Serial.begin(115200);
	bool ready = false;
	bool defaul = true;
	int counter = 0;
	attachInterrupt(0, dataReady, RISING);
	pinMode(button, INPUT_PULLUP);

	Wire.begin();
	imu.initialize();

	//Serial.println("init");

	uint8_t devStat = imu.dmpInitialize();

	//optional calibration...however its use typically
	//caused the Arduino to crash faster or more frequently.
	//If the button is not pushed within about 5 seconds
	//Arduino will go to the default values.
//	while(!ready){
//		if(digitalRead(button) == 0){
//			con.calibrate();
//			ready = true;
//			defaul = false;
//		} else if(counter >= 5000){
//			ready = true;
//		}
//		delay(1);
//		counter++;
//	}

	//default offset values
	//Arduino will default to these after a set amount of time

	if(defaul){
		imu.setXAccelOffset(1501);
		imu.setYAccelOffset(1251);
		imu.setZAccelOffset(1262);
		imu.setXGyroOffset(-4);
		imu.setYGyroOffset(4);
		imu.setZGyroOffset(4);
	}

	if(devStat == 0){
		imu.setDMPEnabled(true);
		con.dmpReady = true;

		con.packetSize = imu.dmpGetFIFOPacketSize();
	} else {
		//something has failed
	}

}


void loop(){
	if(!con.dmpReady) return;

	while(!con.data && con.fifoCount < con.packetSize){
		//Serial.println("Waiting in main loop for data");
		//this loop waits for the interrupt to be set high
	}
	//Serial.println("executing main loop");

	m.checkMotor();
	//Serial.print("motor pwm: "); Serial.println(pwm);

	con.data = false;

	lastLoopTUSE = millis() - loopStartT;
	if(lastLoopTUSE < LOOP_TIME){
		//this sets a certain run time for the loop
		//delay(LOOP_TIME - lastLoopTUSE);
	}
	lastLoopT = millis() - loopStartT;
	loopStartT = millis();
}
