#include <Arduino.h>
#include <math.h>

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Potentiometer.h"

#define leftM 9
#define leftDir 7
#define rightM 10
#define rightDir 8
#define fault 12
#define enable 4
#define LED 13


const int LOOP_TIME = 9;
int lastLoopT = LOOP_TIME;
int lastLoopTUSE = LOOP_TIME;
unsigned long loopStartT = 0;

MPU6050 imu;


/* Accelerometer control
 *   Responsible for handling data retrieval and calculation
 *   with regards to the MPU6050.  This class and its objects
 *   expect that the MPU6050 has already been properly initialized
 *   and that proper communication has been established.
 *
 */
class Accelerometer{
public:
	int acc_angle;

	void getMeasurement(){
		imu.getMotion6(&_ax, &_ay, &_az, &_gx, &_gy, &_gz);

		_gx = _gx * 250.0/32768.0;
		acc_angle = this->arctan2(-_az, -_ay) -38; //-20 for _ay and _az
		Serial.print("acc / gx: \t"); Serial.print(acc_angle);
		Serial.print("\t"); Serial.println(_gx);


	}

	//optional calibration algorithm
	//to set the offsets for desired orientation
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

	//call to the Kalman filter
	//this includes a "filter" which throws out
	//-332 and -68 which came up every so often in
	//the output from the IMU for reasons unknown
	//IDEA: Does the Kalman filter need to know the time
	//it takes to do the measurements or just the
	//overall run time?
	int filter(){
		digitalWrite(LED, HIGH);
		this->getMeasurement();
		digitalWrite(LED, LOW);

		if(acc_angle ==-332 && _gx==-68){
			this->filter();
		}

		// -- DEADBAND --

		if(acc_angle <=5 && acc_angle > 0){
			acc_angle = 11;
		} else if(acc_angle >=-5 && acc_angle < 0){
			acc_angle = -11;
		}

		// --------------

		return kalmanFilter(acc_angle, _gx, LOOP_TIME);
	}

private:
	int16_t _ax = 0, _ay = 0, _az = 0, _gx = 0, _gy = 0, _gz = 0;

	float _mean[6];

	float Q_angle  =  0.001; //0.001
	float Q_gyro   =  0.003;  //0.003
	float R_angle  =  0.03;  //0.03

	float x_angle = 0;
	float x_bias = 0;
	float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
	float dt, y, S;
	float K_0, K_1;

	float kalmanFilter(float newAngle, float newRate,int looptime) {
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
		int i = 0;

		long buff[6] = {0};

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
		int coeff_1 = 128;
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

		if (y < 0){
			return int(-angle);
		} else {
			return int(angle);
		}
	}


}accelControl;


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
 *   IDEA: added pin initialization to the constructor for this
 *         class
 *
 */
class MotorControl{
public:

	void updateMotor(){
		digitalWrite(enable, HIGH);
		this->checkFault();
		int pwm = -this->pidControl(0, accelControl.filter());

//		Serial.print("pwm value: "); Serial.println(pwm);
//		Serial.print("Filter value: "); Serial.println(tmp);

		this->motor('L', pwm);
		this->motor('R', pwm);

	}

	int pidControl(int setPoint, int current){
		Potentiometer kPot(3);
		Potentiometer kpPot(2);
//		Potentiometer kdPot(3);
//		Potentiometer kiPot(3);


		_k = kPot.getReading(1, 10);
		_kp = kpPot.getReading(1, 10);
		float gain = 0.65 * _kp;
		_kd = 0;//kdPot.getReading(-gain, gain); //was at 1.81
		_ki = 0;//kiPot.getReading(0, 10);

//		Serial.print(_k);
//		Serial.print("\t"); Serial.print(_kp);
//		Serial.print("\t"); Serial.print(_kd);
//		Serial.print("\t"); Serial.println(_ki);

		const int guard = 25;

		_error = setPoint - current;

		int p =_kp * _error;

		int intError = intError + _error;
		int i = _ki * constrain(intError, -guard, guard);

		int d = _kd * (_error - _lastE);

		_lastE = _error;


		int out = -constrain(_k*(p + i + d), -255, 255);
		if(out <= 5 && out >=0){
			out = 5;
			return out;
		} else if(out >= -5 && out <= 0){
			out = -5;
			return out;
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
	float _kp = 1;
	float _ki = 0;
	float _kd = 0;
	float _k = 1;

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

	//function specific to implementation of Polulu driver
	//board
	void checkFault(){
		if(digitalRead(fault)==0){
			while(true);
		}
	}

}motor;


void setup(){
	Serial.begin(115200);
	bool ready = false;
	bool defaul = true;
	int counter = 0;

	pinMode(enable, OUTPUT);
	pinMode(fault, INPUT);
	pinMode(LED, OUTPUT);

	Wire.begin();
	TWBR = 24;
	imu.initialize();

//	Serial.println("init");

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

}


void loop(){
//	Serial.println("executing main loop");

	motor.updateMotor();

	//timing stuff for use in the Kalman filter
	lastLoopTUSE = millis() - loopStartT;
	if(lastLoopTUSE < LOOP_TIME){
		//this sets a certain run time for the loop
		delay(LOOP_TIME - lastLoopTUSE);
	}
	lastLoopT = millis() - loopStartT;
	loopStartT = millis();
}
