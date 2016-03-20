// KasBot V1  -  Main module       basic version, angles in Quids, 10 bit ADC
//

#include <math.h>

#define   GYR_Y                 0                              // Gyro Y (IMU pin #4)
#define   ACC_Z                 1                              // Acc  Z (IMU pin #7)
#define   ACC_X                 2                              // Acc  X (IMU pin #9)

#define   InA_R                 6                              // INA right motor pin 
#define   InB_R                 7                              // INB right motor pin
#define   PWM_R                 10                             // PWM right motor pin
#define   InA_L                 8                              // INA left motor pin
#define   InB_L                 9                              // INB left motor pin
#define   PWM_L                 11                             // PWM left motor pin

int   STD_LOOP_TIME  =          9;             

int sensorValue[3]  = { 0, 0, 0};
int sensorZero[3]  = { 0, 0, 0 }; 
int lastLoopTime = STD_LOOP_TIME;
int lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime = 0;
int actAngle;                                                  // angles in QUIDS (360° = 2PI = 1204 QUIDS   <<<
int ACC_angle;
int GYRO_rate;
int setPoint = 0;
int drive = 0;

void setup() {
  analogReference(EXTERNAL);                                              // Aref 3.3V
  Serial.begin(115200);
  for(int pin=InA_R; pin<=PWM_L; pin++)    pinMode(pin, OUTPUT);         // set output mode
  delay(100);                                                
  calibrateSensors();
}

void loop() {
  
// ********************* Sensor aquisition & filtering *******************
  updateSensors();
  ACC_angle = getAccAngle();                                                 // in Quids
  GYRO_rate = getGyroRate();                                                 // in Quids/seconds
  actAngle = kalmanCalculate(ACC_angle, GYRO_rate, lastLoopTime);            // calculate Absolute Angle

// *********************** PID and motor drive *****************
  drive = updatePid(setPoint, actAngle);                                      // PID algorithm
  if(actAngle>(setPoint-15) && actAngle<(setPoint+15))     Drive_Motor(drive); 
  else                                                     Drive_Motor(0);     // stop motors if situation is hopeless

// *********************** loop timing control **************************
  lastLoopUsefulTime = millis()-loopStartTime;
  if(lastLoopUsefulTime<STD_LOOP_TIME)         delay(STD_LOOP_TIME-lastLoopUsefulTime);
  lastLoopTime = millis() - loopStartTime;
  loopStartTime = millis();
}
