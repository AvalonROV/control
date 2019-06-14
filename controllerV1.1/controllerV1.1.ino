#include <SparkFun_MS5803_I2C.h>
#include <Servo.h>
#include <Wire.h>

/*
   Underwater ROV controller
   Developed for: University of Sheffield AVALON ROV 2018/19
   Authored by: Dominic Rugg-Gunn Jan 2019
*/

/*
   =====Plan
   Read pressure sensor
   Calculate depth
   Get depth setpoint
   Calculate error
   P(ID) to get depth correct
   Tune and store in EEPROM
*/

//////////////////////////////////////////////////
unsigned long previousPidTime = 0;
//////////////////////////////////////////////////

int thrusterPin[2] = {9, 10};

Servo up1, up2;
MS5803 sensor(0x76);

unsigned long currLoop = 0;
unsigned long prevLoop = 0;
float dT = 0;
float pidDt = 0;

float thrusterSpeeds[2] = {1500, 1500}; //uf,ub,fl,fr,bl,br

double pressure;
float desiredPressure = 1040;
float errorDepth[3] = {0, 0, 0}; //p,i,d
float kDepth[3] = {12, 0.01, 0.2}; //p,i,d

double pitch;
float desiredPitch = 0;
float errorPitch[3] = {0, 0, 0};
float kPitch[3] = {0, 0, 0};

unsigned long loopTime = 5000;

void setup() {
  up1.attach(thrusterPin[0]);
  up1.writeMicroseconds(1500); // send "stop" signal to ESC.
  up2.attach(thrusterPin[1]);
  up2.writeMicroseconds(1500); // send "stop" signal to ESC.
  delay(1000);
  sensor.reset();
  sensor.begin();
  Serial.begin(38400);
  mpuSetup();
}

void loop() {
  //Time start
  currLoop = micros();
  dT = currLoop - prevLoop;

  //Program
  if (dT >= loopTime) {
    pidDt = dT / 1000.0;
    
    runPid();

    //Time end
    prevLoop = currLoop;
  }
}

void runPid() { //persistentVariables[]{prevLoop,

  //Poll sensors (pressure and controller)
  //fetch_pitch();
  fetch_depth();
  //calculations
  calculateErrorPitch();
  calculateErrorDepth();

  //PID
  thrusterSpeeds[0] = 1500 + ( kDepth[0] * errorDepth[0] + kDepth[1] * errorDepth[1] * pidDt + kDepth[2] * (errorDepth[2] / pidDt) ) + ( kPitch[0] * errorPitch[0] + kPitch[1] * errorPitch[1] * pidDt + kPitch[2] * (errorPitch[2] / pidDt));
  thrusterSpeeds[1] = 1500 + ( kDepth[0] * errorDepth[0] + kDepth[1] * errorDepth[1] * pidDt + kDepth[2] * (errorDepth[2] / pidDt) ) - ( kPitch[0] * errorPitch[0] + kPitch[1] * errorPitch[1] * pidDt + kPitch[2] * (errorPitch[2] / pidDt) );

  //Thrusters
  updateThrusters();
}

void fetch_depth() {
  pressure = sensor.getPressure(ADC_4096);
}

void calculateErrorDepth() {
  float prevError = errorDepth[0];
  errorDepth[0] = desiredPressure - pressure;
  errorDepth[1] += errorDepth[0];
  errorDepth[2] = errorDepth[0] - prevError;
  //Serial.print("\t");Serial.print(errorDepth[0]);
}

void calculateErrorPitch() {
  float prevError = errorPitch[0];
  errorPitch[0] = desiredPitch - pitch;
  errorPitch[1] += errorPitch[0];
  errorPitch[2] = errorPitch[0] - prevError;
  //Serial.print("\t");Serial.print(errorPitch[0]);
}

float limit(float toLimit, int upLim, int lowLim) {
  if (toLimit > upLim) {
    return upLim;
  }
  else if (toLimit < lowLim) {
    return lowLim;
  }
  else {
    return toLimit;
  }
}

void updateThrusters() {
  for (int x = 0; x < 2; x++) {
    thrusterSpeeds[x] = limit(thrusterSpeeds[x], 1900, 1100);
  }
  up1.writeMicroseconds(int(thrusterSpeeds[0]));
  up2.writeMicroseconds(int(thrusterSpeeds[1]));
}
