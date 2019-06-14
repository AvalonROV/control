#include <SparkFun_MS5803_I2C.h>
#include <Servo.h>

/*
 * Underwater ROV controller
 * Developed for: University of Sheffield AVALON ROV 2018/19
 * Authored by: Dominic Rugg-Gunn Jan 2019
 */

/*
 * =====Plan
 * Read pressure sensor
 * Calculate depth
 * Get depth setpoint
 * Calculate error
 * P(ID) to get depth correct
 * Tune and store in EEPROM
 */

//////////////////////////////////////////////////
unsigned long previousPidTime = 0;
//////////////////////////////////////////////////

int thrusterPin[2] = {9,10};

Servo up1,up2;
MS5803 sensor(0x76);

unsigned long currLoop = 0;
unsigned long prevLoop = 0;
float dT = 0;

float thrusterSpeeds[2] = {1500,1500}; //uf,ub,fl,fr,bl,br

double pressure;
float desiredPressure = 1040;
float errorDepth[3] = {0,0,0}; //p,i,d
float kDepth[3] = {12,0.01,0.2}; //p,i,d

double pitch;
float desiredPitch = 0;
float errorPitch[3] = {0,0,0};
float kPitch[3] = {0,0,0};



void setup() {
  up1.attach(thrusterPin[0]);
  up1.writeMicroseconds(1500); // send "stop" signal to ESC.
  up2.attach(thrusterPin[1]);
  up2.writeMicroseconds(1500); // send "stop" signal to ESC.
  delay(1000);
  sensor.reset();
  sensor.begin();
  Serial.begin(115200);
}

void loop(){
  test(&previousPidTime);
  delay(10000);
}

void test(unsigned long prevTimePointer){
  Serial.println(*prevTimePointer);
}

void runPid() { //persistentVariables[]{prevLoop,
  currLoop = millis();
  //Poll sensors (pressure and controller)
  fetch_pitch();
  fetch_depth();
  //calculations
  calculateErrorPitch();
  calculateErrorDepth();
  dT = currLoop - prevLoop;
  //PID
  thrusterSpeeds[0] = 1500 + ( kDepth[0]*errorDepth[0] + kDepth[1]*errorDepth[1]*dT + kDepth[2]*(errorDepth[2]/dT) ) + ( kPitch[0]*errorPitch[0] + kPitch[1]*errorPitch[1]*dT + kPitch[2]*(errorPitch[2]/dT) );
  thrusterSpeeds[1] = 1500 + ( kDepth[0]*errorDepth[0] + kDepth[1]*errorDepth[1]*dT + kDepth[2]*(errorDepth[2]/dT) ) - ( kPitch[0]*errorPitch[0] + kPitch[1]*errorPitch[1]*dT + kPitch[2]*(errorPitch[2]/dT) );
  for(int x = 0; x < 2; x++){
    thrusterSpeeds[x] = limit(thrusterSpeeds[x],1900,1100);
  }
  //Serial.println(); Serial.print(map(pressure,1030,1000,0,30)); //Serial.print("\t"); Serial.print(thrusterSpeeds[0]);
  Serial.println(); Serial.print(pressure);
  updateThrusters();
  prevLoop = currLoop;
  //Serial.print("\t");Serial.print(dT); //current sample rate = 25ms
  //delay(25);
}

float limit(float toLimit,int upLim,int lowLim){
  if(toLimit>upLim){
    return upLim;
  }
  else if(toLimit<lowLim){
    return lowLim;
  }
  else{
    return toLimit;
  }
}

void fetch_depth(){
  pressure = sensor.getPressure(ADC_4096);
}

void fetch_pitch(){
  pitch = 0; //TODO: get from mpu
}

void calculateErrorDepth(){
  float prevError = errorDepth[0];
  errorDepth[0] = desiredPressure - pressure;
  errorDepth[1] += errorDepth[0];
  errorDepth[2] = errorDepth[0] - prevError;
  //Serial.print("\t");Serial.print(errorDepth[0]);
}

void calculateErrorPitch(){
  float prevError = errorPitch[0];
  errorPitch[0] = desiredPitch - pitch;
  errorPitch[1] += errorPitch[0];
  errorPitch[2] = errorPitch[0] - prevError;
  //Serial.print("\t");Serial.print(errorPitch[0]);
}

void updateThrusters(){
  up1.writeMicroseconds(int(thrusterSpeeds[0]));
  up2.writeMicroseconds(int(thrusterSpeeds[1]));
}
