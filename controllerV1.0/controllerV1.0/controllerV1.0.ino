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

int thrusterPin[6] = {0,1,2,3,4,5};
int pressurePin = 6;

unsigned int currLoop = 0;
unsigned int prevLoop = 0;
int loopTime = 0;
float thrusterSpeeds[6] = {}; //uf,ub,fl,fr,bl,br
float k[3] = {0.5,0,0};
float desiredDepth = 0;
float error[3] = {0,0,0}; //p,i,d



void setup() {
  for(int x = 0; x < 6; x ++){
    pinMode(thrusterPin[x],OUTPUT);
  }
  pinMode(pressurePin,INPUT);
  Serial.begin(115200);
}

void loop(float* persistentVariables) { //persistentVariables[]{prevLoop,
  currLoop = millis();
  //Poll sensors (pressure and controller)
  getDepth();
  
  loopTime = currLoop - prevLoop;
  //calculations
  calculateError();
  
  //PID
  thrusterSpeeds[0] += k[0]*error[0] + dT*k[1]*error[1] + iT*k[2]*error[2];
  thrusterSpeeds[1] += k[0]*error[0] + dT*k[1]*error[1] + iT*k[2]*error[2];
  
  updateThrusters();
  prevLoop = currLoop;
}

float fetch_depth(){
  //
}

void calculateError(){
  
}

void updateThrusters(){
  //update all thruster speeds
}
