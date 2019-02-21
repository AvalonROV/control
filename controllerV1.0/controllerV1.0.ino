#include <Wire.h>


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
float depth = 0;
float currentThrusterSpeed[6] = {0,0,0,0,0,0}; //u,d,l,r,f,b
float desiredThrusterSpeed[6] = {0,0,0,0,0,0}; //u,d,l,r,f,b
float K[3] = {0.5,0,0};
float error[3] = {0,0,0}; //p,i,d


void setup() {
  for(int x = 0; x < 6; x ++){
    pinMode(thrusterPin[x],OUTPUT);
  }
  pinMode(pressurePin,INPUT);
}

void loop() {
  currLoop = millis();
  //Poll sensors (pressure and controller)
  getDepth();
  loopTime = currLoop - prevLoop;
  //calculations
  
  //PID
  
  //update thrusters
  prevLoop = currLoop;
}

void updateThrusters(){
  //update all thruster speeds
}

void getDepth(){
  
}

