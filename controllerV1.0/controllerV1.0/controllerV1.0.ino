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

int thrusterPin[2] = {9,10};

Servo up1,up2;
MS5803 sensor(0x76);

unsigned int currLoop = 0;
unsigned int prevLoop = 0;
int loopTime = 0;
float thrusterSpeeds[2] = {1500,1500}; //uf,ub,fl,fr,bl,br
double pressure;
float desiredPressure = 1030;
double pressureErrorMargin = 1.0;
float error[3] = {0,0,0}; //p,i,d
float k[3] = {0.05,0,0.1}; //p,i,d



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

void loop() { //persistentVariables[]{prevLoop,
  currLoop = millis();
  //Poll sensors (pressure and controller)
  fetch_depth();
  
  loopTime = currLoop - prevLoop;
  //calculations
  calculateError();
  int dT = currLoop - prevLoop;
  //PID
  thrusterSpeeds[0] += k[0]*error[0] + k[1]*error[1]*dT + k[2]*(error[2]/dT);
  thrusterSpeeds[1] += k[0]*error[0] + k[1]*error[1]*dT + k[2]*(error[2]/dT);
  Serial.println(); Serial.print(pressure); Serial.print("   "); Serial.print(thrusterSpeeds[1]);
  updateThrusters();
  prevLoop = currLoop;
}

float fetch_depth(){
  pressure = sensor.getPressure(ADC_4096);
}

void calculateError(){
  float prevError = error[0];
  error[0] = desiredPressure - pressure;
  error[1] += error[0];
  error[2] = error[0] - prevError;
}

void updateThrusters(){
  up1.writeMicroseconds(thrusterSpeeds[0]);
  up2.writeMicroseconds(thrusterSpeeds[1]);
}
