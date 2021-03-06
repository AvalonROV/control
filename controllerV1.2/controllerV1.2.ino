#include <SparkFun_MS5803_I2C.h>
#include <Servo.h>
#include <Wire.h>

void setup() {
    upFront.attach(9); //pin for front thruster
    upFront.writeMicroseconds(1500); // send "stop" signal to ESC.
    upBack.attach(10); //pin for back thruster
    upBack.writeMicroseconds(1500); // send "stop" signal to ESC.
    delay(5000);
    sensor.reset();
    sensor.begin();
    Serial.begin(9600);
    Serial.println(); Serial.println();
    
    struct pidVar persistentPidVar;
    struct accelerometerValues accelVal;
    struct gyroscopeValues gyroVal;
    
    persistentPidVar.previousPidTime = millis();
    persistentPidVar.prevDepthError[0] = 0;
    persistentPidVar.prevDepthError[1] = 0;
    persistentPidVar.prevPitchError[0] = 0;
    persistentPidVar.prevPitchError[1] = 0;
    
    ////PID VALUES HERE
    persistentPidVar.kDepth[0] = 0;//12;
    persistentPidVar.kDepth[1] = 0;//0.01;
    persistentPidVar.kDepth[2] = 0;//0.2;
    persistentPidVar.kPitch[0] = 1;
    persistentPidVar.kPitch[1] = 0;
    persistentPidVar.kPitch[2] = 0;
    persistentPidVar.hold[0] = true;
    persistentPidVar.hold[1] = false;
    persistentPidVar.thrusterSpeeds[0] = 1500;
    persistentPidVar.thrusterSpeeds[1] = 1500;
    
    Wire.beginTransmission(0x68);
    Wire.write(0x3F);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 2);
    accelVal.accelOffset = -95; //(Wire.read()<<8|Wire.read())*-1;
    Serial.print("accelOffset = "); Serial.println(accelVal.accelOffset);
    
    while (1) {
        pidMain(&persistentPidVar, &accelVal, &gyroVal);
    }
}

struct pidVar {
    float previousPidTime;
    float prevDepthError[2]; //prev, cumulative
    float prevPitchError[2]; //prev, cumulative
    float kDepth[3]; //p,i,d
    float kPitch[3];
    bool hold[2];
    float thrusterSpeeds[2];
};

struct accelerometerValues {
    float accAvg[3] = {0, 0, 0};
    int accAvgCounter = 0;
    float accRollSum = 0;
    int accelOffset = 0;  // Accelerometer calibration value
    float accelAngle;
};

struct gyroscopeValues {
    long gyroYawCalibration;
    long gyroPitchCalibration;
    float gyroAngle = 0;
};

Servo upFront, upBack;
MS5803 sensor(0x76);
int mpuAddress = 0x68;  // MPU-6050 I2C address (0x68 or 0x69)

float desiredPressure;
float desiredPitch;

unsigned long loopTime = 25500;
uint64_t prevTime = 0;
int calibrateGyro = 1;
int start = 0;

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

void updateThrusters(struct pidVar *pPidVar) {
    Serial.print(pPidVar->thrusterSpeeds[0]+1550); Serial.print("\t");
    Serial.print(pPidVar->thrusterSpeeds[1]+1550); Serial.print("\t"); Serial.print("\t");
    upFront.writeMicroseconds((int)limit((int)pPidVar->thrusterSpeeds[0]+1550, 1900, 1100));
    upBack.writeMicroseconds((int)limit((int)pPidVar->thrusterSpeeds[0]+1550, 1900, 1100));
}

float fetch_depth() {
    return (float)sensor.getPressure(ADC_4096);
}

float fetch_pitch(struct accelerometerValues* aVal, struct gyroscopeValues* gVal, float dT) {
    
    Wire.beginTransmission(mpuAddress);  // Start gyro communication
    Wire.write(0x43);  // Frequest from gyro at register 43 (roll -> pitch yaw)
    Wire.endTransmission();
    Wire.requestFrom(mpuAddress, 4);  // Get pitch and yaw from gyro
    int gyroYawRaw = Wire.read() << 8 | Wire.read();
    int gyroPitchRaw = Wire.read() << 8 | Wire.read();
    
    gyroPitchRaw -= (int)gVal->gyroPitchCalibration;
    
    gVal->gyroAngle += (float)(gyroPitchRaw) * (1.0 / ((1.0 / dT) * 1000000.0 * 131.0)); // Convert angle output to degrees

    
    Wire.beginTransmission(mpuAddress);
    Wire.write(0x3F);  // Request reading at register 3F
    Wire.endTransmission();
    Wire.requestFrom(mpuAddress, 2);  // Request 2 bytes from accel data
    int accelRaw = Wire.read() << 8 | Wire.read();
    accelRaw += aVal->accelOffset;
    if (accelRaw > 8200)accelRaw = 8200;
    if (accelRaw < -8200)accelRaw = -8200;
    // ////
    
    aVal->accRollSum -= aVal->accAvg[aVal->accAvgCounter];  // Remove oldest accel value from rolling sum
    aVal->accAvg[aVal->accAvgCounter] = (float)(asin((float)accelRaw / 8200.0) * 57.296);  // Calculate accel angle
    aVal->accRollSum += aVal->accAvg[aVal->accAvgCounter];  //  Add to rolling sum
    aVal->accAvgCounter = (aVal->accAvgCounter + 1) % 3;  // Increment and modulo counter
    aVal->accelAngle = (float)(aVal->accRollSum / 3.0);  // Divide for rolling average
    
    if (start == 0 && aVal->accelAngle > -0.5 && aVal->accelAngle < 0.5) { // If not started and accel is nearly 0
        gVal->gyroAngle = aVal->accelAngle;  // Set gyro to accel (near reset gyro)
        start = 1;  // Set start
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //MPU-6050 offset compensation
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    
    // ////MPU6050 interfacing
    gVal->gyroAngle = (float)(gVal->gyroAngle * 0.98 + aVal->accelAngle * 0.02);  // Use accel to correct gyro drift
    
    return gVal->gyroAngle;
}

void mpuSetup(struct gyroscopeValues* gVal, unsigned long loopTime)
{
    Wire.begin();  // Start wire I2C bus in master mode
    TWBR = 12;  // Set clock to 400kHz for wire
    
    // Start mpu6050
    Wire.beginTransmission(mpuAddress);
    Wire.write(0x6B);  // Request to write to PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);  // Set register bits to 00000000 - activate gyro
    Wire.endTransmission();
    
    // Set mpu scale +/- 250 deg/sec
    Wire.beginTransmission(mpuAddress);
    Wire.write(0x1B);  // Request to write to GYRO_CONFIG register (1B hex)
    Wire.write(0x00);  // Set register bits to 00000000 (250dps full scale)
    Wire.endTransmission();
    
    // Set accelerometer scale to +/- 4g.
    Wire.beginTransmission(mpuAddress);
    Wire.write(0x1C);  // Request to write to ACCEL_CONFIG register (1A hex)
    Wire.write(0x08);  // Set register bits to 00001000 (+/- 4g full scale)
    Wire.endTransmission();
    
    // Set filtering to improve the raw data.
    Wire.beginTransmission(mpuAddress);
    Wire.write(0x1A);  // Request to write to CONFIG register (1A hex)
    Wire.write(0x03);  // Set register bits to 00000011 (Digital Low Pass Filter ~43Hz)
    Wire.endTransmission();
    // ////
    
    // Gyroscope offset calibration over 500 samples
    for (int x = 0; x < 500; x++) {
        
        // ////MPU6050 interfacing
        Wire.beginTransmission(mpuAddress);  // Start communication with the gyro
        Wire.write(0x43);  // Start reading from gyro
        Wire.endTransmission();  // End the transmission
        Wire.requestFrom(mpuAddress, 4);  // Request 2 bytes from the gyro
        gVal->gyroYawCalibration += Wire.read() << 8 | Wire.read(); // Combine the two bytes to make one integer
        gVal->gyroPitchCalibration += Wire.read() << 8 | Wire.read(); // Combine the two bytes to make one integer
        delayMicroseconds(loopTime - 300);  // Wait for 3700 microseconds to simulate the main program loop time
    }
    gVal->gyroPitchCalibration /= 500;  // Divide the total value by 500 to get the avarage gyro offset
    //Serial.println(gVal->gyroPitchCalibration);
    gVal->gyroYawCalibration /= 500;  // Divide the total value by 500 to get the avarage gyro offset
    Serial.println("MPU calibration complete");
}

void runPid(struct pidVar* pPidIO, struct accelerometerValues* pAccelVal, struct gyroscopeValues* pGyroVal, float dT)
{
    if (pPidIO->hold[0] == false)
    {
        pPidIO->hold[1] = pPidIO->hold[0];
        return;
    }
    if (pPidIO->hold[0] == true && pPidIO->hold[1] == false)
    {
        desiredPressure = 1020;//fetch_depth();
        desiredPitch = 0;//fetch_pitch(pAccelVal, pGyroVal, dT);
        //Serial.println("desired values set");
        pPidIO->hold[1] = pPidIO->hold[0];
        return;
    }
    if (pPidIO->hold[0] == true && pPidIO->hold[1] == true)
    {
        //Poll sensors (pressure and controller)
        float pitch = fetch_pitch(pAccelVal, pGyroVal, dT);
        Serial.print(pitch); Serial.print("\t");
        float pressure = fetch_depth();
        Serial.println(pressure);
        float errorDepth[3];
        float errorPitch[3];
        
        //error calculations
        //depth
        errorDepth[0] = desiredPressure - pressure;
        pPidIO->prevDepthError[1] = errorDepth[1] = limit(pPidIO->prevDepthError[1] + errorDepth[0],200,-200);
        errorDepth[2] = errorDepth[0] - pPidIO->prevDepthError[0];
        pPidIO->prevDepthError[0] = errorDepth[0];
        //pitch
        
        errorPitch[0] = desiredPitch - pitch;
        
        pPidIO->prevPitchError[1] = errorPitch[1] = limit(pPidIO->prevPitchError[1] + errorPitch[0],200,-200);
        errorPitch[2] = errorPitch[0] - pPidIO->prevPitchError[0];
        pPidIO->prevPitchError[0] = errorPitch[0];
        
        float pidDt = dT / 1000;
        
        //PID output handling
        //Serial.print(errorPitch[0]); Serial.print(pPidIO->prevPitchError[1]); Serial.println();
        pPidIO->thrusterSpeeds[0] = ((pPidIO->kDepth[0] * errorDepth[0] + pPidIO->kDepth[1] * errorDepth[1] * pidDt + pPidIO->kDepth[2] * (errorDepth[2] / pidDt)) + (pPidIO->kPitch[0] * errorPitch[0] + pPidIO->kPitch[1] * errorPitch[1] * pidDt + pPidIO->kPitch[2] * (errorPitch[2] / pidDt)));
        pPidIO->thrusterSpeeds[1] = ((pPidIO->kDepth[0] * errorDepth[0] + pPidIO->kDepth[1] * errorDepth[1] * pidDt + pPidIO->kDepth[2] * (errorDepth[2] / pidDt)) - (pPidIO->kPitch[0] * errorPitch[0] + pPidIO->kPitch[1] * errorPitch[1] * pidDt + pPidIO->kPitch[2] * (errorPitch[2] / pidDt)));
        //Serial.print("Done PID   "); Serial.print(pPidIO->thrusterSpeeds[0]); Serial.print("   "); Serial.print(pPidIO->thrusterSpeeds[1]); Serial.println("   ");
    }
}

void pidMain(struct pidVar* pPersPVar, struct accelerometerValues* pPersAcVal, struct gyroscopeValues* pPersGyVal) {
    uint64_t tmpTime = micros();
    float dT = tmpTime - prevTime;
    
    if (dT >= loopTime) {
        prevTime = tmpTime;
        if (calibrateGyro == 0)
        {
            runPid(pPersPVar, pPersAcVal, pPersGyVal, dT);
            
            updateThrusters(pPersPVar);
        }
        else
        {
            mpuSetup(pPersGyVal, loopTime);
            calibrateGyro = 0;
            prevTime = micros();
        }
    }
}
