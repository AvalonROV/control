int mpuAddress = 0x68;  // MPU-6050 I2C address (0x68 or 0x69)
int accelOffset = -600;  // Accelerometer calibration value

int gyroPitchRaw, gyroYawRaw, accelRaw;
long gyroYawCalibration, gyroPitchCalibration;
float gyroAngle, accelAngle, angle;

//Accelerometer rolling average handlers
float accAvg[3] = {0, 0, 0};
int accAvgCounter = 0;
float accRollSum = 0;

int start = 0;

void mpuSetup() {
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

  // Gyroscope offset calibration over 500 samples
  for (int x = 0; x < 500; x++) {

    // ////MPU6050 interfacing
    Wire.beginTransmission(mpuAddress);  // Start communication with the gyro
    Wire.write(0x43);  // Start reading from gyro
    Wire.endTransmission();  // End the transmission
    Wire.requestFrom(mpuAddress, 4);  // Request 2 bytes from the gyro
    gyroYawCalibration += Wire.read() << 8 | Wire.read(); // Combine the two bytes to make one integer
    gyroPitchCalibration += Wire.read() << 8 | Wire.read(); // Combine the two bytes to make one integer
    delayMicroseconds(loopTime - 300);  // Wait for 3700 microseconds to simulate the main program loop time
  }
  gyroPitchCalibration /= 500;  // Divide the total value by 500 to get the avarage gyro offset
  Serial.println(gyroPitchCalibration);
  gyroYawCalibration /= 500;  // Divide the total value by 500 to get the avarage gyro offset
}

void fetch_pitch() {

  Wire.beginTransmission(mpuAddress);  // Start gyro communication
  Wire.write(0x43);  // Frequest from gyro at register 43 (roll -> pitch yaw)
  Wire.endTransmission();
  Wire.requestFrom(mpuAddress, 4);  // Get pitch and yaw from gyro
  gyroYawRaw = Wire.read() << 8 | Wire.read();
  gyroPitchRaw = Wire.read() << 8 | Wire.read();

  gyroPitchRaw -= gyroPitchCalibration;

  Serial.print(dT); Serial.print("\t");
  gyroAngle += gyroPitchRaw * (1 / ((1 / dT) * 1000000 * 131)); // Convert angle output to degrees

  Wire.beginTransmission(mpuAddress);
  Wire.write(0x3F);  // Request reading at register 3F
  Wire.endTransmission();
  Wire.requestFrom(mpuAddress, 2);  // Request 2 bytes from accel data
  accelRaw = Wire.read() << 8 | Wire.read();
  accelRaw += accelOffset;
  if (accelRaw > 8200)accelRaw = 8200;
  if (accelRaw < -8200)accelRaw = -8200;

  accRollSum -= accAvg[accAvgCounter];  // Remove oldest accel value from rolling sum
  accAvg[accAvgCounter] = asin((float)accelRaw / 8200.0) * 57.296; // Calculate accel angle
  accRollSum += accAvg[accAvgCounter];  //  Add to rolling sum
  accAvgCounter = (accAvgCounter + 1) % 3;  // Increment and modulo counter
  accelAngle = accRollSum / 3.0;  // Divide for rolling average

  if (start == 0 && accelAngle > -0.5 && accelAngle < 0.5) { // If not started and accel is nearly 0
    gyroAngle = accelAngle;  // Set gyro to accel (near reset gyro)
    start = 1;  // Set start
  }

  gyroAngle = gyroAngle * 0.9996 + accelAngle * 0.0004;  // Use accel to correct gyro drift
  Serial.println(gyroAngle);

  pitch = gyroAngle;
}
