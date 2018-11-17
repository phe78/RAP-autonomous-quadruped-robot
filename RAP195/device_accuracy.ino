#ifdef DEVICE_ACCURACY

#define X_MIN 0
#define X_MAX 1
#define Y_MIN 2
#define Y_MAX 3
#define Z_MIN 4
#define Z_MAX 5

#define LIMIT 32767

void setupDeviceAccuracy() {
  
  // Setup accelerometer
  int accelRate =
    //    0x00 // off
    //    0x10 // 13 Hz
    //    0x20 // 26 Hz
    0x30 // 52 Hz
    //    0x40 // 104 Hz
    //    0x50 // 208 Hz
    //    0x60 // 416 Hz
    //    0x70 // 833 Hz
    //    0x80 // 1.66 kHz
    //    0x90 // 3.33 kHz
    //    0xa0 // 6.66 kHz
    ;
  int accelScale =
    0x0 // +/-2g
    //    0x8 // +/-4g
    //    0xc // +/-8g
    //    0x4 // +/-16g
    ;
  int antiAliasingFilter =
    //    0x0 // 400 Hz
    //    0x1 // 200 Hz
    0x2 // 100 Hz
    //    0x3 // 50 Hz
    ;

  imu.writeReg(LSM6::CTRL1_XL, accelRate | accelScale | antiAliasingFilter);

  // Gyro
  int gyroRate =
    //    0x00 // off
    //    0x10 // 13 Hz
    //    0x20 // 26 Hz
    //    0x30 // 52 Hz
    0x40 // 104 Hz
    //    0x50 // 208 Hz
    //    0x60 // 416 Hz
    //    0x70 // 833 Hz
    //    0x80 // 1.66 kHz
    ;

  int gyroScale =
    //    0x0 // 245 dps
    //    0x8 // 500 dps
    0xc // 1000 dps
    //    0x4 // 2000 dps
    ;
  int gyroScale125 =
    0x0 // off
    //    0x1 // on
    ;

  imu.writeReg(LSM6::CTRL2_G, gyroRate | accelScale | gyroScale125);
}

void printDeviceAccuracy() {
  static int accel[6] = {LIMIT, -LIMIT, LIMIT, -LIMIT, LIMIT, -LIMIT};
  static int gyro[6] = {LIMIT, -LIMIT, LIMIT, -LIMIT, LIMIT, -LIMIT};

  countRange(accel, imu.a.x, imu.a.y, imu.a.z);
  printCurrentMinMax(imu.a.x, accel[X_MIN], accel[X_MAX]);
  printCurrentMinMax(imu.a.y, accel[Y_MIN], accel[Y_MAX]);
  printCurrentMinMax(imu.a.z, accel[Z_MIN], accel[Z_MAX]);

  countRange(gyro, imu.g.x, imu.g.y, imu.g.z);
  printCurrentMinMax(imu.g.x, gyro[X_MIN], gyro[X_MAX]);
  printCurrentMinMax(imu.g.y, gyro[Y_MIN], gyro[Y_MAX]);
  printCurrentMinMax(imu.g.z, gyro[Z_MIN], gyro[Z_MAX]);
}

void countRange(int result[6], int x, int y, int z) {
  result[X_MIN] = min(result[X_MIN], x);
  result[X_MAX] = max(result[X_MAX], x);
  result[Y_MIN] = min(result[Y_MIN], y);
  result[Y_MAX] = max(result[Y_MAX], y);
  result[Z_MIN] = min(result[Z_MIN], z);
  result[Z_MAX] = max(result[Z_MAX], z);
}

void printCurrentMinMax(int c, int mi, int mx) {
  Serial.print(c);
  Serial.print(" [");
  Serial.print(mi);
  Serial.print("..");
  Serial.print(mx);
  Serial.println("]");
}
#endif
