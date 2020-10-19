#include "MPU9250.h"
#include "IMU.h"
#include "config.h"
#include "data.h"

//define the subject
MPU9250 IMU = MPU9250(Wire, 0x68);  // three MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68

void IMU_init() {
  int status = IMU.begin();            // start communication with IMU
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
  // setting the accelerometer full scale range to +/-4G
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_1000DPS);
  //use 41Hz lowpass filter
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_5HZ);
  //set input datarate
  IMU.setSrd(4);
  //set the correct parm
  IMU_correction();
}

void IMU_correction() {

  /*int status = IMU.calibrateAccel();
  Serial.print("\n");
  Serial.println("POSITION");
  Serial.print(IMU.getAccelBiasX_mss(), 6);
  Serial.print("\t");
  Serial.print(IMU.getAccelBiasY_mss(), 6);
  Serial.print("\t");
  Serial.print(IMU.getAccelBiasZ_mss(), 6);
  Serial.print("\t");
  Serial.print(IMU.getAccelScaleFactorX(), 6);
  Serial.print("\t");
  Serial.print(IMU.getAccelScaleFactorY(), 6);
  Serial.print("\t");
  Serial.println(IMU.getAccelScaleFactorZ(), 6);
  Serial.println(status);
  status = IMU.calibrateGyro();
  Serial.println("POSITION");
  Serial.print("\n");
  Serial.print(IMU.getGyroBiasX_rads(), 6);
  Serial.print("\t");
  Serial.print(IMU.getGyroBiasY_rads(), 6);
  Serial.print("\t");
  Serial.println(IMU.getGyroBiasZ_rads(), 6);
  Serial.print(status);*/
  
  IMU.setGyroBiasX_rads(GyrobiasX);
  IMU.setGyroBiasY_rads(GyrobiasY);
  IMU.setGyroBiasZ_rads(GyrobiasZ);

}

void init_calculation() {
  if (this_process_time == 0)
  {
    last_process_time = micros();
    this_process_time = last_process_time + 1000;
  } else {
    last_process_time = this_process_time;
    this_process_time = micros();
  }
}

void IMU_read() {

  IMU.readSensor();

  accX = -IMU.getAccelX_mss();
  accY = -IMU.getAccelY_mss();
  accZ = -IMU.getAccelZ_mss();
  radX = IMU.getGyroX_rads();
  radY = IMU.getGyroY_rads();
  radZ = IMU.getGyroZ_rads();
  magX = IMU.getMagX_uT();
  magY = IMU.getMagY_uT();
  magZ = IMU.getMagZ_uT();
#if MegDebug == 0
  mag_cali();
#endif
}

void mag_cali() {

  float uncalibrated_values[3];

  uncalibrated_values[0] = magX;
  uncalibrated_values[1] = magY;
  uncalibrated_values[2] = magZ;

  for (int i = 0; i < 3; ++i) uncalibrated_values[i] = uncalibrated_values[i] - mag_bias[i];
  float result[3] = {0, 0, 0};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      result[i] += calibration_matrix[i][j] * uncalibrated_values[j];

  magX = result[0];
  magY = result[1];
  magZ = result[2];

}
