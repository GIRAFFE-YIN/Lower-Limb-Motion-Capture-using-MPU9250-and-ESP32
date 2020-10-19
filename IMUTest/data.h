#ifndef _DATA_H
#define _DATA_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <WiFi.h>


#define PACKET_SIZE 16

#define IMUS  3

#define Debug  0

#define MegDebug  0

extern unsigned char sendData[PACKET_SIZE];

extern const int localPort;

extern unsigned long last_process_time;
extern unsigned long this_process_time;

//read mss of imu
extern float accX, accY, accZ;
//read rads of imu
extern float radX, radY, radZ;
//read mag of imu
extern float magX, magY, magZ;

extern float calibration_matrix[3][3];
extern float mag_bias[3];

extern float GyrobiasX;
extern float GyrobiasY;
extern float GyrobiasZ;
extern float AccbiasX;
extern float AccbiasY;
extern float AccbiasZ;
extern float AccfactorX;
extern float AccfactorY;
extern float AccfactorZ;


struct IMU_data {
  float q1;
  float q2;
  float q3;
  float q4;
};
// instanciate one struct
extern IMU_data IMU_data_holder;

#endif
