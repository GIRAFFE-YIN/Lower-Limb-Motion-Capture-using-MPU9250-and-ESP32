#include "data.h"

unsigned char sendData[PACKET_SIZE] = { 0 };

// instanciate one struct
IMU_data IMU_data_holder;

//read mss of imu
float accX = 0, accY = 0, accZ = 0;
//read rads of imu
float radX = 0, radY = 0, radZ = 0;
//read mag of imu
float magX = 0, magY = 0, magZ = 0;

unsigned long last_process_time = 0;
unsigned long this_process_time = 0;

#if IMUS == 1
const int localPort = 2101;
float calibration_matrix[3][3] = {
  {0.022958397, -0.000024313, -0.00027834},
  {0, 0.02269, 0.000607},
  {0, 0, 0.02374},
};
float mag_bias[3] = { -5.3428, 22.7463, -17.8854 };

float GyrobiasX = 0.026093;
float GyrobiasY = -0.025628;
float GyrobiasZ = -0.022342;
float AccbiasX = 0;
float AccbiasY = 0;
float AccbiasZ = 0;
float AccfactorX = 0;
float AccfactorY = 0;
float AccfactorZ = 0;

#elif IMUS == 2

const int localPort = 2102;

float calibration_matrix[3][3] = {
  {0.020856729, -0.000015252, -0.0001768},
  {0, 0.021254, 0.000324},
  {0, 0, 0.02198},
};

float mag_bias[3] = { 13.885, 11.132, -14.901 };
float GyrobiasX = 0.024458;
float GyrobiasY = -0.027041;
float GyrobiasZ = -0.007267;

#elif IMUS == 3

const int localPort = 2103;
float calibration_matrix[3][3] = {
  {0.02112558, -0.00043216, -0.00054535},
  {0, 0.021369, -0.000316},
  {0, 0, 0.02137},
};

float mag_bias[3] = { 4.1983, 18.3009, 25.3853 };

float GyrobiasX = 0.023641;
float GyrobiasY = -0.023063;
float GyrobiasZ = -0.013666;

#endif
