/******************************************************
  Arduino library for Kalman Filter
  2019.11.22

  Author: Yin maowen
  Version: 1.0.0
 *******************************************************/

#ifndef KALMAN_H
#define KALMAN_H

#include <Arduino.h>
#include <math.h>


class KALMAN
{
  public:
    KALMAN();  //constructor
    ~KALMAN();

    //float Angle; //Final angle get by kalman filter

    //float Angle_gy;  //angle caculated by w

    //void init_filter();

    float getAngle(float newAngle, float newRate, float dt);
    //void six_axis_position(float gx, float gy, float gz, float ax, float ay, float az);
    void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
                                  float gz, float mx, float my, float mz,
                                  float deltat);

    void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
                                float gz, float mx, float my, float mz,
                                float deltat);

    void MadgwickWithoutMag(float gx, float gy, float gz, float ax, float ay, float az, float dt);

    void AnglaUpdate();
    float Q_ANGLE_Y;
    float Q_ANGLE_X;

    float ab_roll;
    float ab_pitch;
    float ab_yaw;

    float q[4];



  private:

    float Q_angle; // Process noise variance for the accelerometer
    float Q_bias; // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float eInt[3];

    float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix

    float invSqrt(float x);

    //float ab_pitch, ab_yaw, ab_roll;

    //float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // 初始姿态四元数，由上篇博文提到的变换四元数公式得来
    //float exInt = 0, eyInt = 0, ezInt = 0;    //当前加计测得的重力加速度在三轴上的分量
    //与用当前姿态计算得来的重力在三轴上的分量的误差的积分

};

#endif
