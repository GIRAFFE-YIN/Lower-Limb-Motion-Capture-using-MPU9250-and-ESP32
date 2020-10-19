/******************************************************
  Arduino library for Kalman Filter
  2019.11.22

  Author: Yin maowen
  Version: 1.2.0
 *******************************************************/

#include "KALMAN.h"

static float beta = 0.1;
static float beta_without = 0.05;


KALMAN::KALMAN()
{
  Q_angle = 0.001f;
  Q_bias = 0.003f;
  R_measure = 0.03f;

  angle = 0.0f; // Reset the angle
  bias = 0.0f; // Reset bias

  P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
  P[0][1] = 0.0f;
  P[1][0] = 0.0f;
  P[1][1] = 0.0f;

  q[0] = 1;
  q[1] = 0;
  q[2] = 0;
  q[3] = 0;

}

KALMAN::~KALMAN()
{

}

float KALMAN::getAngle(float newAngle, float newRate, float dt) {
  // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
  // Modified by Kristian Lauszus
  // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

  // Discrete Kalman filter time update equations - Time Update ("Predict")
  // Update xhat - Project the state ahead
  /* Step 1 */
  rate = newRate - bias;
  angle += dt * rate;

  // Update estimation error covariance - Project the error covariance ahead
  /* Step 2 */
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
  // Calculate Kalman gain - Compute the Kalman gain
  /* Step 4 */
  float S = P[0][0] + R_measure; // Estimate error
  /* Step 5 */
  float K[2]; // Kalman gain - This is a 2x1 vector
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  // Calculate angle and bias - Update estimate with measurement zk (newAngle)
  /* Step 3 */
  float y = newAngle - angle; // Angle difference
  /* Step 6 */
  angle += K[0] * y;
  bias += K[1] * y;

  // Calculate estimation error covariance - Update the error covariance
  /* Step 7 */
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return angle;
};





/*float KALMAN::Filter(float newAngle, float newRate, float dt)
  {
    Angle += (newRate - Q_bias) * dt;

    Pdot[0] = Q_angle - PP[0][1] - PP[1][0];

    Pdot[3] = Q_gyro;

    PP[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    PP[0][1] -= dt * PP[1][1];
    PP[1][0] -= dt * PP[1][1];
    PP[1][1] += Pdot[3] * dt;

    Angle_err = newAngle - Angle;

    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];

    E = R_angle + C_0 * PCt_0;
    if(E!=0)
    {
        K_0 = PCt_0 / E;
        K_1 = PCt_1 / E;
    }

    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    PP[0][0] -= K_0 * t_0;
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    Angle        += K_0 * Angle_err;
    Q_bias        += K_1 * Angle_err;

    return Angle;
  }*/
/*
  void KALMAN::six_axis_position(float gx, float gy, float gz, float ax, float ay, float az)//g表陀螺仪，a表加计
  {

  float q0temp, q1temp, q2temp, q3temp; //四元数暂存变量，求解微分方程时要用
  float norm; //矢量的模或四元数的范数
  float vx, vy, vz;//当前姿态计算得来的重力在三轴上的分量
  float ex, ey, ez;//当前加计测得的重力加速度在三轴上的分量
  //与用当前姿态计算得来的重力在三轴上的分量的误差

  // 先把这些用得到的值算好
  float q0q0 = q0 * q0;
  float q0q1 = q0 * q1;
  float q0q2 = q0 * q2;
  float q1q1 = q1 * q1;
  float q1q3 = q1 * q3;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q3q3 = q3 * q3;
  if (ax * ay * az == 0) //加计处于自由落体状态时不进行姿态解算，因为会产生分母无穷大的情况
    return;
  norm = sqrt(ax * ax + ay * ay + az * az); //单位化加速度计，
  ax = ax / norm; // 这样变更了量程也不需要修改KP参数，因为这里归一化了
  ay = ay / norm;
  az = az / norm;
  //用当前姿态计算出重力在三个轴上的分量，
  //参考坐标n系转化到载体坐标b系的用四元数表示的方向余弦矩阵第三列即是（博文一中有提到）
  vx = 2 * (q1q3 - q0q2);
  vy = 2 * (q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;
  //计算测得的重力与计算得重力间的误差，向量外积可以表示这一误差
  //原因我理解是因为两个向量是单位向量且sin0等于0
  //不过要是夹角是180度呢~这个还没理解
  ex = (ay * vz - az * vy) ;
  ey = (az * vx - ax * vz) ;
  ez = (ax * vy - ay * vx) ;

  exInt = exInt + ex * Ki;                                           //对误差进行积分
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;
  // adjusted gyroscope measurements
  gx = gx + Kp * ex + exInt; //将误差PI后补偿到陀螺仪，即补偿零点漂移
  gy = gy + Kp * ey + eyInt;
  gz = gz + Kp * ez + ezInt;  //这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减
  //下面进行姿态的更新，也就是四元数微分方程的求解
  q0temp = q0; //暂存当前值用于计算
  q1temp = q1; //网上传的这份算法大多没有注意这个问题，在此更正
  q2temp = q2;
  q3temp = q3;
  //采用一阶毕卡解法，相关知识可参见《惯性器件与惯性导航系统》P212
  q0 = q0temp + (-q1temp * gx - q2temp * gy - q3temp * gz) * halfT;
  q1 = q1temp + (q0temp * gx + q2temp * gz - q3temp * gy) * halfT;
  q2 = q2temp + (q0temp * gy - q1temp * gz + q3temp * gx) * halfT;
  q3 = q3temp + (q0temp * gz + q1temp * gy - q2temp * gx) * halfT;
  //单位化四元数在空间旋转时不会拉伸，仅有旋转角度，这类似线性代数里的正交变换
  norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;
  //四元数到欧拉角的转换，公式推导见博文一
  //其中YAW航向角由于加速度计对其没有修正作用，因此此处直接用陀螺仪积分代替
  //Q_ANGLE.Z = GYRO_I.Z; // yaw
  Q_ANGLE_Y = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3; // pitch
  Q_ANGLE_X = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3; // roll
  }
*/

void KALMAN::MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
                                      float gz, float mx, float my, float mz,
                                      float deltat) {

  // short name local variable for readability
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = invSqrt(ax * ax + ay * ay + az * az);
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = invSqrt(mx * mx + my * my + mz * mz);
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 +
       _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = invSqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

  // Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  norm = invSqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
  AnglaUpdate();
}



// These are the free parameters in the Mahony filter and fusion scheme, Kp
// for proportional feedback, Ki for integral
#define Kp 3.0f
#define Ki 0.01f

static float eInt[3] = {0.0f, 0.0f, 0.0f};

// Similar to Madgwick scheme but uses proportional and integral filtering on
// the error between estimated reference vectors and measured ones.
void KALMAN::MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
  // short name local variable for readability
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = invSqrt(ax * ax + ay * ay + az * az);
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = invSqrt(mx * mx + my * my + mz * mz);
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrt((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

  // Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f)
  {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
  }
  else
  {
    eInt[0] = 0.0f;     // prevent integral wind up
    eInt[1] = 0.0f;
    eInt[2] = 0.0f;
  }

  // Apply feedback terms
  gx = gx + Kp * ex + Ki * eInt[0];
  gy = gy + Kp * ey + Ki * eInt[1];
  gz = gz + Kp * ez + Ki * eInt[2];

  // Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
  q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
  q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
  q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

  // Normalise quaternion
  norm = invSqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;

  AnglaUpdate();
}


//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float KALMAN::invSqrt(float x) {
  /* close-to-optimal  method with low cost from http://pizer.wordpress.com/2008/10/12/fast-inverse-square-root */
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
}

void KALMAN::MadgwickWithoutMag(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
  float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;   

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta_without * s0;
    qDot2 -= beta_without * s1;
    qDot3 -= beta_without * s2;
    qDot4 -= beta_without * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q[0] = q0 * recipNorm;
  q[1] = q1 * recipNorm;
  q[2] = q2 * recipNorm;
  q[3] = q3 * recipNorm;

  AnglaUpdate();
}

void KALMAN::AnglaUpdate() {
  float w = q[0];
  float x = q[1];
  float y = q[2];
  float z = q[3];
  float p, h, b;
  //计算pitch
  float sp = -2.0f * ( y * z + w * x);
  //检查万象锁
  if (fabs(sp > 0.999f)) {
    p = 1.570796f * sp;
    h = atan2(-x * z - w * y, 0.5f - y * y - z * z);
    b = 0.0f;
  } else {
    p = asin(sp);
    h = atan2(x * z - w * y, 0.5f - x * x - y * y);
    b = atan2(x * y - w * z, 0.5f - x * x - z * z);
  }
  //ab_yaw = atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
  //ab_roll = -sin(2.0 * (q[1] * q[3] - q[0] * q[2]));
  //ab_pitch = atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
  ab_roll = h;
  ab_pitch = p;
  ab_yaw = b;
}
