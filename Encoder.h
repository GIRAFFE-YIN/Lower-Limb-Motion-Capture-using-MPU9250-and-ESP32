#ifndef _ENCODER_H
#define _ENCODER_H

union thing {
  float d;
  unsigned char b[8];
};

void send_IMU_struct();

void set_IMU_data(float q1, float q2, float q3, float q4);

void setFloatToArray(unsigned char array[], unsigned char index, union thing v);

void SetUDPPacket( float q1, float q2, float q3, float q4);

#endif
