#include "Encoder.h"
#include "config.h"
#include "data.h"

union thing things[4];

// function to update the struct with the new data
void set_IMU_data(float q1, float q2, float q3, float q4) {
  IMU_data_holder.q1 = q1;
  IMU_data_holder.q2 = q2;
  IMU_data_holder.q3 = q3;
  IMU_data_holder.q4 = q4;
}

void send_IMU_struct() {
  Serial.write('S');
  Serial.write((uint8_t *)&IMU_data_holder, sizeof(IMU_data_holder));
  Serial.write('E');
}

void SetUDPPacket( float q1, float q2, float q3, float q4) {
  things[0].d = q1;
  things[1].d = q2;
  things[2].d = q3;
  things[3].d = q4;
  for (uint8_t i = 0; i < 4; i += 1)
    setFloatToArray(sendData, i * 4, things[i]);
}

void setFloatToArray(uint8_t array[], uint8_t index, union thing v)
{
  for (uint8_t i = 0; i < 4; i += 1)
    array[index + i] = v.b[i];
}
