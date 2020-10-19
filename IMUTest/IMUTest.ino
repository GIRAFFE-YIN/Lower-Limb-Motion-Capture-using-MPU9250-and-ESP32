#include "data.h"
#include "IMU.h"
#include "wirelessDataTransfer.h"
#include "KALMAN.h"
#include "Encoder.h"
#include "timer.h"


KALMAN Kalman;

float Angle_roll = 0.0;
float Angle_pitch = 0.0;
float Angle_yaw = 0.0;

void setup() {

  Serial.begin(115200);           // Define baud rate
  IMU_init();                     //init the imu sensor
#if Debug == 0
  setupWifi();                    //init the wifi block
  init_timer();                   //tirgger the timer every 100ms to send the udp data
#endif
}

void loop() {

  IMU_read();

  calculation();
  
#if Debug == 1
  prints();
  
  
#elif Debug == 0
  if (timer_trigger()) {
    //SetUDPPacket(Kalman.q[0], Kalman.q[1], Kalman.q[2], Kalman.q[3]);
    set_IMU_data(Kalman.q[0], Kalman.q[1], Kalman.q[2], Kalman.q[3]);
    sendUTF(Kalman.q[0], Kalman.q[1], Kalman.q[2], Kalman.q[3]);
    //sendUDP(sendData);
  }
  
#endif

}

void calculation() {

  init_calculation();

  unsigned long time_g = this_process_time - last_process_time;
  double dt = time_g / 1000000.0;

  Kalman.MadgwickQuaternionUpdate(accX, accY, accZ, radX, radY, radZ, magX, magY, magZ, dt);

  Angle_pitch = Kalman.ab_pitch * RAD_TO_DEG;
  Angle_yaw   = Kalman.ab_yaw * RAD_TO_DEG;
  // Declination of leeds (53°50'00.00"N，1°35'00.00"W) is 0° 44' W  ± 0° 24' on 2020-1-1
  // via http://www.ngdc.noaa.gov/geomag-web/#declination
  Angle_yaw  += 0.44;
  Angle_roll = Kalman.ab_roll * RAD_TO_DEG;
  
}

void prints() {
  Serial.print(magX, 8); Serial.print(',');
  Serial.print(magY, 8); Serial.print(',');
  Serial.println(magZ, 8);
  Serial.flush();
  delay(100);
}

void printacc() {
  Serial.print(accX); Serial.print(",");
  Serial.print(accY); Serial.print(",");
  Serial.println(accZ);
  Serial.flush();
  delay(100);
}

void printq() {
  Serial.print("f"); Serial.print('/');
  Serial.print(Kalman.q[0], 6); Serial.print('/');
  Serial.print(Kalman.q[1], 6); Serial.print('/');
  Serial.print(Kalman.q[2], 6); Serial.print('/');
  Serial.println(Kalman.q[3], 6);
  Serial.flush();
  delay(1);
}
