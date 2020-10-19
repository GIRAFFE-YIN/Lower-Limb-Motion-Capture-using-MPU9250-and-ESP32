#ifndef _WIRELESSDATATRANSFER_H
#define _WIRELESSDATATRANSFER_H

//extern WiFiUDP Udp;

bool setupWifi();
void printWifiInfo();
void setupUDP();
bool sendUDP(unsigned char values[]);
void checkWifi();
void sendUTF(float q1, float q2, float q3, float q4);

#endif
