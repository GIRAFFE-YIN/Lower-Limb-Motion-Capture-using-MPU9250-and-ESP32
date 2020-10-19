#include "wirelessDataTransfer.h"
#include "config.h"
#include "data.h"
//#include "AsyncUDP.h"

// WiFi network name and password
const char* ssid = "8305";
const char* password = "19980418";

const char* send_address = "192.168.0.106";  //macbook address

//The udp library class
WiFiUDP udp;

//AsyncUDP asudp;

bool setupWifi()
{
  //Connect to the WiFi network
  Serial.println("[ESP32] Connecting to WiFi network: " + String(ssid));
  WiFi.disconnect(true, true);
  delay(500);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  while ( WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  printWifiInfo();

  setupUDP();

  return true;
}

void printWifiInfo()
{
  Serial.println("Connected! IP address: ");
  Serial.println(WiFi.localIP());
  Serial.printf("UDP server on port %d\n", localPort);
}

void setupUDP()
{
  udp.begin(localPort);
}

void sendUTF(float q1, float q2, float q3, float q4)
{
  //Serial.print(millis());
  if (udp.beginPacket(send_address, localPort))
  {
    //Send a packet
    udp.print(q1);udp.print(",");
    udp.print(q2);udp.print(",");
    udp.print(q3);udp.print(",");
    udp.print(q4);
    //udp.write((unsigned char *)&IMU_data_holder, sizeof(IMU_data_holder));
    udp.flush();
    udp.endPacket();
  }
}

bool sendUDP(unsigned char values[])
{
  if (udp.beginPacket(send_address, localPort))
  {
    //Send a packet
    udp.write(values, 16);
    udp.flush();
    udp.endPacket();
    return true;
  }
  else
  {
#ifdef DEBUGUDP
    Serial.println("UDP beginpacket FAILED");
#endif
  }
  //Serial.println("sendUDP: end");
  return 0;
}

void checkWifi()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WIFI HAS BEEN DISCONNECTED... RESETTING NOW");
    //if (setupWifi())
    //    printWifiInfo();

  }
}
