#include <Arduino.h>
#include "Dasta.hpp"

Dasta dasta;

void setup()
{
  Serial.begin(115200);
  dasta.sensors.init();
  dasta.communication.device_name = "ESP32-Bluetooth";
  dasta.communication.start();

}

String vec2str(Vector &v){
  String s = "";
  for(int i = 0; i < v.size; i++){
    s += String(v(i)) + " ";
  }
  return s;
}

void loop()
{
  if (!dasta.communication.SerialBT.connected(100))
  {
    Serial.println("Bluetooth disconnected");
    ESP.restart();
  }
  
  dasta.sensors.readSensors();
  dasta.sensors.compensateIMU();

  dasta.estimator.run(dasta.sensors.getTime()/1000.0);

  dasta.communication.send();
  dasta.communication.receive();
}