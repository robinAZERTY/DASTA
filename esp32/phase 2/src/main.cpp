#include "Dasta.hpp"
Dasta dasta;
#define KALMAN_DT_s 0.01
#define CLOSED_LOOP_DT_s 0.05
#define PRINT_DT_s 0.5

float now = 0;

float last_time_kalman = 0;
void kalman(const float now)
{
  if (now - last_time_kalman > KALMAN_DT_s)
  {
    last_time_kalman = now;
    dasta.sensors.readSensors();
    dasta.sensors.compensateIMU();
    vector::cd(dasta.estimator.ekf.u, dasta.sensors.gyro);
    vector::cd(dasta.estimator.ekf.z[0], dasta.sensors.acc);
    dasta.estimator.run(now);
  }
}

float last_time_closed_loop = 0;
void closed_loop(const float now)
{
  if (now - last_time_closed_loop > CLOSED_LOOP_DT_s)
  {
    last_time_closed_loop = now;
    dasta.run_attitude_control(now);
  }
}


void receive()
{
    if(dasta.communication.receive())
      dasta.runDecisionOnUserEvent();
  
  if (!dasta.communication.SerialBT.connected(100)) // safety feature
  {
    Serial.println("Bluetooth disconnected");
    dasta.actuators.stopMotors();
    ESP.restart();
  }
}

float last_time_send = 0;
void send(const float now)
{
  if (now - last_time_send > dasta.communication.send_stream.delay/1000.0)
  {
    last_time_send = now;
    dasta.communication.send();
  }
}

String vec2str(Vector &v, const int precision = 3)
{
  String s = "[";
  for (int i = 0; i < v.size; i++)
  {
    s += String(v.data[i], precision) + " ";
  }
  s += "]";
  return s;
}

float last_time_print = 0;
void print(const float now)
{
  if (now - last_time_print > PRINT_DT_s)
  {
    last_time_print = now;
    Serial.print("gyro = " + vec2str(dasta.sensors.gyro));
    Serial.print("\tacc = " + vec2str(dasta.sensors.acc));
    Serial.print("\th_val[0] =" + vec2str(dasta.estimator.ekf.h_val[0]));
    Serial.println("\testimator.running = " + String(dasta.estimator.running));
  }
}


void setup()
{
  Serial.begin(115200);
  dasta.init();
}

void loop()
{
  now = millis() / 1000.0;
  kalman(now);
  closed_loop(now);
  receive();
  send(now);
  print(now);
}