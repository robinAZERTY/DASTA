#include "Dasta.hpp"
Dasta dasta;
#define KALMAN_DT_s 0.01
#define CLOSED_LOOP_DT_s 0.01
#define PRINT_DT_s 0.2

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
    // cd(dasta.angular_velocity_command,dasta.communication.angular_velocity_command);
    // dasta.run_angular_velocity_control(now);
    dasta.run_attitude_control(now);
  }
}


void receive()
{   
   if(dasta.communication.receive()==1)
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
    bool new_lipo_track = dasta.sensors.LiPo.run(now);
    if (new_lipo_track)
      dasta.communication.send_stream.enable("battery_voltages");
    if (dasta.decisionnal_unit.internal_event)
          dasta.communication.send_stream.enable("internal_event");

    dasta.communication.send();

    if (new_lipo_track)
      dasta.communication.send_stream.disable("battery_voltages");
    if (dasta.decisionnal_unit.internal_event)
      {
        dasta.decisionnal_unit.internal_event = 0;
        dasta.communication.send_stream.enable("internal_event");
      }

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
    // Serial.print("battery :" + vec2str(dasta.sensors.LiPo.voltages) + " | " + vec2str(dasta.sensors.LiPo.charges));
    // Serial.print("thrust = " + String(dasta.thrust) + "\t");
    // Serial.print("gyro_raw = " + vec2str(dasta.sensors.gyro_raw));
    // Serial.print("\tacc_raw = " + vec2str(dasta.sensors.acc_raw));
    // Serial.print("\th_val[0] =" + vec2str(dasta.estimator.ekf.h_val[0]));
    // Serial.print("\testimator.running = " + String(dasta.estimator.running));
    // Serial.print("\tattitude_control_running = " + String(dasta.attitude_control_running));
    // Serial.print("\tangular_velocity_control_running = " + String(dasta.angular_velocity_control_running));
    Serial.print("acc = " + vec2str(dasta.sensors.acc,2));
    Serial.print("\tq = " + vec2str(dasta.estimator.orientation,2));
    Serial.print("\trpy = " + vec2str(dasta.estimator.rpy,2));
    Serial.print("\tgyro = " + vec2str(dasta.sensors.gyro));
    Serial.println("\trot_vel_command = " + vec2str(dasta.angular_velocity_command,2));
  }
}


void setup()
{
  Serial.begin(115200);
  dasta.init();
  // pinMode(5, OUTPUT);
  // pinMode(17, OUTPUT);
  // digitalWrite(17, HIGH);
  // digitalWrite(5, HIGH);

}

void loop()
{
  now = millis() / 1000.0;
  kalman(now);
  closed_loop(now);
  receive();
  send(now);
  print(now);
  delayMicroseconds(100);
}