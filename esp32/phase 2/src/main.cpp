#include "Dasta.hpp"

Dasta dasta;
#define KALMAN_DT_us 2000
#define CLOSED_LOOP_DT_us 10000
// #define PRINT_DT_ms 100



int64_t last_time_kalman = 0;
void kalman(const int64_t &now)
{
  if (now - last_time_kalman < KALMAN_DT_us)
    return;
  if (dasta.sensors.readSensors(now))
  {
    dasta.sensors.compensateIMU();
    if (last_time_kalman != 0)
    {
      vector::cd(dasta.estimator.ekf.u, dasta.sensors.gyro);
      vector::cd(dasta.estimator.ekf.z[0], dasta.sensors.acc);
      dasta.kalman_delay.data[0] = (now - last_time_kalman)/1000000.0;
      dasta.kalman_delay.data[1] = max(dasta.kalman_delay.data[1], dasta.kalman_delay.data[0]);
      dasta.estimator.run(dasta.kalman_delay.data[0]);
    }
    last_time_kalman = now;
  }
}


int64_t last_time_closed_loop = 0;
void closed_loop(const int64_t &now)
{
  if (now - last_time_closed_loop < CLOSED_LOOP_DT_us)
    return;
  if (last_time_closed_loop != 0)
  {
    dasta.close_loop_delay.data[0] = (now - last_time_closed_loop)/1000000.0;
    dasta.control(dasta.close_loop_delay.data[0]);
    dasta.close_loop_delay.data[1] = max(dasta.close_loop_delay.data[1], dasta.close_loop_delay.data[0]);
  }
  last_time_closed_loop = now;
}


void receive()
{
  if (dasta.communication.receive() == 1)
    dasta.runDecisionOnUserEvent();

  if (!dasta.communication.SerialBT.connected(100)) // safety feature
  {
    Serial.println("Bluetooth disconnected");
    dasta.actuators.stopMotors();
    ESP.restart();
  }
}

int64_t last_time_send = 0;
void send(const int64_t &now)
{

  if (now - last_time_send < dasta.communication.send_stream.delay * 1000)
    return;

  if (last_time_send != 0)
  {
    dasta.stream_delay.data[0] = (now - last_time_send)/1000000.0;
    dasta.stream_delay.data[1] = max(dasta.stream_delay.data[0], dasta.stream_delay.data[1]);
  }
  last_time_send = now;
  bool new_lipo_track = dasta.sensors.LiPo.run(now/1000000.0);
  if (new_lipo_track)
  {
    dasta.communication.send_stream.enable("battery_voltages");
    dasta.communication.send_stream.enable("battery_lvl");
  }
  if (dasta.decisionnal_unit.internal_event)
    dasta.communication.send_stream.enable("internal_event");

  dasta.communication.send();

  if (new_lipo_track)
  {
    dasta.communication.send_stream.disable("battery_voltages");
    dasta.communication.send_stream.disable("battery_lvl");
  }
  if (dasta.decisionnal_unit.internal_event)
  {
    dasta.decisionnal_unit.internal_event = 0;
    dasta.communication.send_stream.enable("internal_event");
  }
  dasta.kalman_delay.data[1] = dasta.kalman_delay.data[0];
  dasta.close_loop_delay.data[1] = dasta.close_loop_delay.data[0];
  dasta.stream_delay.data[1] = dasta.stream_delay.data[0];
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

#ifdef PRINT_DT_ms
int64_t last_time_print = 0, print_dt = 0, max_print_dt = 0;
void print(const int64_t &now)
{
  if (!Serial)
    return;
  if (now - last_time_print < PRINT_DT_ms * 1000)
    return;

  if (last_time_print != 0)
  {
    print_dt = now - last_time_print;
    max_print_dt = max(max_print_dt, print_dt);
  }
  last_time_print = now;
  // Serial.print("kalman_dt:" + String(dasta.kalman_delay.data[0]) + ",max_kalman_dt:" + String(dasta.kalman_delay.data[1]));
  // Serial.print(",closed_loop_dt:" + String(dasta.close_loop_delay.data[0]) + ",max_closed_loop_dt:" + String(dasta.close_loop_delay.data[1]));
  // Serial.print(",send_dt:" + String(send_dt) + ",max_send_dt:" + String(max_send_dt));
  // // Serial.print(",print_dt:" + String(print_dt) + ",max_print_dt:" + String(max_print_dt));
  // Serial.print(",x:" + String(dasta.estimator.lpf_gyr_derx.getFilteredValue()) + ",y:" + String(dasta.estimator.lpf_gyr_dery.getFilteredValue()));
  Serial.print(",integralePidRx:" + String(dasta.pidRx.integral) + ",integralePidRy:" + String(dasta.pidRy.integral) + ",integralePidRz:" + String(dasta.pidRz.integral));
  Serial.println();
}
#endif

// #define IR_LED_1_PIN 17
// #define IR_LED_2_PIN 5
// #define IR_LED_3_PIN 18
// #define IR_LED_4_PIN 16

void setup()
{
  Serial.begin(115200);
  dasta.init();
  // dasta.actuators.led1.on();
  // dasta.actuators.led2.on();
  // dasta.actuators.led3.on();
  // dasta.actuators.led4.on();
}

void loop()
{
  dasta.now = esp_timer_get_time();
  kalman(dasta.now);
  closed_loop(dasta.now);
  receive();
  send(dasta.now);
  #ifdef PRINT_DT_ms
  print(dasta.now);
  #endif
}