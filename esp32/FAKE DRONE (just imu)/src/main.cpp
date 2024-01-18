#include "Dasta.hpp"

Dasta dasta;

#define STATE_ESTIMATE_DT_ms 10
#define PRINT_DT_ms 500

unsigned long now = 0;

TaskHandle_t stateEstimateTaskHandle, communicationTaskHandle, printTaskHandle;

void stateEstimateTask(void *pvParameters)
{
  unsigned long last_time = now;
  while (1)
  {
    if (now - last_time > STATE_ESTIMATE_DT_ms)
    {
      last_time = now;
      dasta.sensors.readSensors();
      // dasta.estimator.run(dasta.sensors.getTime() / 1000.0);
    }
    // freeRTOS
    delay(1);
  }
}

void communicationTask(void *pvParameters)
{
  unsigned long last_time = now;
  while (1)
  {
    if (now - last_time > dasta.communication.send_stream.delay)
    {
      last_time = now;
      dasta.communication.send();
      dasta.communication.receive();
      dasta.runDecisionOnUserEvent();
    }
    if (!dasta.communication.SerialBT.connected(100))
    {
      Serial.println("Bluetooth disconnected");
      ESP.restart();
    }
    delay(1);
  }
}

String vec2str(Vector &v)
{
  String s = "";
  for (int i = 0; i < v.size; i++)
  {
    s += String(v.data[i]) + " ";
  }
  return s;
}

Vector rpy(3);
void printTask(void *pvParameters)
{
  unsigned long last_time = now;
  while (1)
  {
    if (now - last_time >= PRINT_DT_ms)
    {
      // q2rpy(rpy, dasta.estimator.orientation);
      // mul(rpy, rpy, 180 / M_PI);
      last_time = now;
      Serial.print("t: ");
      Serial.print(dasta.sensors.imu.getTime());
      // Serial.print("\tekf_dt: ");
      // Serial.print(String(StateEstimate::dt_proprio, 4));
      // Serial.print("\tp: ");
      // Serial.print(vec2str(dasta.estimator.position));
      // Serial.print("\tv: ");
      // Serial.print(vec2str(dasta.estimator.velocity));
      // Serial.print("\trpy: ");
      // Serial.print(vec2str(rpy));
      Serial.print("\tacc: ");
      Serial.print(vec2str(dasta.sensors.acc));
      Serial.print("\tgyro: ");
      Serial.print(vec2str(dasta.sensors.gyro));
      Serial.print("\tuser_event: ");
      Serial.print(dasta.decisionnal_unit.user_event);
      Serial.println();
    }
    delay(1);
  }
}

void setup()
{
  Serial.begin(115200); // for more speed, use 921600
  dasta.communication.device_name = "ESP32-Bluetooth";
  dasta.communication.start();
  dasta.sensors.init();

  // start tasks
  xTaskCreatePinnedToCore(
      stateEstimateTask,        /* Function to implement the task */
      "stateEstimateTask",      /* Name of the task */
      10000,                    /* Stack size in words */
      NULL,                     /* Task input parameter */
      0,                        /* Priority of the task , the lower the more priority*/
      &stateEstimateTaskHandle, /* Task handle. */
      0);                       /* Core where the task should run */

  xTaskCreatePinnedToCore(
      communicationTask,        /* Function to implement the task */
      "communicationTask",      /* Name of the task */
      10000,                    /* Stack size in words */
      NULL,                     /* Task input parameter */
      0,                        /* Priority of the task */
      &communicationTaskHandle, /* Task handle. */
      1);                       /* Core where the task should run */

  xTaskCreatePinnedToCore(
      printTask,        /* Function to implement the task */
      "printTask",      /* Name of the task */
      10000,            /* Stack size in words */
      NULL,             /* Task input parameter */
      1,                /* Priority of the task */
      &printTaskHandle, /* Task handle. */
      1);               /* Core where the task should run */
}


void loop()
{
  now = millis();
  delay(1);
}