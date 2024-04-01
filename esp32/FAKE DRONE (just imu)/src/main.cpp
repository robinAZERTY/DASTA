#include "Dasta.hpp"

Dasta dasta;

#define STATE_ESTIMATE_DT_ms 10
#define PRINT_DT_ms 500

unsigned long now = 0;

TaskHandle_t stateEstimateTaskHandle;
TaskHandle_t communicationTaskHandle, printTaskHandle;
// Création d'un sémaphore pour la synchronisation
SemaphoreHandle_t xSemaphore;

void stateEstimateTask(void *pvParameters)
{
  dasta.sensors.startGyroBiasEstimation();
  unsigned long last_time = now;
  while (1)
  {
    if (now - last_time > STATE_ESTIMATE_DT_ms)
    {
      last_time = now;
      // entrer dans la section critique (en attente du sémaphore)
      if (xSemaphoreTake(xSemaphore, STATE_ESTIMATE_DT_ms))
        dasta.sensors.readSensors();
      // dasta.sensors.compensateGyroBias();
      dasta.run_anguler_velocity_control();
      // dasta.estimator.run(dasta.sensors.getTime() / 1000.0);
      // quitter la section critique (rendre le sémaphore)
      xSemaphoreGive(xSemaphore);
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

    // float angular_velocity_command = dasta.communication.angular_velocity_command.data[0];
    // Serial.println(angular_velocity_command);

    if (dasta.communication.receive())
    {
      dasta.runDecisionOnUserEvent();
    }

    if (now - last_time > dasta.communication.send_stream.delay)
    {
      last_time = now;

      // entrer dans la section critique (en attente du sémaphore)
      if (xSemaphoreTake(xSemaphore, dasta.communication.send_stream.delay))
        dasta.communication.send();
      // quitter la section critique (rendre le sémaphore)
      xSemaphoreGive(xSemaphore);
    }
    if (!dasta.communication.SerialBT.connected(100))
    {
      Serial.println("Bluetooth disconnected");
      dasta.actuators.motor1.write(0);
      dasta.actuators.motor2.write(0);
      dasta.actuators.motor3.write(0);
      dasta.actuators.motor4.write(0);
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
    s += String(v.data[i], 3) + " ";
  }
  return s;
}

Vector rpy(3);
void printTask(void *pvParameters)
{
  unsigned long last_time = now;

  last_time = now;
  while (1)
  {
    if (now - last_time >= PRINT_DT_ms)
    {
      last_time = now;
      // Serial.print(" gyrx: ");
      // Serial.print(dasta.sensors.gyro(0));
      // Serial.print(" x rot command: ");
      // Serial.print(dasta.communication.angular_velocity_command(0));
      // Serial.print(" motor1: ");
      // Serial.print(dasta.actuators.motor1.read());
      // Serial.print(" gyro bias compensation: ");
      // Serial.print(dasta.sensors.gyro_bias_co(0));
      Serial.print(" pidx_param: ");
      Serial.print(dasta.pidRx.kp);
      Serial.print(" ");
      Serial.print(dasta.pidRx.ki);
      Serial.print(" ");
      Serial.print(dasta.pidRx.kd);
      Serial.print(" ");
      Serial.print(dasta.pidRx.maxIntegral);
      Serial.print(" ");
      Serial.print(dasta.pidRx.timeConstDerFilter);
      Serial.print(" ");
      Serial.print(dasta.pidRx.min);
      Serial.print(" ");
      Serial.print(dasta.pidRx.max);
      Serial.print(" ");
      Serial.print(" integral: ");
      Serial.print(dasta.pidRx.integral);
      Serial.print(" ");
      Serial.print(" lastder: ");
      Serial.println(dasta.pidRx.lastDerivative);
      /*
       float lastError;                // Last error value
    float integral;                 // Integral term
    float lastDerivative;           // Last derivative term
    float lastTime;                 // Last time value
    float deltaTime; 
    */


      // Serial.print("\tTasks stack size left: ");
      // Serial.print(uxTaskGetStackHighWaterMark(stateEstimateTaskHandle));
      // Serial.print(" ");
      // Serial.print(uxTaskGetStackHighWaterMark(communicationTaskHandle));
      // Serial.print(" ");
      // Serial.print(uxTaskGetStackHighWaterMark(printTaskHandle));
      // Serial.print("\t total remaining RAM: ");
      // Serial.print(ESP.getFreeHeap());
      // Serial.print(" / ");
      // Serial.print(ESP.getHeapSize());
      // Serial.print("\t send stream status (running, delay): ");
      // Serial.print(dasta.communication.running_send_stream);
      // Serial.print(", ");
      // Serial.print(dasta.communication.send_stream.delay);
      // Serial.print("\t gyro : ");
      // Serial.print(vec2str(dasta.sensors.gyro));
      // Serial.print("\t gyro bias compensation : ");
      // Serial.print(vec2str(dasta.sensors.gyro_bias_co));
      // Serial.print("\t dt_proprio: ");
      // Serial.print(dasta.sensors.dt_proprio);
    }
    delay(1);
  }
}

void setup()
{
  xSemaphore = xSemaphoreCreateBinary();
  if (xSemaphore != NULL)
    xSemaphoreGive(xSemaphore);

  Serial.begin(115200); // for more speed, use 921600
  delay(4000);
  dasta.communication.device_name = "ESP32-Bluetooth";
  dasta.communication.start();
  delay(1000); // wait for the serial monitor to open
  dasta.sensors.init();
  dasta.configActuators();

  // Serial.println("Sensors initialized");

  // dasta.actuators.motor1.write(0.01); // start the motor at a really low speed (should be enough to start the motor)

  // dasta.actuators.motor1.write(0.01); // start the motor at a really low speed (should be enough to start the motor)
  // delay(3000);                        // during 10 seconds
  // dasta.actuators.motor1.write(0);    // stop the motor
  // delay(3000);                        // wait 3 seconds

  // // start tasks
  xTaskCreatePinnedToCore(
      stateEstimateTask,        /* Function to implement the task */
      "stateEstimateTask",      /* Name of the task */
      3000,                     /* Stack size in words */
      NULL,                     /* Task input parameter */
      0,                        /* Priority of the task , the lower the more priority*/
      &stateEstimateTaskHandle, /* Task handle. */
      0);                       /* Core where the task should run */

  xTaskCreatePinnedToCore(
      communicationTask,        /* Function to implement the task */
      "communicationTask",      /* Name of the task */
      1200,                      /* Stack size in words */
      NULL,                     /* Task input parameter */
      1,                        /* Priority of the task */
      &communicationTaskHandle, /* Task handle. */
      1);                       /* Core where the task should run */

  xTaskCreatePinnedToCore(
      printTask,        /* Function to implement the task */
      "printTask",      /* Name of the task */
      1500,             /* Stack size in words */
      NULL,             /* Task input parameter */
      2,                /* Priority of the task */
      &printTaskHandle, /* Task handle. */
      1);               /* Core where the task should run */
}

void loop()
{
  now = millis();
  // dasta.sensors.readSensors();
  // Serial.println(dasta.sensors.gyro(0));
  delay(1);
}