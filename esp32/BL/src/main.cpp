#include <Arduino.h>

#include "ekf.hpp"
#include "Communication.hpp"

#define M_PI 3.14159265358979323846
const float dt_proprio = 0.01; // période d'échantillonnage du proprio
const float dt_gps = 1;        // période d'échantillonnage du gps

// X = [x,y,theta]
// Z = [x,y]
// U = [v,w]
// fonction de transition

#define Vout (*Ekf::Vin)
#define Min (*Ekf::Min)

void f(Vector &x, Vector &u)
{
  Vout(0) = x(0) + u(0) * cos(x(2)) * dt_proprio;
  Vout(1) = x(1) + u(0) * sin(x(2)) * dt_proprio;
  Vout(2) = x(2) + u(1) * dt_proprio;
}

// fonction de mesure
void h(Vector &x)
{
  Vout(0) = x(0);
  Vout(1) = x(1);
}

// jacobienne de f par rapport à x
void Fx(Vector &x, Vector &u)
{
  Min(0, 0) = 1;                              // dx/dx
  Min(0, 1) = 0;                              // dx/dy
  Min(0, 2) = -u(0) * sin(x(2)) * dt_proprio; // dx/dtheta
  Min(1, 0) = 0;                              // dy/dx
  Min(1, 1) = 1;                              // dy/dy
  Min(1, 2) = u(0) * cos(x(2)) * dt_proprio;  // dy/dtheta
  Min(2, 0) = 0;                              // dtheta/dx
  Min(2, 1) = 0;                              // dtheta/dy
  Min(2, 2) = 1;                              // dtheta/dtheta
}

// jacobienne de f par rapport à u
void Fu(Vector &x, Vector &u)
{
  Min(0, 0) = cos(x(2)) * dt_proprio; // dx/dv
  Min(0, 1) = 0;                      // dx/dw
  Min(1, 0) = sin(x(2)) * dt_proprio; // dy/dv
  Min(1, 1) = 0;                      // dy/dw
  Min(2, 0) = 0;                      // dtheta/dv
  Min(2, 1) = dt_proprio;             // dtheta/dw
}

// jacobienne de h par rapport à x
void H(Vector &x)
{
  Min(0, 0) = 1; // dx/dx
  Min(0, 1) = 0; // dx/dy
  Min(0, 2) = 0; // dx/dtheta
  Min(1, 0) = 0; // dy/dx
  Min(1, 1) = 1; // dy/dy
  Min(1, 2) = 0; // dy/dtheta
}

// pour la simulation des mesures
const float randn(const float sigma)
{
  // Box-Muller transform
  // return a random number from a normal distribution of mean 0 and standard deviation sigma
  const float u1 = rand() / (float)RAND_MAX;
  const float u2 = rand() / (float)RAND_MAX;
  return sigma * sqrt(-2 * log(u1)) * cos(2 * M_PI * u2);
}

Communication comm;

void setup()
{

  Serial.begin(115200);
  comm.SerialBT.begin("ESP32test"); // Bluetooth device name
  // print the mac address of the ESP32 to which you need to connect
  Serial.print("The MAC address of ESP32 is: ");
  Serial.println(comm.SerialBT.getBtAddressString());

  while (!Serial || !comm.SerialBT.connected(0))
  {
    delay(100);
    ;
  }
  // set the seed of the random number generator
  srand(analogRead(0));

  Ekf ekf(f, h, 3, 2, 2, Fx, Fu, H);

  // comm.set_X_ptr(ekf.x);
  // comm.set_P_ptr(ekf.P);
  // comm.set_Z_ptr(ekf.z);
  // comm.send_t = true;
  // comm.send_X = true;
  // comm.send_Z = true;

  // cout << "ekf constructed !\n";
  // cout << "used memory : " << Matrix::memory_usage() << " bytes\n";
  Serial.println("ekf constructed !");
  Serial.print("used memory : ");
  Serial.print(Matrix::memory_usage() + sizeof(ekf));
  Serial.println(" bytes");

  // incertitudes
  const float gyro_noise = 0.05; // 0.05 rad/s
  const float speed_noise = 0.1; // 0.1 m/s
  float gps_noise = 5.0;   // 5 m

  // initialisation
  (*ekf.x)(0) = 0;
  (*ekf.x)(1) = 0;
  (*ekf.x)(2) = 0;
  (*ekf.P)(0, 0) = 1e5;
  (*ekf.P)(1, 1) = 1e5;
  (*ekf.P)(2, 2) = 0.03;
  (*ekf.Q)(0, 0) = gyro_noise * gyro_noise;
  (*ekf.Q)(1, 1) = speed_noise * speed_noise;
  (*ekf.R)(0, 0) = gps_noise * gps_noise;
  (*ekf.R)(1, 1) = gps_noise * gps_noise;

  // simulation
  const float T = 100;
  float sleepTime = 1;
  Vector true_X(3);
  true_X.set_zero();
  Vector gps(2);
  gps.set_zero();
  float last_gps_t = -1e5;

  // cout<<"t x y theta ekfX ekfY ekfTheta\n";
  // Serial.println("t x y theta ekfX ekfY ekfTheta P00 P01 P02 P10 P11 P12 P20 P21 P22");
  // comm.SerialBT.print(comm.describe());

  unsigned long long t_ms = 0;
  unsigned long long t_ms_old = 0;

  // construction du stream d'envoi
  comm.send_stream.include("t", (uint8_t *)&t_ms, UNSIGNED_LONG_LONG_KEY, sizeof(t_ms));
  comm.send_stream.include("true_X", true_X);
  comm.send_stream.include("ekf_X", *ekf.x);
  comm.send_stream.include("ekf_P", *ekf.P);
  comm.send_stream.include("gps", gps);
  
  // construction du stream de réception
  comm.receive_stream.include("sleepTime", (uint8_t *)&sleepTime, FLOAT_KEY, sizeof(sleepTime));
  comm.receive_stream.include("gps_noise", (uint8_t *)&gps_noise, FLOAT_KEY, sizeof(gps_noise));
  
  comm.send_header(&comm.send_stream);
  comm.send_header(&comm.receive_stream);

  for (float t = 0; t < T; t += dt_proprio)
  {
    const float true_v = sin(t / T * M_PI * 3) + 1;
    const float true_w = cos(t / T * M_PI * 4);

    const float v_sample = true_v + randn(speed_noise);
    const float w_sample = true_w + randn(gyro_noise);

    true_X(0) += true_v * cos(true_X(2)) * dt_proprio;
    true_X(1) += true_v * sin(true_X(2)) * dt_proprio;
    true_X(2) += true_w * dt_proprio;

    (*ekf.u)(0) = v_sample;
    (*ekf.u)(1) = w_sample;

    (*ekf.u)(0) = v_sample;
    (*ekf.u)(1) = w_sample;

    ekf.predict();
    comm.send_stream.disable("gps");
    if (t - last_gps_t > dt_gps)
    {
      comm.send_stream.enable("gps");
      last_gps_t = t;
      gps(0) = true_X(0) + randn(gps_noise);
      gps(1) = true_X(1) + randn(gps_noise);
      (*ekf.z)(0) = gps(0);
      (*ekf.z)(1) = gps(1);
      ekf.update();
    }
    t_ms = t * 1000;
    comm.send();
    unsigned long long tt = millis();
    Serial.print("gps_noise : "+String(gps_noise)+"\n");
    do
    {
      comm.receive();
      tt = millis();
    } while (tt - t_ms_old < sleepTime*1000);
    t_ms_old = tt;

    if (!Serial || !comm.SerialBT.connected(0))
    {
      Serial.println("Restarting...");
      ESP.restart();
    }
  }
}

void loop()
{
  // reset if disconnected
  if (!Serial || !comm.SerialBT.connected(0))
  {
    Serial.println("Restarting...");
    ESP.restart();
  }
}