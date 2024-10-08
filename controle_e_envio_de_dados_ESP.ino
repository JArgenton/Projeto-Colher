#include <Kalman.h>
#include "Wire.h"
#include "MPU6050.h"
#include <HTTP_Method.h>
#include <Uri.h>
#include <WebServer.h>
#include <Wire.h>
#include <WiFi.h>


MPU6050 mpu; //cria obj classe MPU6050
Kalman kalmanPitch; //obj kalman para pitch --> BIBLIOTECA EXTERNA --> github.com/TKJEletronics/KalmanFilter --> eu escrevi o link, pode estar errado @sabado
Kalman kalmanRoll; //obj kalman para roll 

//SABADO, A FINS DE TESTES, MUDE PARA O WIFI DA SUA CASA
const char* ssid = "HUAWEI-5G-Zs26";
const char* password = "HcrcvARh";

WebServer server(80);

int16_t acelX, acelY, acelZ;
int16_t giroX, giroY, giroZ; 

//VALORES CONSTANTES PID, I é 0 devido a especificidade do problema
float Kp = 1.0;
float Ki = 0.01;
float Kd = 0;

float setPointPitch = 0.0;
float setPointRoll = 0.0; 

//VARIAVEIS PARA GUARDAR VALORES PITCH E ROLL ANTES DO PID
float pitch, roll;

//INICIALIZAÇAO VARIAVEIS PID

//PID PITCH
float pitchError, pitchPreviousError;
float pitchIntegral, pitchDerivative;
float pitchOutput;

//PID ROLL
float rollError, rollPreviousError;
float rollIntegral, rollDerivative;
float rollOutput;

unsigned long timer = micros();

void setup() {
  Wire.begin(); //inicializa I2c
  Serial.begin(115200);
  mpu.initialize(); //inicializa MPU 

  //Inicializa filtro Kalman
  kalmanPitch.setAngle(0);
  kalmanRoll.setAngle(0);
  
  //temporizador
  timer = micros();

  
  //CONEXAO WIFI
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED){

    delay(1000);
    Serial.println("connecting to WIFI ...");
  }
  
  Serial.println("conectado");

// Define the "/data" route to send sensor data
  server.on("/data", HTTP_GET, []() {
    String json = "{\"pitch\":" + String(pitch) + ", \"roll\":" + String(roll) +
                  ", \"pidPitchOutput\":" + String(pitchOutput) + 
                  ", \"pidRollOutput\":" + String(rollOutput) + "}"; server.send(200,"application/json",json);});

   //START SERVER
   server.begin();
   Serial.println("server started");
   Serial.println("Access: hhtp://");
   Serial.println(WiFi.localIP());
}

void loop() {
  
   //leitura MPU
  mpu.getMotion6(&acelX, &acelY, &acelZ, &giroX, &giroY, &giroZ);

  // conversao de dados do acelerometro para unidades fisicas, utilizando faixa 2g, pode variar de sensor para sensor ***verificar***
  float acelX_g = acelX / 16384.0; 
  float acelY_g = acelY / 16384.0;
  float acelZ_g = acelZ / 16384.0;  

  //conversao de dados do giroscopio para unidade fisica, feixa utilizada foi +-250*/S
  float giroX_deg_s = giroX / 131.0; 
  float giroY_deg_s = giroY / 131.0;
  float giroZ_deg_s = giroZ / 131.0;

  //VARIAÇAO DE TEMPO
  float dt = (micros() - timer)/ 1000000.0;
  timer = micros();

  //calcula o angulo baseado nos dados fornecidos pelo ACELEROMETRO
  float pitchAcel = atan2(acelY, acelZ) * 180 / PI; 
  float rollAcel = atan2(acelX, acelZ) * 180 / PI;

  //PASSA OS DADOS PELO FILTRO DE KALMAN, RETORNANDO ANGULOS MAIS PRECISOS
  pitch = kalmanPitch.getAngle(pitchAcel, giroY_deg_s, dt); // O filtro de kalman usa o valor da VARIAÇAO do angulo "giroY_deg_s" e o angulo fornecido pelo calculo usando aceleraçao e retorna o valor "corrigido"
  roll = kalmanRoll.getAngle(rollAcel, giroX/131, dt);

  //CONTROLE PID PITCH --> APENAS PI
  pitchError = setPointPitch - pitch;
  pitchIntegral += pitchError * dt;
  pitchDerivative = (pitchError - pitchPreviousError) / dt;
  pitchOutput = Kp * pitchError + Ki * pitchIntegral + Kd * pitchDerivative;
  
  pitchPreviousError = pitchError;
  
  
  //CONTROLE PID ROLL --> APENAS PI  
  rollError = setPointRoll - pitch;
  rollIntegral += rollError * dt;
  rollDerivative = (rollError - rollPreviousError) / dt;
  pitchOutput = Kp * rollError + Ki * rollIntegral + Kd * rollDerivative;
  
  rollPreviousError = rollError;

  server.handleClient();
}
