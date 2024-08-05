#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP32Servo.h>

// Inicialização do sensor MPU6050
Adafruit_MPU6050 mpu;

// Inicialização dos servos
Servo motorX;
Servo motorY;

// Variáveis do filtro de Kalman
float Q_angle = 0.001;
float Q_gyro = 0.003;
float R_angle = 0.03;
float angleX = 0.0, angleY = 0.0;
float biasX = 0.0, biasY = 0.0;
float rateX = 0.0, rateY = 0.0;
float P[2][2] = {{0, 0}, {0, 0}};

// Função do filtro de Kalman
float kalmanFilter(float newRate, float newAngle, float dt, float& angle, float& bias) {
    rate = newRate - bias;
    angle += dt * rate;
    
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_gyro * dt;
    
    float S = P[0][0] + R_angle;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;
    
    float y = newAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;
    
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];
    
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;
    
    return angle;
}

// Variáveis do PID
float Kp = 2.0, Ki = 5.0, Kd = 1.0;
float setpointX = 0.0, setpointY = 0.0;
float errorX, errorY, last_errorX, last_errorY;
float integralX, integralY, derivativeX, derivativeY;
float outputX, outputY;

// Função de controle PID
float pidControl(float error, float dt, float& last_error, float& integral) {
    integral += error * dt;
    derivative = (error - last_error) / dt;
    float output = Kp * error + Ki * integral + Kd * derivative;
    last_error = error;
    return output;
}

// Variáveis de tempo
unsigned long lastTime;

void setup() {
    Serial.begin(115200);
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) { delay(10); }
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Inicializar os servos
    motorX.attach(18); // Conectar ao pino 18
    motorY.attach(19); // Conectar ao pino 19
    
    lastTime = millis();
}

void loop() {
    // Ler os dados do giroscópio e acelerômetro
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Calcular o tempo decorrido
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // Aplicar o filtro de Kalman
    float kalmanAngleX = kalmanFilter(g.gyro.x, a.acceleration.x, dt, angleX, biasX);
    float kalmanAngleY = kalmanFilter(g.gyro.y, a.acceleration.y, dt, angleY, biasY);

    // Calcular o erro
    float errorX = setpointX - kalmanAngleX;
    float errorY = setpointY - kalmanAngleY;

    // Calcular a saída do PID
    float outputX = pidControl(errorX, dt, last_errorX, integralX);
    float outputY = pidControl(errorY, dt, last_errorY, integralY);

    // Controlar os atuadores
    motorX.write(outputX);
    motorY.write(outputY);

    // Espera para a próxima leitura
    delay(10);
}
