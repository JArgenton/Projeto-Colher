#include <Wire.h>        // Library for I2C communication
#include <MPU6050.h>     // Library for MPU6050 sensor
#include <Kalman.h>      // Library for Kalman filtering
#include <Servo.h>       // Library for controlling servos
#include <PID_v1.h>      // Library for PID control

// Sensor and motor definitions
MPU6050 baseSensor;  // Accelerometer/Gyroscope at the base
MPU6050 tipSensor;   // Accelerometer/Gyroscope at the tip

Kalman kalmanX;  // Kalman filter for X-axis
Kalman kalmanY;  // Kalman filter for Y-axis

Servo pitchMotor;  // Motor for pitch control
Servo rollMotor;   // Motor for roll control

// Sensor and motor pins
const int baseSensorAddr = 0x68; // I2C address for MPU6050
const int tipSensorAddr = 0x69;  // I2C address for second MPU6050
const int pitchMotorPin = 9;     // Pin for pitch motor
const int rollMotorPin = 10;     // Pin for roll motor

// Variables for storing sensor data and angles
double baseAccelX, baseAccelY, baseGyroX;
double tipAccelX, tipAccelY, tipGyroX;
double angleX, angleY;

// PID control variables
double setpointX = 0; // Desired pitch angle
double setpointY = 0; // Desired roll angle
double inputX, inputY; // Current angles
double outputX, outputY; // PID output for motors

// PID parameters (tuning required)
double Kp = 2.0, Ki = 5.0, Kd = 1.0;
PID pidX(&inputX, &outputX, &setpointX, Kp, Ki, Kd, DIRECT); // PID controller for pitch
PID pidY(&inputY, &outputY, &setpointY, Kp, Ki, Kd, DIRECT); // PID controller for roll

void setup() {
  Wire.begin();            // Initialize I2C communication
  Serial.begin(115200);    // Initialize serial communication at 115200 baud rate
  
  baseSensor.initialize(); // Initialize base sensor
  tipSensor.initialize();  // Initialize tip sensor
  
  // Check sensor connections
  if (!baseSensor.testConnection() || !tipSensor.testConnection()) {
    Serial.println("Failed to connect to one of the sensors");
    while (1); // Stay here if sensor connection fails
  }
  
  // Attach servos to their respective pins
  pitchMotor.attach(pitchMotorPin);
  rollMotor.attach(rollMotorPin);

  // Initialize Kalman filters with initial angle
  kalmanX.setAngle(0);
  kalmanY.setAngle(0);

  // Set PID controllers to automatic mode
  pidX.SetMode(AUTOMATIC);
  pidY.SetMode(AUTOMATIC);
}

void loop() {
  // Read base sensor data
  int16_t ax, ay, az, gx, gy, gz;
  baseSensor.getAcceleration(&ax, &ay, &az);
  baseSensor.getRotation(&gx, &gy, &gz);
  baseAccelX = ax; baseGyroX = gx;

  // Read tip sensor data
  tipSensor.getAcceleration(&ax, &ay, &az);
  tipSensor.getRotation(&gx, &gy, &gz);
  tipAccelX = ax; tipGyroX = gx;

  // Apply Kalman filter to calculate angles
  angleX = kalmanX.getAngle(tipAccelX, tipGyroX);
  angleY = kalmanY.getAngle(tipAccelY, tipGyroY);

  // Update PID input with current angles
  inputX = angleX;
  inputY = angleY;

  // Compute PID output
  pidX.Compute();
  pidY.Compute();

  // Map PID output to servo range (0 to 180 degrees)
  int pitchValue = map(outputX, -90, 90, 0, 180);
  int rollValue = map(outputY, -90, 90, 0, 180);

  // Write values to servos
  pitchMotor.write(pitchValue);
  rollMotor.write(rollValue);

  // Debugging information
  Serial.print("Angle X: "); Serial.print(angleX);
  Serial.print(" Pitch Motor: "); Serial.println(pitchValue);
  Serial.print("Angle Y: "); Serial.print(angleY);
  Serial.print(" Roll Motor: "); Serial.println(rollValue);

  delay(100); // Loop delay
}
