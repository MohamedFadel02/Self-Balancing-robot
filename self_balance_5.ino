
#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

MPU6050 mpu;

int stepsPerRevolution = 200;
int motor1Step = 3;
int motor1Dir = 2;
int motor2Step = A3;
int motor2Dir = A2;

float setpoint = 0.0;
float Kp = 20.0;
float Ki = 0.0;
float Kd = 0.0;
float prevAngleX = 0.0;
float prevAngleY = 0.0;

float integral = 0.0;  // Added integral here

#define TRIGGER_PIN 6
#define ECHO_PIN 7

const uint64_t pipe = 0xE8E8F0F0E1LL;
RF24 radio(9, 10);

void setup() {
 Serial.begin(9600);

  mpu.initialize();
  mpu.setXGyroOffset(-1);
  mpu.setYGyroOffset(-1);
  mpu.setZGyroOffset(-1);
  mpu.setXAccelOffset(-1);
  mpu.setYAccelOffset(-1);
  mpu.setZAccelOffset(-1);

  pinMode(motor1Step, OUTPUT);
  pinMode(motor1Dir, OUTPUT);
  pinMode(motor2Step, OUTPUT);
  pinMode(motor2Dir, OUTPUT);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  radio.begin();
  radio.openWritingPipe(pipe);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop() {
  float angle = getAngle();
  float error = setpoint - angle;
  integral += error;  // Update integral term
  float derivative = error - prevAngleX;

  float controlSignal = (Kp * error) + (Ki * integral) + (Kd * derivative);

  adjustMotorSpeeds(controlSignal);

  prevAngleX = angle;

  float distance = getDistance();

  Serial.print("Angle: ");
  Serial.print(angle);
  Serial.print(" Setpoint: ");
  Serial.print(setpoint);
  Serial.print(" Control Signal: ");
  Serial.print(controlSignal);
  Serial.print(" Motor 1 Speed: ");
  Serial.print(motor1Step);
  Serial.print(" Motor 2 Speed: ");
  Serial.println(motor2Step);
  Serial.print(" Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  delay(2000);

  radio.write(&distance, sizeof(float));

  delay(10);
}

float getAngle() {
  int16_t rawAccelX, rawAccelY, rawAccelZ;
  mpu.getAcceleration(&rawAccelX, &rawAccelY, &rawAccelZ);
  
  float accelAngleX = atan2((float)rawAccelY, (float)rawAccelZ) * RAD_TO_DEG;
  float accelAngleY = atan2((float)-rawAccelX, (float)rawAccelZ) * RAD_TO_DEG;

  int16_t rawGyroX, rawGyroY, rawGyroZ;
  mpu.getRotation(&rawGyroX, &rawGyroY, &rawGyroZ);
  
  float gyroRateX = ((float)rawGyroX - mpu.getXGyroOffset()) / 131.0;
  float gyroRateY = ((float)rawGyroY - mpu.getYGyroOffset()) / 131.0;

  float deltaAngleX = gyroRateX * 0.01;
  float deltaAngleY = gyroRateY * 0.01;

  float angleX = 0.98 * (prevAngleX + deltaAngleX) + 0.02 * accelAngleX;
  float angleY = 0.98 * (prevAngleY + deltaAngleY) + 0.02 * accelAngleY;

  prevAngleX = angleX;
  prevAngleY = angleY;

  return angleX; // Return the angle in X-axis
}

float getDistance() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  float duration = pulseIn(ECHO_PIN, HIGH);
  float distance = (duration * 0.034) / 2;  // Speed of sound = 343 m/s
  return distance;
}

void adjustMotorSpeeds(float controlSignal) {
  if (controlSignal > 0.0) {
    digitalWrite(motor1Dir, HIGH);
    digitalWrite(motor2Dir, HIGH);

    int steps1 = controlSignal * stepsPerRevolution / 360;
    int steps2 = controlSignal * stepsPerRevolution / 360;

    for (int i = 0; i < steps1; i++) {
      digitalWrite(motor1Step, HIGH);
      delayMicroseconds(500);
      digitalWrite(motor1Step, LOW);
      delayMicroseconds(500);
    }

    for (int i = 0; i < steps2; i++) {
      digitalWrite(motor2Step, HIGH);
      delayMicroseconds(500);
      digitalWrite(motor2Step, LOW);
      delayMicroseconds(500);
    }
  }
  else if (controlSignal < 0.0) {
    controlSignal = -controlSignal; 
    digitalWrite(motor1Dir, LOW);
    digitalWrite(motor2Dir, LOW);

    int steps1 = controlSignal * stepsPerRevolution / 360;
    int steps2 = controlSignal * stepsPerRevolution / 360;

    for (int i = 0; i < steps1; i++) {
      digitalWrite(motor1Step, HIGH);
      delayMicroseconds(500);
      digitalWrite(motor1Step, LOW);
      delayMicroseconds(500);
    }

    for (int i = 0; i < steps2; i++) {
      digitalWrite(motor2Step, HIGH);
      delayMicroseconds(500);
      digitalWrite(motor2Step, LOW);
      delayMicroseconds(500);
    }
  }
  else {
    digitalWrite(motor1Step, LOW);
    digitalWrite(motor2Step, LOW);
  }
}
