#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Encoder.h>

const float WHEEL_RADIUS = 0.027;
const float WHEEL_BASE = 0.160;
const int TICKS_PER_REV = 0;
const float DIST_PER_TICK = (2 * PI * WHEEL_RADIUS) / TICKS_PER_REV;

#define LEFT_ENC_A 2
#define LEFT_ENC_B 4
#define RIGHT_ENC_A 3
#define RIGHT_ENC_B 5
Encoder leftEnc(LEFT_ENC_A, LEFT_ENC_B);
Encoder rightEnc(RIGHT_ENC_A, RIGHT_ENC_B);

#define L_IN1 7
#define L_IN2 8
#define L_EN 9

#define R_IN1 12
#define R_IN2 13
#define R_EN 10

MPU6050 mpu;
int16_t gyroZ_offset = 0;
const float GYRO_SCALE = PI / (180.0 * 16.4);

float x = 0.0, y = 0.0, heading = 0.0;
float v = 0.0, omega = 0.0;
float gyro_bias = 0.0;

float speedLeft = 0.0, speedRight = 0.0;

const float Kp = 1.2;
const float Ki = 3.0;
const float Kd = 0.0;
const float PID_LIMIT = 255.0;

float leftIntegral = 0.0, rightIntegral = 0.0;
float leftPrevError = 0.0, rightPrevError = 0.0;

unsigned long lastUpdateTime = 0;
unsigned long lastMicros = 0;

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  mpu.initialize();
  if (!mpu.testConnection())
  {
    while (1)
      ;
  }

  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  calibrateGyro(2000);

  lastUpdateTime = millis();
  lastMicros = micros();
}

void calibrateGyro(int samples)
{
  long sum = 0;
  for (int i = 0; i < samples; i++)
  {
    sum += mpu.getRotationZ();
    delay(1);
  }

  gyroZ_offset = sum / samples;

  Serial.print("Gyro Z offs: ");
  Serial.println(gyroZ_offset);
}

uint8_t ticks = 0;
void loop()
{
  handleSerialInput();

  if (millis() - lastUpdateTime >= 10)
  {
    ticks += 1;

    unsigned long nowMicros = micros();
    float dt = (nowMicros - lastMicros) / 1000000.0; // seconds

    if (dt <= 0.0 || dt > 0.25)
    {
      lastUpdateTime = millis();
      lastMicros = nowMicros;
      return;
    }

    static long lastLeftTicks = leftEnc.read();
    static long lastRightTicks = rightEnc.read();

    long leftTicks = leftEnc.read();
    long rightTicks = rightEnc.read();

    long deltaLeft = leftTicks - lastLeftTicks;
    long deltaRight = rightTicks - lastRightTicks;

    lastLeftTicks = leftTicks;
    lastRightTicks = rightTicks;

    float dLeft = deltaLeft * DIST_PER_TICK;
    float dRight = deltaRight * DIST_PER_TICK;

    float dCenter = (dLeft + dRight) / 2.0;
    float dThetaEnc = (dRight - dLeft) / WHEEL_BASE;

    int16_t gyroRaw = mpu.getRotationZ() - gyroZ_offset;
    float omega_gyro = gyroRaw * GYRO_SCALE;
    float omega_enc = dThetaEnc / dt;

    float error = omega_gyro - omega_enc;

    // slow correction
    const float beta = 0.15;
    gyro_bias += beta * error * dt;

    omega = omega_gyro - gyro_bias;

    v = dCenter / dt;

    heading += omega * dt;

    // normalize
    while (heading > PI)
      heading -= 2 * PI;
    while (heading < -PI)
      heading += 2 * PI;

    x += v * cos(heading) * dt;
    y += v * sin(heading) * dt;

    motorControl(dt, dLeft, dRight);

    lastUpdateTime = millis();
    lastMicros = nowMicros;
  }

  if (ticks >= 5)
  {
    ticks = 0;

    Serial.print(x, 4);
    Serial.print(" ");
    Serial.print(y, 4);
    Serial.print(" ");
    Serial.print(heading, 4);
    Serial.print(" ");
    Serial.print(v, 4);
    Serial.print(" ");
    Serial.println(omega, 4);
  }
}

void handleSerialInput()
{
  if (Serial.available() > 0)
  {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("R "))
    {
      cmd.remove(0, 2);

      float newX, newY, newH;
      int n = sscanf(cmd.c_str(), "%f %f %f", &newX, &newY, &newH);

      if (n == 3)
      {
        x = newX;
        y = newY;
        heading = newH;

        gyro_bias = 0.0;
        leftIntegral = 0;
        rightIntegral = 0;
      }
    }
    else if (cmd.startsWith("S "))
    {
      cmd.remove(0, 2);

      float newLeft, newRight;
      int n = sscanf(cmd.c_str(), "%f %f", &newLeft, &newRight);

      if (n == 2)
      {
        speedLeft = newLeft;
        speedRight = newRight;

        leftIntegral = 0;
        rightIntegral = 0;
      }
    }
  }
}

void setMotor(int in1, int in2, in en, int pwm)
{
  if (pwm > 0)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (pwm < 0)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  if (pwm > 255)
  {
    pwm = 255;
  }
  else if (pwm < -255)
  {
    pwm = -255;
  }

  if (pwm < 0)
  {
    pwm *= -1;
  }

  analogWrite(en, pwm);
}

void motorControl(float dt, float dLeft, float dRight) {
  float leftMeas = (dLeft / WHEEL_RADIUS) / dt;
  float rightMeas = (dRight / WHEEL_RADIUS) / dt;

  float leftErr = speedLeft - leftMeas;
  float rightErr = speedRight - rightMeas;

  leftIntegral += leftErr * dt;
  rightIntegral += rightErr * dt;

  leftIntegral = constrain(leftIntegral, -PID_LIMIT / Ki, PID_LIMIT / Ki);
  rightIntegral = constrain(rightIntegral, -PID_LIMIT / Ki, PID_LIMIT / Ki);

  float leftDeriv = (leftErr - leftPrevError) / dt;
  float rightDeriv = (rightErr - rightPrevError) / dt;

  int leftPwm = (int) (Kp * leftErr + Ki * leftIntegral + Kd * leftDeriv);
  int rightPwm = (int) (Kp * rightErr + Ki * rightIntegral + Kd * rightDeriv);

  leftPrevError = leftErr;
  rightPrevError = rightErr;

  setMotor(L_IN1, L_IN2, L_EN, leftPwm);
  setMotor(R_IN1, R_IN2, R_EN, rightPwm);
}