#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Encoder.h>

// encoders pins
#define LEFT_ENC_A 2
#define LEFT_ENC_B 4
#define RIGHT_ENC_A 3
#define RIGHT_ENC_B 5

// left motor
#define L_IN1 7
#define L_IN2 8
#define L_EN 9

// right motor
#define R_IN1 12
#define R_IN2 13
#define R_EN 10

// hardware constants
const float WHEEL_RADIUS = 0.022;
// const float WHEEL_BASE = 0.1872;
const float WHEEL_BASE = 0.198;
const int TICKS_PER_REV = 660;
const float DIST_PER_TICK = (2 * PI * WHEEL_RADIUS) / TICKS_PER_REV;

// encoders
Encoder leftEnc(LEFT_ENC_A, LEFT_ENC_B);
Encoder rightEnc(RIGHT_ENC_A, RIGHT_ENC_B);

long lastLeftTicks = 0;
long lastRightTicks = 0;

// gyro configuration
MPU6050 mpu;
int16_t gyroZ_offset = 0;
const float GYRO_SCALE = PI / (180.0 * 16.4);

// orientation variables
float x = 0.0, y = 0.0, heading = 0.0;
float v = 0.0, omega = 0.0;
float gyro_bias = 0.0;

// wheels target speeds
float speedLeft = 0.0, speedRight = 0.0;

// PID constants
const float Kp = 1.2;
const float Ki = 3.0;
const float Kd = 0.0;
const float PID_LIMIT = 255.0;

// PID variables
float leftIntegral = 0.0, rightIntegral = 0.0;
float leftPrevError = 0.0, rightPrevError = 0.0;

// time
unsigned long lastUpdateTime = 0;
unsigned long lastMicros = 0;

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  mpu.initialize();

  // if mpu is not available
  // run infinite loop
  if (!mpu.testConnection())
  {
    while (1) {
      Serial.println("mpu failed");
      delay(1000);      
    }
  }

  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  calibrateGyro(2000);

  // reset time
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
}

uint8_t ticks = 0;
void loop()
{
  handleSerialInput();

  // update every 10 ms
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

    long leftTicks = leftEnc.read();
    long rightTicks = -rightEnc.read();

    long deltaLeft = leftTicks - lastLeftTicks;
    long deltaRight = rightTicks - lastRightTicks;

    lastLeftTicks = leftTicks;
    lastRightTicks = rightTicks;

    float dLeft = deltaLeft * DIST_PER_TICK;
    float dRight = deltaRight * DIST_PER_TICK;

    float dCenter = (dLeft + dRight) / 2.0;
    float dThetaEnc = (dRight - dLeft) / WHEEL_BASE;

    // int16_t gyroRaw = mpu.getRotationZ() - gyroZ_offset;
    // float omega_gyro = gyroRaw * GYRO_SCALE;
    float omega_enc = dThetaEnc / dt;

    // float error = omega_gyro - omega_enc;

    // // slow correction
    // const float beta = 0.15;
    // gyro_bias += beta * error * dt;

    // omega = omega_gyro - gyro_bias;

    omega = omega_enc;
    v = dCenter / dt;

    heading += omega * dt;

    // normalize heading angle to [-PI; PI]
    while (heading > PI)
      heading -= 2 * PI;
    while (heading < -PI)
      heading += 2 * PI;

    // integrate coordinates
    x += v * cos(heading) * dt;
    y += v * sin(heading) * dt;

    // update motor signals
    motorControl(dt, dLeft, dRight);

    // reset time
    lastUpdateTime = millis();
    lastMicros = nowMicros;
  }

  // send every 50 ms
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

void setMotor(int in1, int in2, int en, int pwm)
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

void motorControl(float dt, float dLeft, float dRight)
{
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

  int leftPwm = (int)(Kp * leftErr + Ki * leftIntegral + Kd * leftDeriv);
  int rightPwm = (int)(Kp * rightErr + Ki * rightIntegral + Kd * rightDeriv);

  leftPrevError = leftErr;
  rightPrevError = rightErr;

  setMotor(L_IN1, L_IN2, L_EN, leftPwm);
  setMotor(R_IN1, R_IN2, R_EN, rightPwm);
}
