#include <Wire.h>
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

// extended kalman filter for realtime pose estimation and correction
class EKF {
public:
  float x, y, theta;  // state

  float P[9];  // covariance matrix (3x3)
  float Q[9];  // process (odometry) noise matrix
  float R[9];  // measurement (SLAM) noise matrix

  // ekf buffers
  float F[9], FP[9], FPFt[9];  // F, F * P, (F * P) * F^T
  float S[9], S_inv[9], K[9], I_m_K[9], newP[9];
  float Z[3], KZ[3];

  EKF() {
    x = 0;
    y = 0;
    theta = 0;

    float P_init[9] = { 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1 };
    memcpy(P, P_init, sizeof(P));

    float Q_init[9] = { 0.012, 0, 0, 0, 0.012, 0, 0, 0, 0.004 };
    memcpy(Q, Q_init, sizeof(Q));

    float R_init[9] = { 0.04, 0, 0, 0, 0.04, 0, 0, 0, 0.015 };
    memcpy(R, R_init, sizeof(R));
  }

  /// predicts position using odometry data
  void predict(float v, float omega, float dt) {
    theta += omega * dt;

    // normalize angle
    while (theta > PI)
      theta -= 2 * PI;
    while (theta < -PI)
      theta += 2 * PI;

    float st = sinf(theta);
    float ct = cosf(theta);

    x += v * dt * ct;
    y += v * dt * st;

    // jacobian matrix
    F[0] = 1.0;
    F[1] = 0.0;
    F[2] = -v * dt * st;
    F[3] = 0.0;
    F[4] = 1.0;
    F[5] = v * dt * ct;
    F[6] = 0.0;
    F[7] = 0.0;
    F[8] = 1.0;

    // mat mul
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        FP[i * 3 + j] = 0;

        for (int k = 0; k < 3; k++) {
          FP[i * 3 + j] += F[i * 3 + k] * P[k * 3 + j];
        }
      }
    }

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        FPFt[i * 3 + j] = 0;
        for (int k = 0; k < 3; k++) {
          FPFt[i * 3 + j] += FP[i * 3 + k] * F[j * 3 + k];
        }
      }
    }

    for (int i = 0; i < 9; i++) {
      P[i] = FPFt[i] + Q[i];
    }
  }

  /// updates position using SLAM data
  void update(float sx, float sy, float stheta) {
    float yaw_err = stheta - theta;

    // normalize angle
    while (yaw_err > PI)
      yaw_err -= 2 * PI;
    while (yaw_err < -PI)
      yaw_err += 2 * PI;

    // error matrix
    Z[0] = sx - x;
    Z[1] = sy - y;
    Z[2] = yaw_err;

    for (int i = 0; i < 9; i++) {
      S[i] = P[i] + R[i];
    }

    // invert S
    if (!invert3x3(S, S_inv)) {
      return;
    }

    // kalman coef
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        K[i * 3 + j] = 0;

        for (int k = 0; k < 3; k++) {
          K[i * 3 + j] += P[i * 3 + k] * S_inv[k * 3 + j];
        }
      }
    }

    KZ[0] = KZ[1] = KZ[2] = 0.0f;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        KZ[i] += K[i * 3 + j] * Z[j];
      }
    }

    x += KZ[0];
    y += KZ[1];
    theta += KZ[2];

    // normalize angle
    while (theta > PI)
      theta -= 2 * PI;
    while (theta < -PI)
      theta += 2 * PI;

    // update covariance P = (I - K) * P;
    I_m_K[0] = 1 - K[0];
    I_m_K[1] = -K[1];
    I_m_K[2] = -K[2];
    I_m_K[3] = -K[3];
    I_m_K[4] = 1 - K[4];
    I_m_K[5] = -K[5];
    I_m_K[6] = -K[6];
    I_m_K[7] = -K[7];
    I_m_K[8] = 1 - K[8];

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        newP[i * 3 + j] = 0;
        for (int k = 0; k < 3; k++) {
          newP[i * 3 + j] += I_m_K[i * 3 + k] * P[k * 3 + j];
        }
      }
    }

    memcpy(P, newP, sizeof(P));
  }

private:
  bool invert3x3(const float m[9], float inv[9]) {
    float det = m[0] * (m[4] * m[8] - m[7] * m[5]) - m[1] * (m[3] * m[8] - m[6] * m[5]) + m[2] * (m[3] * m[7] - m[6] * m[4]);

    if (fabs(det) < 1e-8) return false;

    float invDet = 1.0 / det;
    inv[0] = (m[4] * m[8] - m[7] * m[5]) * invDet;
    inv[1] = (m[2] * m[7] - m[1] * m[8]) * invDet;
    inv[2] = (m[1] * m[5] - m[2] * m[4]) * invDet;
    inv[3] = (m[5] * m[6] - m[3] * m[8]) * invDet;
    inv[4] = (m[0] * m[8] - m[2] * m[6]) * invDet;
    inv[5] = (m[2] * m[3] - m[0] * m[5]) * invDet;
    inv[6] = (m[3] * m[7] - m[4] * m[6]) * invDet;
    inv[7] = (m[1] * m[6] - m[0] * m[7]) * invDet;
    inv[8] = (m[0] * m[4] - m[1] * m[3]) * invDet;

    return true;
  }
};

// hardware constants
const float WHEEL_RADIUS = 0.0189;
const float WHEEL_BASE = 0.198;

// this value depends on motor
// how to measure this:
// 1. uncomment code in the loop()
// 2. select any dot on the wheel so you can track its rotation easily
// 3. rotate wheel by 360 degrees exactly 10 times
// 4. get last serial output, divide it by 10 and round to the next integer
// you may turn wheel more times to get better results, but usually 10 is enough
const int TICKS_PER_REV = 682;
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
EKF filter;
float v = 0.0, omega = 0.0;

// wheels target speeds
float speedLeft = 0.0, speedRight = 0.0;

// PID constants
const float Kp = 0.8;
const float Ki = 15.0;
const float Kd = 0;
const float PID_LIMIT = 255.0;

// PID variables
float leftIntegral = 0.0, rightIntegral = 0.0;
float leftPrevError = 0.0, rightPrevError = 0.0;

// time
unsigned long lastUpdateTime = 0;
unsigned long lastMicros = 0;

// complementary filter dynamic value
float alpha = 0.0;

// sign function
template<typename T> int sign(T val) {
  return (T(0) < val) - (val < T(0));
}

void setup() {
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(L_EN, OUTPUT);

  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(R_EN, OUTPUT);

  Serial.begin(115200);
  Wire.begin();

  mpu.initialize();

  // if mpu is not available
  // run infinite loop
  if (!mpu.testConnection()) {
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

void calibrateGyro(int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += mpu.getRotationZ();
    delay(1);
  }

  gyroZ_offset = sum / samples;
}

uint8_t ticks = 0;
void loop() {
  // if (millis() - lastUpdateTime >= 500) {
  //   lastUpdateTime = millis();

  //   Serial.println(leftEnc.read());
  // }
  // return;

  handleSerialInput();

  // update every 10 ms
  if (millis() - lastUpdateTime >= 10) {
    ticks += 1;

    unsigned long nowMicros = micros();
    float dt = (nowMicros - lastMicros) / 1000000.0;  // seconds

    if (dt <= 0.0 || dt > 0.25) {
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

    int16_t gyroRaw = mpu.getRotationZ() - gyroZ_offset;
    float omega_gyro = gyroRaw * GYRO_SCALE;
    float omega_enc = dThetaEnc / dt;

    // recalculate alpha value
    if (abs(deltaLeft) > 0 || abs(deltaRight) > 0) {
      alpha = 0.98;
    } else {
      alpha = 0;
    }

    // use complementary filter and integrate
    omega = omega_gyro * alpha + omega_enc * (1 - alpha);
    v = dCenter / dt;

    // update motor signals
    motorControl(dt, dLeft, dRight);
    filter.predict(v, omega, dt);

    // reset time
    lastUpdateTime = millis();
    lastMicros = nowMicros;
  }

  // send every 100 ms
  if (ticks >= 10) {
    ticks = 0;

    Serial.print(filter.x, 4);
    Serial.print(" ");
    Serial.print(filter.y, 4);
    Serial.print(" ");
    Serial.print(filter.theta, 4);
    Serial.print(" ");
    Serial.print(v, 4);
    Serial.print(" ");
    Serial.println(omega, 4);
  }
}

char cmd[25];
void handleSerialInput() {
  if (Serial.available() > 0) {
    Serial.readBytesUntil('\n', cmd, 25);
    char* token = strtok(cmd, " ");
    if (!token) return;

    if (token[0] == 'R') {
      token = strtok(NULL, " ");
      if (!token) return;
      float newX = atof(token);

      token = strtok(NULL, " ");
      if (!token) return;
      float newY = atof(token);

      token = strtok(NULL, " ");
      if (!token) return;
      float newH = atof(token);

      // old: hard reset variables
      // x = newX;
      // y = newY;
      // heading = newH;

      // new: use EKF to estimate position
      filter.update(newX, newY, newH);
    } else if (token[0] == 'S') {
      token = strtok(NULL, " ");
      if (!token) return;
      float newLeft = atof(token);

      token = strtok(NULL, " ");
      if (!token) return;
      float newRight = atof(token);

      speedLeft = newLeft;
      speedRight = newRight;
    }
  }
}

void setMotor(int in1, int in2, int en, int pwm) {
  if (pwm > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (pwm < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  if (pwm > 255) {
    pwm = 255;
  } else if (pwm < -255) {
    pwm = -255;
  }

  if (pwm < 0) {
    pwm *= -1;
  }

  analogWrite(en, pwm);
}

void motorControl(float dt, float dLeft, float dRight) {
  if (dt < 0.0001f) dt = 0.0001f;

  float leftMeas = (dLeft / WHEEL_RADIUS) / dt;
  float rightMeas = (dRight / WHEEL_RADIUS) / dt;

  float leftErr = speedLeft - leftMeas;
  float rightErr = speedRight - rightMeas;

  leftIntegral += leftErr * dt;
  leftIntegral = constrain(leftIntegral, -PID_LIMIT / Ki, PID_LIMIT / Ki);

  rightIntegral += rightErr * dt;
  rightIntegral = constrain(rightIntegral, -PID_LIMIT / Ki, PID_LIMIT / Ki);

  float leftDeriv = (leftErr - leftPrevError) / dt;
  float rightDeriv = (rightErr - rightPrevError) / dt;

  int leftPwm = (int)(Kp * leftErr + Ki * leftIntegral + Kd * leftDeriv);
  int rightPwm = (int)(Kp * rightErr + Ki * rightIntegral + Kd * rightDeriv);

  leftPrevError = leftErr;
  rightPrevError = rightErr;

  setMotor(L_IN1, L_IN2, L_EN, leftPwm);
  setMotor(R_IN1, R_IN2, R_EN, rightPwm * -1);
}
