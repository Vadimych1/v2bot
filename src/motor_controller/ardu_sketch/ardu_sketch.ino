#include "MPU6050.h"
#include <Encoder.h>
#include <math.h>

#define BUFFER_SIZE 100 // calibration data

// Pins
#define PWM_A 10
#define PWM_B 11

#define AA 8
#define AB 9
#define BA 12
#define BB 13

#define ENC_A1 2
#define ENC_A2 3

#define ENC_B1 4
#define ENC_B2 5

// Packet vars
#define PACKET_PING 'P'
#define PACKET_DATA 'D'

// Speeds & encoders
#define MAX_SPEED 10.0f
#define PPR 11
#define GEAR_RATIO 100 // TODO: change that value to real reduction ratio
#define BASELINE 0.16f

const int TICKS_PER_REV = PPR * GEAR_RATIO;
const float WHEEL_CIRCUMFERENCE = M_PI * 0.04;
const float DIST_PER_TICK = WHEEL_CIRCUMFERENCE / ((float) TICKS_PER_REV);

union SpeedsData {
  byte bytes[8];
  struct {
    float left;
    float right;
  } floats;
} speedsData;

MPU6050 mpu;
Encoder encA(ENC_A1, ENC_A2);
Encoder encB(ENC_B1, ENC_B2);

long aTicks = 0;
long bTicks = 0;
unsigned long prevTime = 0;

int16_t ax, ay, az;  // raw acc
int16_t gx, gy, gz;  // raw gyro
float x = 0, y = 0, theta = 0;
uint8_t send_ctr = 0;

void setup() {
  pinMode(AA, OUTPUT);
  pinMode(AB, OUTPUT);
  pinMode(BA, OUTPUT);
  pinMode(BB, OUTPUT);

  digitalWrite(AA, LOW);
  digitalWrite(AB, LOW);
  digitalWrite(BA, LOW);
  digitalWrite(BB, LOW);

  pinMode(PWM_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);

  analogWrite(PWM_A, 0);
  analogWrite(PWM_B, 0);

  Wire.begin();
  Serial.begin(115200);

  mpu.initialize();
  delay(80);
  calibration();
}

void loop() {
  long newATicks = encA.read();
  long newBTicks = encB.read();

  unsigned long now = micros();
  float dt = (now - prevTime) / 1e6;
  if (dt < 0.02) return;
  prevTime = now;

  send_ctr++;

  long dA = newATicks - aTicks;
  long dB = newBTicks - bTicks;

  aTicks = newATicks;
  bTicks = newBTicks;

  float deltaA = dA * DIST_PER_TICK;
  float deltaB = dB * DIST_PER_TICK;

  float deltaS = (deltaA + deltaB) / 2.0;
  float deltaTheta = (deltaA - deltaB) / BASELINE;

  float dx = deltaS * cos(theta + deltaTheta / 2);
  float dy = deltaS * sin(theta + deltaTheta / 2);

  theta += deltaTheta;
  x += dx;
  y += dy;

  if (send_ctr == 5) {
    send_ctr = 0;

    Serial.print(x);
    Serial.print("\t");
    Serial.print(y);
    Serial.print("\t");
    Serial.println(theta);
  }

  //// TODO: add a gyro complementary filter
  // float gyroZ = readGyroZ();
  // float thetaGyro = thetaGyro + gyroZ * dt;
  // float alpha = 0.98;
  // theta = alpha * thetaGyro + (1 - alpha) * theta;

  if (Serial.available() > 0) {
    char pktType = Serial.read();

    switch (pktType) {
      case PACKET_PING:
        _handlePing();
        break;

      case PACKET_DATA:
        _handleData();
        break;

      default:
        while (Serial.available() > 0) {
          Serial.read();
        }
        break;
    }
  }

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

}

void calibration() {
  long offsets[6];
  long offsetsOld[6];
  int16_t mpuGet[6];

  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

  delay(10);

  Serial.println("Calibration start. It will take about 5 seconds");
  for (byte n = 0; n < 10; n++) {
    for (byte j = 0; j < 6; j++) {
      offsets[j] = 0;
    }
    for (byte i = 0; i < 100 + BUFFER_SIZE; i++) {
      mpu.getMotion6(&mpuGet[0], &mpuGet[1], &mpuGet[2], &mpuGet[3], &mpuGet[4], &mpuGet[5]);
      if (i >= 99) {
        for (byte j = 0; j < 6; j++) {
          offsets[j] += (long)mpuGet[j];
        }
      }
    }
    for (byte i = 0; i < 6; i++) {
      offsets[i] = offsetsOld[i] - ((long)offsets[i] / BUFFER_SIZE);
      offsetsOld[i] = offsets[i];
    }

    mpu.setXAccelOffset(offsets[0] / 8);
    mpu.setYAccelOffset(offsets[1] / 8);
    mpu.setZAccelOffset(offsets[2] / 8);
    mpu.setXGyroOffset(offsets[3] / 4);
    mpu.setYGyroOffset(offsets[4] / 4);
    mpu.setZGyroOffset(offsets[5] / 4);
    
    delay(2);
  }
}

void _handlePing() {
  Serial.println("PONG");

  // blink led
  digitalWrite(LED_BUILTIN, HIGH);
  delay(50);
  digitalWrite(LED_BUILTIN, LOW);
}

void _handleData() {
  while (Serial.available() < 8) {
    delayMicroseconds(100);
  }

  for (int i = 0; i < 8; i++) {
    speedsData.bytes[i] = Serial.read();
  }

  float left = speedsData.floats.left;
  float right = speedsData.floats.right;

  int val_AA = left > 0 ? HIGH : LOW;
  int val_AB = left < 0 ? HIGH : LOW;
  int val_BA = right < 0 ? HIGH : LOW;
  int val_BB = right > 0 ? HIGH : LOW;

  int val_A = (int)(min(abs(left), MAX_SPEED) / MAX_SPEED * 255);
  int val_B = (int)(min(abs(right), MAX_SPEED) / MAX_SPEED * 255);

  analogWrite(PWM_A, val_A);
  analogWrite(PWM_B, val_B);

  digitalWrite(AA, LOW);
  digitalWrite(BA, LOW);
  digitalWrite(AB, LOW);
  digitalWrite(BB, LOW);

  delayMicroseconds(10);

  digitalWrite(AA, val_AA);
  digitalWrite(BA, val_BA);
  digitalWrite(AB, val_AB);
  digitalWrite(BB, val_BB);
}
