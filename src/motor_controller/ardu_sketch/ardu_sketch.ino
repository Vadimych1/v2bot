#include "MPU6050.h"
#include <math.h>

#define BUFFER_SIZE 100

// Pins
#define PWM_A 10
#define PWM_B 11

#define AA 8
#define AB 9
#define BA 12
#define BB 13

// Packet vars
#define PACKET_PING 'P'
#define PACKET_DATA 'D'

// Speeds
#define MAX_SPEED 10.0

union SpeedsData {
  byte bytes[8];
  struct {
    float left;
    float right;
  } floats;
} speedsData;

MPU6050 mpu;

int16_t ax, ay, az;  // raw acc
int16_t gx, gy, gz;  // raw gyro

float theta = 0;
float vel = 0;
float xy = 0;
const float dt = 0.01;

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
  Serial.println(mpu.testConnection() ? "MPU6050 OK" : "MPU6050 FAIL");
  delay(80);
  calibration();
}

uint8_t ctr = 0;
void loop() {
  // Process incoming data
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

  float accX_f = ((float)ax) / 32768 * 2;
  float accY_f = ((float)ay) / 32768 * 2;
  // float accZ_f = ((float)az) / 32768 * 2;

  // float gyrX_f = ((float)gx) / 32768 * 250 / 180 * PI;
  // float gyrY_f = ((float)gy) / 32768 * 250 / 180 * PI;
  float gyrZ_f = ((float)gz) / 32768 * 250 / 180 * PI;

  if (speedsData.floats.left == 0.0 && speedsData.floats.right == 0.0) {
    vel = 0;
  }

  theta += gyrZ_f * dt;
  vel += accX_f * dt;  // simplified model: count only X axis movement
  xy += vel * dt;

  if (ctr >= 20) {
    Serial.print(theta, 6);
    Serial.print(",");
    Serial.print(xy, 6);
    Serial.print(",");
    Serial.println(0.2, 6);

    ctr = 0;
    theta = 0;
    xy = 0;
  }

  ctr += 1;

  delay(10);
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

  digitalWrite(AA, val_AA);
  digitalWrite(BA, val_BA);
  digitalWrite(AB, val_AB);
  digitalWrite(BB, val_BB);
}
