#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define ABS(a) ((a > 0) ? (a) : (-a))

// Pins
#define PWM_A 10
#define PWM_B 11

#define AA 8
#define AB 9
#define BA 12
#define BB 13

// Packet vars
#define PACKET_PING  'P'
#define PACKET_DATA  'D'

// Speeds
#define MAX_SPEED  10.0

union SpeedsData {
  byte bytes[8];
  struct {
    float left;
    float right;
  } floats;
} speedsData;

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  lcd.init();
  // lcd.noBacklight();
  lcd.backlight();

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

  Serial.begin(9600);
  Serial.println("[info] Running");
}

void loop() {
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
}

void _handlePing() {
  // send pong
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

  int val_A = (int) (min(ABS(left), MAX_SPEED) / MAX_SPEED * 255);
  int val_B = (int) (min(ABS(right), MAX_SPEED) / MAX_SPEED * 255);

  analogWrite(PWM_A, val_A);
  analogWrite(PWM_B, val_B);

  digitalWrite(AA, val_AA);
  digitalWrite(BA, val_BA);
  digitalWrite(AB, val_AB);
  digitalWrite(BB, val_BB);
}

