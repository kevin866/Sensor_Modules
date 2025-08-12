#include <Wire.h>
#include <SerLCD.h>
#include <SharpIR.h>

#define TRIG_PIN 9
#define ECHO_PIN 10
#define IR_SENSOR A0
SharpIR sensor(SharpIR::GP2Y0A41SK0F, A0);

SerLCD lcd;

static uint8_t frame[9];
static int index = 0;
static uint16_t tfDistance = 0;
static uint16_t tfStrength = 0;

// History size
const int HISTORY_SIZE = 5;
float usHistory[HISTORY_SIZE] = {0};
float irHistory[HISTORY_SIZE] = {0};
float lidarHistory[HISTORY_SIZE] = {0};
int historyIndex = 0;

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
  Wire.begin();

  lcd.begin(Wire);
  lcd.setBacklight(255, 255, 255);
  lcd.setContrast(5);
  lcd.clear();
  lcd.print("Initializing...");

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  delay(2000);
  lcd.clear();
}

void loop() {
  // --- Ultrasonic ---
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  float ultrasonicDistance = duration * 0.034 / 2.0;

  // --- IR ---
  float irDistance = sensor.getDistance() * 2.54;

  // --- LiDAR ---
  while (Serial3.available()) {
    uint8_t b = Serial3.read();
    if (index == 0 && b != 0x59) continue;
    frame[index++] = b;
    if (index == 2 && frame[1] != 0x59) { index = 0; continue; }
    if (index == 9) {
      index = 0;
      uint16_t distance = frame[2] + (frame[3] << 8);
      uint8_t checksum = 0;
      for (int i = 0; i < 8; i++) checksum += frame[i];
      if (checksum == frame[8]) {
        tfDistance = distance;
        tfStrength = frame[4] + (frame[5] << 8);
      }
    }
  }

  float lidarDistance = tfDistance;

  // --- Save readings to history ---
  usHistory[historyIndex] = ultrasonicDistance;
  irHistory[historyIndex] = irDistance;
  lidarHistory[historyIndex] = lidarDistance;
  historyIndex = (historyIndex + 1) % HISTORY_SIZE;

  // --- Select reference sensor ---
  float reference = (lidarDistance > 25) ? lidarDistance : ultrasonicDistance;

  // --- Find best match from each history ---
  float bestUS = usHistory[0], bestIR = irHistory[0], bestLiDAR = lidarHistory[0];
  float bestUSDiff = abs(usHistory[0] - reference);
  float bestIRDiff = abs(irHistory[0] - reference);
  float bestLiDARDiff = abs(lidarHistory[0] - reference);

  for (int i = 1; i < HISTORY_SIZE; i++) {
    float d;

    d = abs(usHistory[i] - reference);
    if (d < bestUSDiff) { bestUS = usHistory[i]; bestUSDiff = d; }

    d = abs(irHistory[i] - reference);
    if (d < bestIRDiff) { bestIR = irHistory[i]; bestIRDiff = d; }

    d = abs(lidarHistory[i] - reference);
    if (d < bestLiDARDiff) { bestLiDAR = lidarHistory[i]; bestLiDARDiff = d; }
  }

  // --- Serial Output ---
  Serial.print("Raw: US=");
  Serial.print(ultrasonicDistance, 1);
  Serial.print(" IR=");
  Serial.print(irDistance, 1);
  Serial.print(" LiDAR=");
  Serial.print(lidarDistance, 1);
  Serial.print(" | Smoothed: US=");
  Serial.print(bestUS, 1);
  Serial.print(" IR=");
  Serial.print(bestIR, 1);
  Serial.print(" LiDAR=");
  Serial.println(bestLiDAR, 1);

  // --- LCD Output ---
  lcd.setCursor(0, 0);
  lcd.print("US     IR     LiDAR");

  lcd.setCursor(0, 1);
  lcd.print(bestUS, 1);
  lcd.setCursor(0, 2);
  lcd.print("cm");

  lcd.setCursor(7, 1);
  lcd.print(bestIR, 1);
  lcd.setCursor(7, 2);
  lcd.print("cm");

  lcd.setCursor(14, 1);
  lcd.print(bestLiDAR, 1);
  lcd.setCursor(14, 2);
  lcd.print("cm");

  delay(100);
}
