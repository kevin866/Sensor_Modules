#include <Wire.h>
#include <SerLCD.h>
#include <SharpIR.h>

#define TRIG_PIN 9
#define ECHO_PIN 10
#define IR_SENSOR A0
SharpIR sensor( SharpIR::GP2Y0A41SK0F, A0 );

SerLCD lcd; // create an lcd object

// Variables for TFmini Plus LiDAR
static uint8_t frame[9];
static int index = 0;
static uint16_t tfDistance = 0;  // store the latest lidar distance
static uint16_t tfStrength = 0;
static int tol = 2;
void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);       // TFmini Plus LiDAR serial port
  Wire.begin();

  lcd.begin(Wire);
  lcd.setBacklight(255, 255, 255); // White backlight
  lcd.setContrast(5);
  lcd.clear();
  lcd.print("Initializing...");

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  delay(2000);
  lcd.clear();

  Serial.println("TFmini Plus - 9-byte parser");
}

void loop() {
  // --- Ultrasonic Sensor ---
  long duration;
  float ultrasonicDistance;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  ultrasonicDistance = duration * 0.034 / 2.0;

  // --- Infrared Sensor ---
  float irDistance = sensor.getDistance() * 2.54;

  // --- TFmini Plus LiDAR ---
  while (Serial3.available()) {
    uint8_t b = Serial3.read();
    if (index == 0 && b != 0x59) continue;
    frame[index++] = b;
    if (index == 2 && frame[1] != 0x59) { index = 0; continue; }
    if (index == 9) {
      index = 0;
      uint16_t distance = frame[2] + (frame[3] << 8);
      uint16_t strength = frame[4] + (frame[5] << 8);
      uint8_t checksum = 0;
      for (int i = 0; i < 8; i++) checksum += frame[i];
      if (checksum == frame[8]) {
        tfDistance = distance;
        tfStrength = strength;
      }
    }
  }

  float lidarDistance = tfDistance;

  // --- Sensor Fusion / Flicker Reduction ---
  float referenceDistance;
  bool useLidar = lidarDistance > 25;

  if (useLidar) {
    referenceDistance = lidarDistance;
    // Adjust ultrasonic and IR toward LiDAR
    ultrasonicDistance = abs(ultrasonicDistance - lidarDistance) < tol ? ultrasonicDistance : lidarDistance;
    irDistance = abs(irDistance - lidarDistance) < tol ? irDistance : lidarDistance;
  } else {
    referenceDistance = ultrasonicDistance;
    // Adjust IR and LiDAR toward Ultrasonic
    irDistance = abs(irDistance - ultrasonicDistance) < tol ? irDistance : ultrasonicDistance;
    lidarDistance = abs(lidarDistance - ultrasonicDistance) < tol ? lidarDistance : ultrasonicDistance;
  }

  // --- Serial Output ---
  Serial.print("Ultrasonic: ");
  Serial.print(ultrasonicDistance, 1);
  Serial.print(" cm\tIR: ");
  Serial.print(irDistance, 1);
  Serial.print(" cm\tLiDAR: ");
  Serial.print(lidarDistance, 1);
  Serial.println(" cm");

  // --- LCD Output ---
  lcd.setCursor(0, 0);
  lcd.print("US     IR     LiDAR");

  lcd.setCursor(0, 1);
  lcd.print(ultrasonicDistance, 1);
  lcd.setCursor(0, 2);
  lcd.print("cm");

  lcd.setCursor(7, 1);
  lcd.print(irDistance, 1);
  lcd.setCursor(7, 2);
  lcd.print("cm");

  lcd.setCursor(14, 1);
  lcd.print(lidarDistance, 1);
  lcd.setCursor(14, 2);
  lcd.print("cm");

  delay(100); // Optional: slight delay to reduce flickering
}