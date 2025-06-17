#include <Wire.h>
#include <SerLCD.h>

#define TRIG_PIN 9
#define ECHO_PIN 10
#define IR_SENSOR A0

SerLCD lcd; // create an lcd object

// Variables for TFmini Plus LiDAR
static uint8_t frame[9];
static int index = 0;
static uint16_t tfDistance = 0;  // store the latest lidar distance
static uint16_t tfStrength = 0;

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
  ultrasonicDistance = duration * 0.034 / 2.0; // Distance in cm

  // --- Infrared Sensor ---
  float volts = analogRead(IR_SENSOR) * 0.0048828125;
  int irDistance = 13 * pow(volts, -1);

  // --- TFmini Plus LiDAR reading ---
  while (Serial3.available()) {
    uint8_t b = Serial3.read();

    if (index == 0 && b != 0x59) {
      continue;
    }

    frame[index++] = b;

    if (index == 2 && frame[1] != 0x59) {
      index = 0;
      continue;
    }

    if (index == 9) {
      index = 0;

      uint16_t distance = frame[2] + (frame[3] << 8);
      uint16_t strength = frame[4] + (frame[5] << 8);
      uint8_t checksum = 0;
      for (int i = 0; i < 8; i++) checksum += frame[i];

      if (checksum == frame[8]) {
        tfDistance = distance;
        tfStrength = strength;

        // Serial.print("TFmini Distance: ");
        // Serial.print(tfDistance);
        // Serial.print(" cm | Strength: ");
        // Serial.println(tfStrength);
      } else {
        Serial.println("Checksum failed (9-byte frame)");
      }
    }
  }

  // Print ultrasonic and IR sensor data to Serial
  Serial.print("Ultrasonic: ");
  Serial.print(ultrasonicDistance);
  Serial.print(" cm\tIR: ");
  Serial.print(irDistance);
  Serial.print(" cm\tLiDAR: ");
  Serial.print(tfDistance);
  Serial.println(" cm");

// Print to LCD with sensor names on first line and values aligned below
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("US      IR     LiDAR");

  lcd.setCursor(0, 1);
  // Format the values with spacing to align under names
  lcd.print(ultrasonicDistance, 1);
  lcd.print("cm");
  
  lcd.setCursor(8, 1);
  lcd.print(irDistance);
  lcd.print("cm");

  lcd.setCursor(14, 1);
  lcd.print(tfDistance);
  lcd.print("cm");
  delay(1000);
}
