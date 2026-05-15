#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SerLCD.h>
#include <digitalWriteFast.h>

// =====================
// Qwiic LCD
// =====================
SerLCD lcd;

// =====================
// Quadrature Encoder
// =====================
#define QUAD_PIN_A 18   // white wire
#define QUAD_PIN_B 17   // black wire

volatile long quadTicks = 0;

// =====================
// AMT22 Absolute Encoder
// =====================
#define AMT22_CS_PIN 53
#define AMT22_RESOLUTION 14
#define AMT22_NOP 0x00

// =====================
// Multi-turn Potentiometer
// =====================
#define POT_PIN A14
#define POT_TURNS 10.0

// =====================
// Timing
// =====================
unsigned long lastLCDUpdate = 0;
const unsigned long LCD_UPDATE_INTERVAL = 200; // ms

void setup()
{
  Serial.begin(115200);

  // LCD setup
  Wire.begin();
  lcd.begin(Wire);
  lcd.setBacklight(255, 255, 255);
  lcd.setContrast(5);
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Encoder Test");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");
  delay(1000);
  lcd.clear();

  // Quadrature encoder setup
  pinMode(QUAD_PIN_A, INPUT);
  pinMode(QUAD_PIN_B, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(QUAD_PIN_A),
    handleQuadEncoder,
    RISING
  );

  // AMT22 setup
  pinMode(AMT22_CS_PIN, OUTPUT);
  digitalWrite(AMT22_CS_PIN, HIGH);

  // Important for Arduino Mega SPI
  pinMode(53, OUTPUT);

  SPI.begin();

  // Pot setup
  pinMode(POT_PIN, INPUT);
}

void loop()
{
  // Safely copy quadrature tick count
  noInterrupts();
  long quadTicksCopy = quadTicks;
  interrupts();

  // Read AMT22
  uint16_t amtRaw = readAMT22Position();
  float amtAngle = amtRaw * 360.0 / 16384.0;

  // Read potentiometer
  int potRaw = analogRead(POT_PIN);
  float potTurns = potRaw * POT_TURNS / 1023.0;
  float potAngle = potTurns * 360.0;

  // Serial debug
  Serial.print("Quad ticks: ");
  Serial.print(quadTicksCopy);

  Serial.print("\tAMT raw: ");
  Serial.print(amtRaw);

  Serial.print("\tAMT angle: ");
  Serial.print(amtAngle, 2);

  Serial.print("\tPot turns: ");
  Serial.println(potTurns, 2);

  // Update LCD slowly
  if (millis() - lastLCDUpdate >= LCD_UPDATE_INTERVAL)
  {
    updateLCD(quadTicksCopy, amtAngle, potTurns, potRaw);
    lastLCDUpdate = millis();
  }
}

void handleQuadEncoder()
{
  bool bState = digitalReadFast(QUAD_PIN_B);

  // Flip +1 and -1 if direction is reversed
  quadTicks += bState ? -1 : +1;
}

uint16_t readAMT22Position()
{
  uint16_t position = 0;

  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));

  digitalWrite(AMT22_CS_PIN, LOW);
  delayMicroseconds(3);

  position = SPI.transfer(AMT22_NOP) << 8;
  delayMicroseconds(3);
  position |= SPI.transfer(AMT22_NOP);

  delayMicroseconds(3);
  digitalWrite(AMT22_CS_PIN, HIGH);

  SPI.endTransaction();

  // Remove checksum bits
  position &= 0x3FFF;

  return position;
}


void updateLCD(long quadTicksValue, float amtAngle, float potTurns, int potRaw)
{
  lcd.setCursor(0, 0);
  lcd.print("Quad: ");
  lcd.print(quadTicksValue);
  lcd.print("        ");  // clear leftover characters

  lcd.setCursor(0, 1);
  lcd.print("AMT: ");
  lcd.print(amtAngle, 1);
  lcd.print(" deg     ");

  lcd.setCursor(0, 2);
  lcd.print("Pot: ");
  lcd.print(potTurns, 2);
  lcd.print(" turns   ");

  lcd.setCursor(0, 3);
  lcd.print("Pot raw: ");
  lcd.print(potRaw);
  lcd.print("        ");
}