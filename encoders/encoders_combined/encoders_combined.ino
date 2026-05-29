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
#define QUAD_PIN_A 2   // white wire
#define QUAD_PIN_B 3   // black wire

volatile long quadTicks = 0;

// =====================
// AMT22 Absolute Encoder
// =====================
#define AMT22_CS_PIN 10
#define AMT22_RESOLUTION 14
#define AMT22_NOP 0x00

// =====================
// Multi-turn Potentiometer
// =====================
#define POT_PIN A0
#define POT_TURNS 10.0

// =====================
// Timing
// =====================
unsigned long lastLCDUpdate = 0;
const unsigned long LCD_UPDATE_INTERVAL = 200; // ms
const uint16_t AMT22_ZERO_OFFSET = 13714;

void setup()
{
  Serial.begin(115200);

  // LCD setup
  Wire1.begin();
  lcd.begin(Wire1);
  lcd.setBacklight(255, 255, 255);
  lcd.setContrast(5);
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Encoder Test");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");
  delay(1000);
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("     RAW       DEG");

  lcd.setCursor(0, 1);
  lcd.print("ABS");

  lcd.setCursor(0, 2);
  lcd.print("INC");

  lcd.setCursor(0, 3);
  lcd.print("POT");

  // Quadrature encoder setup
  pinMode(QUAD_PIN_A, INPUT);
  pinMode(QUAD_PIN_B, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(QUAD_PIN_A),
    handleQuadEncoder,
    RISING
  );


  // AMT22 setup
  SPI.begin();
  //SPISettings(500000, MSBFIRST, SPI_MODE0);

  pinMode(AMT22_CS_PIN, OUTPUT);
  digitalWrite(AMT22_CS_PIN, HIGH);

}

void loop()
{
  // Safely copy quadrature tick count
  noInterrupts();
  long quadTicksCopy = quadTicks;
  float encoder_inc_ang = 360.0 * ((float(quadTicksCopy))/1024.0);
  interrupts();

  // Read AMT22
  uint16_t amtRaw = readAMT22Position();
  Serial.print("AMT raw masked: ");
  Serial.println(amtRaw);
  // Offset and flip direction
  int32_t amtCounts = (int32_t)AMT22_ZERO_OFFSET - (int32_t)amtRaw;
  amtRaw = amtCounts;
  // Wrap to 0 ~ 16383
  amtCounts = (amtCounts + 16384) % 16384;

  // Convert to degrees
  float amtAngle = amtCounts * 360.0 / 16384.0;
  // Read potentiometer
  int potRaw = analogRead(POT_PIN);
  float potTurns = potRaw * POT_TURNS / 1023.0;
  float potAngle = potTurns * 360.0;

  // Serial debug

  Serial.print("\tABS RAW: ");
  Serial.print(amtRaw);
  Serial.print("\tDEG: ");
  Serial.print(amtAngle, 2);
  Serial.print("\t");

  Serial.print("\tINC RAW: ");
  Serial.print(quadTicksCopy);
  Serial.print("\tDEG: ");
  Serial.print(encoder_inc_ang); // needs to be updated
  Serial.print("\t");

  Serial.print("\tPOT RAW: ");
  Serial.print(potRaw);
  Serial.print("\tDEG: ");
  Serial.print(potAngle, 2); // needs to be updated
  Serial.print("\t");

  Serial.println();

  // Update LCD slowly
  if (millis() - lastLCDUpdate >= LCD_UPDATE_INTERVAL)
  {
    updateLCD(quadTicksCopy, encoder_inc_ang, amtRaw, amtAngle, potRaw, potAngle);
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

void updateLCD(long incRaw, float incDeg,
               uint16_t absRaw, float absDeg,
               int potRaw, float potDeg)
{
  char line[21];  // 20 chars + null terminator

  lcd.setCursor(0, 0);
  lcd.print("TYPE RAW      DEG   ");

  snprintf(line, sizeof(line), "ABS  %-7u %7.1f", absRaw, absDeg);
  lcd.setCursor(0, 1);
  lcd.print(line);

  snprintf(line, sizeof(line), "INC  %-7ld %7.1f", incRaw, incDeg);
  lcd.setCursor(0, 2);
  lcd.print(line);

  snprintf(line, sizeof(line), "POT  %-7d %7.1f", potRaw, potDeg);
  lcd.setCursor(0, 3);
  lcd.print(line);
}