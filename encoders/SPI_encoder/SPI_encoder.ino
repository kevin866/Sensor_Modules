#include <Arduino.h>
#include <SPI.h>
// datasheet: https://www.sameskydevices.com/product/resource/amt22.
// wiring:
// | AMT22 function | Cable color from sample | Arduino Mega pin |
// | -------------- | ----------------------- | ---------------: |
// | +5V            | White/Green             |               5V |
// | GND            | Green/White             |              GND |
// | SCLK           | Blue/White              |              D52 |
// | MOSI           | White/Blue              |              D51 |
// | MISO           | Orange/White            |              D50 |
// | CS             | White/Orange            |              D53 |

#define BAUDRATE 115200

#define AMT22_NOP   0x00
#define AMT22_ZERO  0x70

// Change this to 12 if your AMT22 is 12-bit
#define RESOLUTION 14

// Arduino Mega CS pin
#define CS_PIN 53

bool verifyChecksumSPI(uint16_t message)
{
  // AMT22 checksum: upper 2 bits are inverted XOR parity bits
  uint16_t checksum = 0x3;

  for (int i = 0; i < 14; i += 2)
  {
    checksum ^= (message >> i) & 0x3;
  }

  return checksum == (message >> 14);
}

uint16_t readAMT22Position()
{
  uint16_t encoderPosition = 0;

  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(3);

  // Read high byte first
  encoderPosition = SPI.transfer(AMT22_NOP) << 8;

  delayMicroseconds(3);

  // Read low byte
  encoderPosition |= SPI.transfer(AMT22_NOP);

  delayMicroseconds(3);
  digitalWrite(CS_PIN, HIGH);

  if (!verifyChecksumSPI(encoderPosition))
  {
    return 0xFFFF;  // error value
  }

  // Remove top 2 checksum bits
  encoderPosition &= 0x3FFF;

  // If 12-bit encoder, lower 2 bits are unused
  if (RESOLUTION == 12)
  {
    encoderPosition = encoderPosition >> 2;
  }

  return encoderPosition;
}

void setup()
{
  Serial.begin(BAUDRATE);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  // Important on Arduino Mega: keep hardware SS pin as output
  pinMode(53, OUTPUT);

  SPI.begin();

  // AMT22 supports SPI mode 0.
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));

  delay(250);  // give encoder time to start
}

void loop()
{
  uint16_t position = readAMT22Position();

  if (position == 0xFFFF)
  {
    Serial.println("Encoder position error");
  }
  else
  {
    float maxCount = (RESOLUTION == 14) ? 16384.0 : 4096.0;
    float angleDeg = position * 360.0 / maxCount;

    Serial.print("Raw position: ");
    Serial.print(position);
    Serial.print("\tAngle: ");
    Serial.println(angleDeg, 2);
  }

  delay(100);
}