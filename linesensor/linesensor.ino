#include "Wire.h"
#include "sensorbar.h"
#include "Arduino_LED_Matrix.h"

SensorBar mySensorBar(0x3E); // Default I2C address
ArduinoLEDMatrix matrix;
int analogValue;

void setup()
{
  Serial.begin(9600);
  Serial.println("Program started.\n");

  matrix.begin(); // Initialize LED matrix

  mySensorBar.setBarStrobe();
  mySensorBar.clearInvertBits();

  if (!mySensorBar.begin()) {
    Serial.println("sx1509 IC communication FAILED!");
    while (1);
  } else {
    Serial.println("sx1509 IC communication OK");
  }
}

void loop()
{
  uint8_t rawValue = mySensorBar.getRaw();
  // Print binary value
  Serial.print("Bin value of input: ");
  for (int i = 7; i >= 0; i--) {
    Serial.print((rawValue >> i) & 0x01);
  }
  Serial.println("b");

  // Print position and density
  Serial.print("Position (-127 to 127): ");
  Serial.println(mySensorBar.getPosition());
  Serial.print("Density (0 to 8): ");
  Serial.println(mySensorBar.getDensity());
  Serial.println("");

  // Create a 2D display buffer: 8 rows x 12 columns
  uint8_t display[8][12] = {0}; // all off

  // Light up LEDs on the bottom row (row 7)
  for (int col = 0; col < 8; col++) {
    if ((rawValue >> (7 - col)) & 0x01) {
      display[7][col + 2] = 1; // shift +2 to center in 12 columns
    }
  }

  // Send to matrix
  matrix.renderBitmap(display, 12, 8);

  delay(666);
}
