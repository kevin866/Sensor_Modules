#include <Wire.h>
#include <SerLCD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// LCD
SerLCD lcd;

// BNO055 IMU
Adafruit_BNO055 bno = Adafruit_BNO055(28);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // LCD setup
  lcd.begin(Wire);
  lcd.setBacklight(255, 255, 255);
  lcd.setContrast(5);
  lcd.clear();
  delay(1000);
  lcd.print("Initializing...");

  // IMU setup
  if (!bno.begin()) {
    lcd.clear();
    lcd.print("IMU error!");
    while (1);
  }

  delay(1000);
  lcd.clear();
}
String formatCell(float val, int width = 7, int precision = 2) {
  String s = String(val, precision);
  while (s.length() < width) {
    s = " " + s;
  }
  return s;
}
void printTruncated(float val, int totalDigits) {
  char buffer[10];
  // Convert float to string with 3 decimals (max)
  dtostrf(val, 0, 3, buffer);
  
  // Truncate the string to totalDigits characters
  buffer[totalDigits] = '\0';

  lcd.print(buffer);
}


void loop() {
  // Get sensor data
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> mag   = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  // Print table header
  Serial.println("+-----+---------+---------+---------+");
  Serial.println("|     |    X    |    Y    |    Z    |");
  Serial.println("+-----+---------+---------+---------+");

  Serial.print("| A   | ");
  Serial.print(formatCell(accel.x())); Serial.print(" | ");
  Serial.print(formatCell(accel.y())); Serial.print(" | ");
  Serial.print(formatCell(accel.z())); Serial.println(" |");

  Serial.print("| G   | ");
  Serial.print(formatCell(gyro.x())); Serial.print(" | ");
  Serial.print(formatCell(gyro.y())); Serial.print(" | ");
  Serial.print(formatCell(gyro.z())); Serial.println(" |");

  Serial.print("| M   | ");
  Serial.print(formatCell(mag.x())); Serial.print(" | ");
  Serial.print(formatCell(mag.y())); Serial.print(" | ");
  Serial.print(formatCell(mag.z())); Serial.println(" |");

  Serial.println("+-----+---------+---------+---------+");
  Serial.println();

  // Header
  lcd.setCursor(0, 0);
  lcd.print("  X     Y     Z");

  // Accelerometer row
  lcd.setCursor(0, 1);
  lcd.print("A");
  lcd.setCursor(2, 1);
  printTruncated(accel.x(), 4);
  lcd.setCursor(8, 1);
  printTruncated(accel.y(), 4);
  lcd.setCursor(14, 1);
  printTruncated(accel.z(), 4);

  // Gyroscope row
  lcd.setCursor(0, 2);
  lcd.print("G");
  lcd.setCursor(2, 2);
  printTruncated(gyro.x(), 4);
  lcd.setCursor(8, 2);
  printTruncated(gyro.y(), 4);
  lcd.setCursor(14, 2);
  printTruncated(gyro.z(), 4);

  // Magnetometer row
  lcd.setCursor(0, 3);
  lcd.print("M");
  lcd.setCursor(2, 3);
  printTruncated(mag.x(), 4);
  lcd.setCursor(8, 3);
  printTruncated(mag.y(), 4);
  lcd.setCursor(14, 3);
  printTruncated(mag.z(), 4);


  delay(100);
}
