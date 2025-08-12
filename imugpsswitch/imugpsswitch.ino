
#include <Wire.h>
#include <SoftwareSerial.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <SparkFun_I2C_GPS_Arduino_Library.h>
#include <TinyGPS++.h>
#include <SerLCD.h>

I2CGPS myI2CGPS; //Hook object to the library
TinyGPSPlus gps; //Declare gps object

// LCD
SerLCD lcd; // Uses default I2C address 0x72

// BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(28);

// GPS
SoftwareSerial gpsSerial(2, 3); // RX, TX

//Switch
bool lastSwitchState = HIGH; // Assuming pull-up, so default is HIGH (not pressed)


const int switchPin = 7;
void setup() {
  pinMode(switchPin, INPUT_PULLUP); // enable internal pull-up
  Serial.begin(115200);
  // gpsSerial.begin(9600);
  
  Wire.begin();
  
  // LCD
  lcd.begin(Wire); 
  lcd.setBacklight(255, 255, 255);
  lcd.setContrast(5); 
  lcd.clear();
  delay(1000);
  lcd.print("Initializing...");

  // BNO055
  if (!bno.begin()) {
    lcd.clear();
    lcd.print("IMU error!");
    while (1);
  }

  // GPS
  if (myI2CGPS.begin() == false)
  {
    Serial.println("Module failed to respond. Please check wiring.");
    while (1); //Freeze!
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
void displayIMUAndGPSData() {
  lcd.clear();
  // Read GPS data
  // while (gpsSerial.available()) {
  //   gps.encode(gpsSerial.read());
  // }
    while (myI2CGPS.available()) //available() returns the number of new bytes available from the GPS module
  {
    gps.encode(myI2CGPS.read()); //Feed the GPS parser
  }

  // Read IMU
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  // Display on LCD
  lcd.setCursor(10, 0);
  lcd.print("Sat: ");
  lcd.print(gps.satellites.value());

  lcd.setCursor(0, 0);
  if (gps.location.isValid()) {
    lcd.print("Lat:");
    lcd.setCursor(0, 1);
    lcd.print(gps.location.lat(),10);
  } else {
    lcd.print("Lat:     ");
  }


  lcd.setCursor(0, 2);
  if (gps.location.isValid()) {
    lcd.print("Lng:");
    lcd.setCursor(0, 3);
    lcd.print(gps.location.lng(),10);
  } else {
    lcd.print("Lng: ");
  }
 


  // Print headers
  Serial.println("Pitch:    Yaw:     Roll:");

  // Print Euler angles with 1 decimal place, spaced similarly
  // Serial.print("");
  Serial.print(-euler.y(), 1);     // Pitch
  Serial.print("      ");
  Serial.print(euler.x(), 1);     // Yaw
  Serial.print("     ");
  Serial.println(euler.z(), 1);   // Roll

  // Print Latitude
  if (gps.location.isValid()) {
    Serial.print("Lat: ");
    Serial.println(gps.location.lat(), 4);
  } else {
    Serial.println("Lat: ----");
  }

  // Print Longitude
  if (gps.location.isValid()) {
    Serial.print("Lng: ");
    Serial.println(gps.location.lng(), 4);
  } else {
    Serial.println("Lng: ----");
  }

  // Print Satellite count
  Serial.print("Sat: ");
  Serial.println(gps.satellites.value());

  // delay(1000);
  delay(500);
}

void displayIMURawData() {
  // Read IMU
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> mag   = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

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
  Serial.print(formatCell(-gyro.y())); Serial.print(" | ");
  Serial.print(formatCell(gyro.z())); Serial.println(" |");

  Serial.print("| M   | ");
  Serial.print(formatCell(mag.x())); Serial.print(" | ");
  Serial.print(formatCell(mag.y())); Serial.print(" | ");
  Serial.print(formatCell(mag.z())); Serial.println(" |");

  Serial.println("+-----+---------+---------+---------+");
  Serial.println();

  // Header
  lcd.setCursor(0, 0);
  // lcd.print("  X     Y     Z");
  lcd.print("  X(");
  // lcd.print(euler.x(), 1); // Roll
  printTruncated(euler.z(), 3); // Roll
  lcd.print(")");
  lcd.print("Y(");
  printTruncated(-euler.y(), 3); 
  lcd.print(")");
  lcd.print("Z(");
  printTruncated(euler.x(), 3); 
  lcd.print(")");




  // Accelerometer row
  lcd.setCursor(0, 1);
  lcd.print("A");
  lcd.setCursor(2, 1);
  printTruncated(-accel.x(), 4);
  lcd.setCursor(8, 1);
  printTruncated(-accel.y(), 4);
  lcd.setCursor(14, 1);
  printTruncated(-accel.z(), 4);

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


  // delay(100);
}


void loop() {
  
  bool currentSwitchState = digitalRead(switchPin);

  // If the switch state changed, clear the LCD
  if (currentSwitchState != lastSwitchState) {
    lcd.clear();
    lastSwitchState = currentSwitchState;
  }
  
  if (digitalRead(switchPin) == LOW) {
    displayIMURawData();
  } else {
    displayIMUAndGPSData();
  }
 
}
