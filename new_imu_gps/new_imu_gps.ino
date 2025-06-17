
#include <SerLCD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
// #include <SparkFun_SerLCD.h>
#include <SoftwareSerial.h>
#include <SparkFun_I2C_GPS_Arduino_Library.h> //Use Library Manager or download here: https://github.com/sparkfun/SparkFun_I2C_GPS_Arduino_Library
I2CGPS myI2CGPS; //Hook object to the library
#include <TinyGPS++.h> //From: https://github.com/mikalhart/TinyGPSPlus
TinyGPSPlus gps; //Declare gps object

// LCD
SerLCD lcd; // Uses default I2C address 0x72

// BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(28);

// GPS
// SoftwareSerial gpsSerial(2, 3); // RX, TX


void setup() {
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

void loop() {
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
  lcd.setCursor(0, 0);
  lcd.print("Rol:   Pitch:  Yaw:");

  lcd.setCursor(0, 1); 
  lcd.print(euler.z(), 1); // Pitch
  lcd.setCursor(7, 1); 
  lcd.print(euler.y(), 1); // Heading/Yaw
  lcd.setCursor(15, 1); 
  lcd.print(euler.x(), 1); // Roll

  lcd.setCursor(0, 2);
  if (gps.location.isValid()) {
    lcd.print("Lat:");
    lcd.print(gps.location.lat(), 4);
  } else {
    lcd.print("Lat: ----");
  }
  lcd.setCursor(15, 2);
  lcd.print("Sat");

  lcd.setCursor(0, 3);
  if (gps.location.isValid()) {
    lcd.print("Lng:");
    lcd.print(gps.location.lng(), 4);
  } else {
    lcd.print("Lng: ----");
  }
  lcd.setCursor(15, 3);
  lcd.print(gps.satellites.value());


// Print headers
Serial.println("Pitch:    Yaw:     Roll:");

// Print Euler angles with 1 decimal place, spaced similarly
// Serial.print("");
Serial.print(euler.y(), 1);     // Pitch
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

  delay(1000);
}
