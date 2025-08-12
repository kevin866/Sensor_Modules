#include <Wire.h>
#include <SerLCD.h>
#include <sensorbar.h>

SerLCD lcd;
SensorBar mySensorBar(0x3E); // Default I2C address

int analogValue;

void setup() {
  Wire1.begin();              // For LCD on Wire1
  Wire.begin();               // For SensorBar on Wire (default)
  
  lcd.begin(Wire1);
  lcd.setBacklight(255,255,255);
  lcd.clear();
  lcd.print("Starting...");
  delay(1000);

  Serial.begin(9600);

  mySensorBar.setBarStrobe();
  mySensorBar.clearInvertBits();

  if (!mySensorBar.begin()) {
    Serial.println("SensorBar FAIL!");
    while (1);
  } else {
    Serial.println("SensorBar OK");
  }
  lcd.clear();
}

void loop() {
  // Read analog input
  analogValue = analogRead(A0);
  int clampedAnalog = 700 - constrain(analogValue, 100, 600);
  int distance = (clampedAnalog-350)*160/500;
  int analogPos = map(clampedAnalog, 100, 600, 4, 14);
  // int analogPos = map(distance, -80, 80, 4, 14);

  // Serial.println(distance);

  // Read SensorBar position
  int sensorPos = mySensorBar.getPosition(); // -127 to 127
  int sensorMappedPos = map(sensorPos, -127, 127, 6, 12);
  Serial.println(analogPos);

  

  // Top row: show SensorBar position
  lcd.setCursor(6, 0);
  for (int i = 6; i < 13; i++) {
    if (i == sensorMappedPos) {
      lcd.print("^");
    } else {
      lcd.print(" ");
    }
  }


  lcd.setCursor(0, 1);
  if (distance>0){
    lcd.print("+");
    if (distance<10){
      lcd.print(distance);
      lcd.print(" ");
    }else{
      lcd.print(distance);
    }
  }else if(distance == 0){
    lcd.print(" ");
    lcd.print(distance);
  }
  else{
      if (distance>-10){
        lcd.print(distance);
        lcd.print(" ");
      }else{
        lcd.print(distance);
      }
  }
  // Bottom row: show analog position
  lcd.setCursor(4, 1);
  for (int i = 4; i < 15; i++) {
    if (i == analogPos)
      lcd.print("^");
    else
      lcd.print(" ");
  }

  lcd.setCursor(13, 0);  // (column, row) -> (0-based)
  lcd.write(255);          // Write full block character
  lcd.setCursor(5, 0);  // (column, row) -> (0-based)
  lcd.write(255);          // Write full block character
  lcd.setCursor(15, 1);  // (column, row) -> (0-based)
  lcd.write(255);          // Write full block character
  lcd.setCursor(3, 1);  // (column, row) -> (0-based)
  lcd.write(255);          // Write full block character

  

  // delay(300);
}
