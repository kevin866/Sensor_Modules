#include <Wire.h>
#include <BH1750.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <SerLCD.h>
#define SEALEVELPRESSURE_HPA (1013.25)

// Sensors
BH1750 lightSensor;
Adafruit_BME680 bme; // Uses I2C
SerLCD lcd; // Qwiic display

// Collision switch pin
const int switchPin = 2;
bool collisionDetected = false;
const int hallPin = 26;       // Digital pin connected to the sensor output
int hall = 0;
float alt_off = 0;
bool first = true;
void setup() {
  Wire.begin(); // Join I2C bus
  Serial.begin(115200);

  // Setup collision pin
  pinMode(switchPin, INPUT_PULLUP); // NO config, internal pull-up
    pinMode(hallPin, INPUT_PULLUP);   // Set sensor pin as input


  // Initialize Light Sensor
  if (!lightSensor.begin()) {
    Serial.println("BH1750 not found");
    while (1);
  }

  // Initialize BME680
  if (!bme.begin()) {
    Serial.println("BME680 not found");
    while (1);
  }

  // Configure BME680 settings
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setGasHeater(320, 150); // 320*C for 150ms

  // Initialize LCD
  lcd.begin(Wire); // Use default I2C address 0x72
  lcd.setBacklight(255, 255, 255); // Full brightness
  lcd.clear();
  lcd.setContrast(5);
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  lcd.clear();
  delay(2000);
}

void loop() {
  // Read light sensor
  float lux = lightSensor.readLightLevel();

  // Read BME680
  if (!bme.performReading()) {
    Serial.println("BME680 failed to perform reading");
    return;
  }

  float temp = bme.temperature;
  float hum = bme.humidity;
  float pres = bme.pressure / 100.0; // Pa to hPa
  float gas = bme.gas_resistance / 1000.0; // Ohms to KOhms
  float alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
  if (first){
      alt_off = alt;
      first = false;
  }

  // Read collision switch
  int switchState = digitalRead(switchPin);
  collisionDetected = (switchState == LOW); // Pressed = collision

  int sensorState = digitalRead(hallPin);  // Read the sensor
  if (sensorState == LOW) {
    Serial.println("Magnet detected!");
  } else {
    Serial.println("No magnet.");
  }

  // // Debug output
  // Serial.print("Lux: "); Serial.print(lux); Serial.print(" lx, ");
  // Serial.print("Temp: "); Serial.print(temp); Serial.print(" C, ");
  // Serial.print("Hum: "); Serial.print(hum); Serial.print(" %, ");
  // Serial.print("Pres: "); Serial.print(pres); Serial.print(" hPa, ");
  // Serial.print("Gas: "); Serial.print(gas); Serial.print(" KOhms, ");
  Serial.print("Alt: "); Serial.print(alt-alt_off); Serial.print(" m, ");
  // Serial.print("Collision: "); Serial.println(collisionDetected ? "YES" : "NO");

  // LCD Display
  // lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SW: ");
  if (collisionDetected) {
    lcd.print("1");
  } else {
    lcd.print("0");
  }
  lcd.print("   Lux: "); lcd.print(lux, 1); lcd.print("lx");  
  lcd.setCursor(0, 1);
  lcd.print("H: ");lcd.print(hum, 0);lcd.print("%"); 
  lcd.print("  T: "); lcd.print(temp, 1); lcd.print("C");  


  lcd.setCursor(0, 2);
  // lcd.print("Gas:"); lcd.print(gas, 1); lcd.print(" kOhms");
  lcd.print("P: ");lcd.print(pres, 1); lcd.print("hPa");


  lcd.setCursor(0, 3);
  lcd.print("Alt: ");lcd.print(alt-alt_off); lcd.print("m");
  lcd.setCursor(13,3);
  lcd.print("Hall ");lcd.print(abs(sensorState-1), 1); 
}
