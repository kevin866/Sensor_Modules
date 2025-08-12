// Pin configuration
const int hallPin = 26;       // Digital pin connected to the sensor output
const int ledPin = 13;       // Onboard LED for visualization

void setup() {
  pinMode(hallPin, INPUT);   // Set sensor pin as input
  pinMode(ledPin, OUTPUT);   // Set LED pin as output
  Serial.begin(9600);        // Start serial communication
  digitalWrite(hallPin, HIGH);

}

void loop() {
  int sensorState = digitalRead(hallPin);  // Read the sensor

  if (sensorState == LOW) {
    // Magnet detected
    digitalWrite(ledPin, HIGH);
    Serial.println("Magnet detected!");
  } else {
    // No magnet
    digitalWrite(ledPin, LOW);
    Serial.println("No magnet.");
  }

  // delay(200);  // Delay for readability
}
