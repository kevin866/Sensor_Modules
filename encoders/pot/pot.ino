#include <Arduino.h>

#define POT_PIN A0
// website: https://www.digikey.com/en/products/detail/bourns-inc/3590S-2-201L/2534354?gclsrc=aw.ds&gad_source=1&gad_campaignid=20243136172&gbraid=0AAAAADrbLljzsCKHCttNxlRUGpN-4hTKM&gclid=Cj0KCQjwiJvQBhCYARIsAMjts3JYw1nXnSPnaX_IrtIpKePZCk7E9fWGcxRT7Eq39d1zEXlScpx7KdYaAljuEALw_wcB
void setup()
{
  Serial.begin(115200);
}

void loop()
{
  int raw = analogRead(POT_PIN);

  float voltage = raw * (5.0 / 1023.0);
  float turns = raw * (10.0 / 1023.0);
  float angleDeg = turns * 360.0;

  Serial.print("Raw: ");
  Serial.print(raw);

  Serial.print("\tVoltage: ");
  Serial.print(voltage, 3);

  Serial.print("\tTurns: ");
  Serial.print(turns, 3);

  Serial.print("\tAngle: ");
  Serial.println(angleDeg, 1);

  delay(100);
}