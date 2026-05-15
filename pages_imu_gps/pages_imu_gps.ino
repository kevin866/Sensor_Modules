#include <SparkFun_Qwiic_Button.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SerLCD.h>

// ── Hardware ──────────────────────────────────────────────────────────────────
SerLCD lcd;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);
QwiicButton button;

// ── Constants ─────────────────────────────────────────────────────────────────
static const uint8_t UBLOX_ADDR = 0x42;

// ── State ─────────────────────────────────────────────────────────────────────
int functionIndex     = 0;
int lastFunctionIndex = -1;

// ── IMU globals ───────────────────────────────────────────────────────────────
imu::Vector<3> g_accel, g_gyro, g_mag, g_euler;
uint32_t lastImuUpdate = 0;

// ── GPS parse buffers ─────────────────────────────────────────────────────────
static char     lineBuf[200];
static uint16_t lineLen = 0;

static double lat_deg = NAN, lon_deg = NAN;
static float  alt_m  = NAN;
static int    fixq   = -1;
static int    sats   = -1;
static float  pdop   = NAN, hdop = NAN, vdop = NAN;
// ── Add these globals near your other GPS globals ─────────────────────────────
static uint32_t lastNmeaMs = 0;
static uint32_t lastGgaMs  = 0;
// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire1.begin();
  Wire1.setClock(400000);

  // IMU first — before LCD so its Wire init doesn't reset LCD
  Serial.println("Before IMU");
  if (!bno.begin()) {
    Serial.println("IMU error, freezing");
    while (1);
  }

  // LCD after IMU
  Serial.println("Before LCD");
  lcd.begin(Wire1);
  lcd.setBacklight(255, 255, 255);
  lcd.setContrast(10);
  lcd.clear();
  delay(200);
  lcd.setCursor(0, 0);
  lcd.print("Ready!");

  // Wire for button + GPS
  Wire.begin();
  Wire.setClock(400000);
  if (!button.begin(0x6F, Wire)) {
    Serial.println("Button error");
    while (1);
  }

  Serial.println("SETUP DONE");
}
void loop() {
  bool pressed = button.isPressed();
  if (pressed) {
    static uint32_t lastPress = 0;
    if (millis() - lastPress > 200) {
      lastPress = millis();
      functionIndex = (functionIndex + 1) % 5;
      Serial.print("Screen = ");
      Serial.println(functionIndex);
      lcd.clear();
      delay(50);
    }
  }

  readUbloxGPS();
  updateIMU();

  static uint32_t lastUpdate = 0;
  if (millis() - lastUpdate >= 500) {
    lastUpdate = millis();
    executeFunction(functionIndex);
  }
}
// void loop() {
//   bool pressed = button.isPressed();
//   if (pressed) {
//     static uint32_t lastPress = 0;
//     if (millis() - lastPress > 200) {
//       lastPress = millis();
//       functionIndex = (functionIndex + 1) % 5;
//       Serial.print("Screen = ");
//       Serial.println(functionIndex);
//     }
//   }

//   readUbloxGPS();
//   updateIMU();

//   static uint32_t lastUpdate = 0;
//   if (millis() - lastUpdate >= 500) {
//     lastUpdate = millis();
//     lcd.setCursor(0, 0);
//     lcd.print("Screen:");
//     lcd.print(functionIndex);
//     lcd.setCursor(0, 1);
//     lcd.print("ax:");
//     lcd.print(g_accel.x());
//   }
// }
// ── IMU update ────────────────────────────────────────────────────────────────
void updateIMU() {
  if (millis() - lastImuUpdate >= 200) {
    lastImuUpdate = millis();
    Wire1.setClock(400000);  // re-assert clock speed
    g_accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    g_gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    g_mag   = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    g_euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  }
}

// ── Loop ──────────────────────────────────────────────────────────────────────
// void loop() {
//   bool pressed = button.isPressed();
//   if (pressed) {
//     static uint32_t lastPress = 0;
//     if (millis() - lastPress > 200) {
//       lastPress = millis();
//       functionIndex = (functionIndex + 1) % 5;
//       Serial.print("Screen = ");
//       Serial.println(functionIndex);
//       // lcd.clear();
//       // delay(50);
//       lastFunctionIndex = -1;
//     }
//   }

//   readUbloxGPS();
//   updateIMU();

//   static uint32_t lastUpdate = 0;
//   if (millis() - lastUpdate >= 500) {
//     lastUpdate = millis();
//     Serial.println("Writing to LCD...");
//     lcd.setCursor(0, 0);
//     delay(10);
//     lcd.print("Screen:");
//     lcd.print(functionIndex);
//     lcd.setCursor(0, 1);
//     delay(10);
//     lcd.print("Millis:");
//     lcd.print(millis());
//     Serial.println("Done writing");
//   }
// }

// ── Helpers ───────────────────────────────────────────────────────────────────
void printTruncated(float val, int totalDigits) {
  char buffer[12];
  dtostrf(val, 0, 3, buffer);
  if (totalDigits >= (int)sizeof(buffer))
    totalDigits = sizeof(buffer) - 1;
  buffer[totalDigits] = '\0';
  lcd.print(buffer);
}

void executeFunction(int index) {
  switch (index) {
    case 0: function1(); break;
    case 1: function2(); break;
    case 2: function3(); break;
    case 3: function4(); break;
    case 4: function5(); break;
  }
}

// ── Screen 0 – Accelerometer ──────────────────────────────────────────────────
void function1() {
  lcd.setCursor(0, 0); delay(10); lcd.print("IMU accel.      ");
  lcd.setCursor(0, 1); delay(10); lcd.print("x      y      z ");
  lcd.setCursor(0, 2); delay(10);
  printTruncated(-g_accel.x(), 4);
  lcd.setCursor(7,  2); printTruncated(-g_accel.y(), 4);
  lcd.setCursor(14, 2); printTruncated(-g_accel.z(), 4);
  lcd.setCursor(0, 3); delay(10); lcd.print("m/s^2           ");
}

// ── Screen 1 – Gyroscope ──────────────────────────────────────────────────────
void function2() {
  lcd.setCursor(0, 0); delay(10); lcd.print("IMU gyro.       ");
  lcd.setCursor(0, 1); delay(10); lcd.print("x      y      z ");
  lcd.setCursor(0, 2); delay(10);
  printTruncated( g_gyro.x(), 4);
  lcd.setCursor(7,  2); printTruncated(-g_gyro.y(), 4);
  lcd.setCursor(14, 2); printTruncated( g_gyro.z(), 4);
  lcd.setCursor(0, 3); delay(10); lcd.print("deg/s           ");
}

// ── Screen 2 – Magnetometer ───────────────────────────────────────────────────
void function3() {
  lcd.setCursor(0, 0); delay(10); lcd.print("IMU magneto.    ");
  lcd.setCursor(0, 1); delay(10); lcd.print("x      y      z ");
  lcd.setCursor(0, 2); delay(10);
  printTruncated(g_mag.x(), 4);
  lcd.setCursor(7,  2); printTruncated(g_mag.y(), 4);
  lcd.setCursor(14, 2); printTruncated(g_mag.z(), 4);
  lcd.setCursor(0, 3); delay(10); lcd.print("uTesla          ");
}

// ── Screen 3 – Euler angles (RPY) ────────────────────────────────────────────
void function4() {
  lcd.setCursor(0, 0); delay(10); lcd.print("IMU RPY         ");
  lcd.setCursor(0, 1); delay(10); lcd.print("R(x)   P(y)  Y(z)");
  lcd.setCursor(0, 2); delay(10);
  printTruncated( g_euler.z(), 4);
  lcd.setCursor(7,  2); printTruncated(-g_euler.y(), 4);
  lcd.setCursor(14, 2); printTruncated( g_euler.x(), 4);
  lcd.setCursor(0, 3); delay(10); lcd.print("degree          ");
}

// ── Screen 4 – GPS ────────────────────────────────────────────────────────────
// void function5() {
//   lcd.setCursor(0, 0); delay(10);
//   if (!isnan(lat_deg)) { lcd.print("Lat:"); lcd.print(lat_deg, 6); }
//   else                 { lcd.print("Lat: ----        "); }
//   lcd.setCursor(0, 1); delay(10);
//   if (!isnan(lon_deg)) { lcd.print("Lon:"); lcd.print(lon_deg, 6); }
//   else                 { lcd.print("Lon: ----        "); }
//   lcd.setCursor(0, 2); delay(10);
//   lcd.print("Alt:");
//   if (!isnan(alt_m)) lcd.print(alt_m, 1); else lcd.print("----");
//   lcd.print(" F:"); lcd.print(fixq);
//   lcd.setCursor(0, 3); delay(10);
//   lcd.print("Sat:"); lcd.print(sats);
//   lcd.print(" HD:");
//   if (!isnan(hdop)) lcd.print(hdop, 1); else lcd.print("--");
// }
// ── Add this helper near your other helpers ───────────────────────────────────
static void lcdPrintRow(uint8_t row, const char *s) {
  lcd.setCursor(0, row);
  char out[21];
  size_t n = strnlen(s, 20);
  memcpy(out, s, n);
  for (size_t i = n; i < 20; i++) out[i] = ' ';
  out[20] = '\0';
  lcd.print(out);
}

// ── Screen 4 – GPS (replace existing function5) ───────────────────────────────
void function5() {
  uint32_t now = millis();
  bool haveNmea = (now - lastNmeaMs) < 1500;
  bool haveGga  = (now - lastGgaMs)  < 1500;

  if (!haveNmea) {
    lcdPrintRow(0, "No NMEA from u-blox");
    lcdPrintRow(1, "Check I2C 0x42");
    lcdPrintRow(2, "Wire/Qwiic bus");
    lcdPrintRow(3, "");
    return;
  }

  if (!haveGga || isnan(lat_deg) || isnan(lon_deg)) {
    lcdPrintRow(0, "NMEA OK, no GGA yet");
    lcdPrintRow(1, "Enable GGA+GSA out");
    lcdPrintRow(2, "Waiting for fix...");
    char fixChar = (fixq >= 0 && fixq <= 9) ? (char)('0' + fixq) : '-';
    char satsbuf[3];
    if (sats < 0) strncpy(satsbuf, "--", sizeof(satsbuf));
    else snprintf(satsbuf, sizeof(satsbuf), "%02d", sats);
    char r3[21]; snprintf(r3, sizeof(r3), "Fix:%c  Sats:%s", fixChar, satsbuf);
    lcdPrintRow(3, r3);
    return;
  }

  char r0[21], r1[21], r2[21], r3[21];
  snprintf(r0, sizeof(r0), "Lat:% .7f", lat_deg);
  snprintf(r1, sizeof(r1), "Lon:% .7f", lon_deg);

  char altbuf[6];
  if (isnan(alt_m)) strncpy(altbuf, " ----", sizeof(altbuf));
  else snprintf(altbuf, sizeof(altbuf), "%5.1f", (double)alt_m);

  char fixChar = (fixq >= 0 && fixq <= 9) ? (char)('0' + fixq) : '-';
  char satsbuf[3];
  if (sats < 0) strncpy(satsbuf, "--", sizeof(satsbuf));
  else snprintf(satsbuf, sizeof(satsbuf), "%02d", sats);

  snprintf(r2, sizeof(r2), "Alt:%s m F:%c S:%s", altbuf, fixChar, satsbuf);

  char pbuf[6], hbuf[6], vbuf[6];
  if (isnan(pdop)) strncpy(pbuf, " ----", sizeof(pbuf)); else snprintf(pbuf, sizeof(pbuf), "%5.2f", (double)pdop);
  if (isnan(hdop)) strncpy(hbuf, " ----", sizeof(hbuf)); else snprintf(hbuf, sizeof(hbuf), "%5.2f", (double)hdop);
  if (isnan(vdop)) strncpy(vbuf, " ----", sizeof(vbuf)); else snprintf(vbuf, sizeof(vbuf), "%5.2f", (double)vdop);
  snprintf(r3, sizeof(r3), "P%s H%s V%s", pbuf, hbuf, vbuf);

  lcdPrintRow(0, r0);
  lcdPrintRow(1, r1);
  lcdPrintRow(2, r2);
  lcdPrintRow(3, r3);
}
// ── GPS I²C read & NMEA feed (Wire) ──────────────────────────────────────────
void readUbloxGPS() {
  uint8_t  buf[128];
  uint16_t n = ubloxDDCRead(buf, sizeof(buf));

  for (uint16_t i = 0; i < n; i++) {
    char c = (char)buf[i];
    if (c == '\r') continue;

    if (c == '\n') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        handleNMEALine(lineBuf);
        lineLen = 0;
      }
    } else {
      if (lineLen + 1 < sizeof(lineBuf)) {
        lineBuf[lineLen++] = c;
      } else {
        lineLen = 0;
      }
    }
  }
}

// ── u-blox DDC (Wire) ─────────────────────────────────────────────────────────
static uint16_t ubloxDDCRead(uint8_t *dst, uint16_t dstMax) {
  Wire.requestFrom(UBLOX_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return 0;

  uint16_t avail = (uint16_t)Wire.read();
  avail |= (uint16_t)Wire.read() << 8;

  if (avail == 0 || avail == 0xFFFF) return 0;

  uint16_t toRead = min(avail, dstMax);
  uint16_t got    = 0;

  while (got < toRead) {
    uint8_t chunk = (uint8_t)min<uint16_t>(32, toRead - got);
    if (chunk == 0) break;
    Wire.requestFrom(UBLOX_ADDR, chunk);
    while (Wire.available() && got < toRead)
      dst[got++] = (uint8_t)Wire.read();
  }
  return got;
}

// ── NMEA sentence handler ─────────────────────────────────────────────────────
static void handleNMEALine(const char *s_in) {
  const char *s = strchr(s_in, '$');
  if (!s)              return;
  if (!strchr(s, '*')) return;
  lastNmeaMs = millis();

  char   tmp[200];
  size_t n = strnlen(s, sizeof(tmp) - 1);
  memcpy(tmp, s, n);
  tmp[n] = '\0';

  char *star = strchr(tmp, '*');
  if (star) *star = '\0';

  char  *tokens[40];
  int    nt = 0;
  char  *p  = tmp;
  while (p && nt < 40) {
    tokens[nt++] = p;
    char *comma  = strchr(p, ',');
    if (!comma) break;
    *comma = '\0';
    p      = comma + 1;
  }
  if (nt < 2) return;

  // GGA
  if (strcmp(tokens[0], "$GNGGA") == 0 || strcmp(tokens[0], "$GPGGA") == 0) {
    if (nt <= 9)                                              return;
    if (!looksLikeDM(tokens[2]) || !looksLikeDM(tokens[4])) return;
    if (!tokens[3][0] || !tokens[5][0])                     return;

    lat_deg = dmToDeg(tokens[2], tokens[3][0]);
    lon_deg = dmToDeg(tokens[4], tokens[5][0]);
    if (tokens[6][0]) fixq  = atoi(tokens[6]);
    if (tokens[7][0]) sats  = atoi(tokens[7]);
    if (tokens[8][0]) hdop  = atof(tokens[8]);
    if (tokens[9][0]) alt_m = atof(tokens[9]);
    lastGgaMs = millis();

    return;
  }

  // GSA
  if (strcmp(tokens[0], "$GNGSA") == 0) {
    if (nt > 2 && tokens[2][0] && tokens[2][0] != '3') return;
    float p2, h, v;
    if (extractGSA_DOPs(s_in, p2, h, v)) {
      pdop = p2; hdop = h; vdop = v;
    }
    return;
  }
}

// ── DOP extractor ─────────────────────────────────────────────────────────────
static bool extractGSA_DOPs(const char *s_in, float &outP, float &outH, float &outV) {
  const char *s    = strchr(s_in, '$');
  if (!s) return false;
  const char *star = strchr(s, '*');
  const char *end  = star ? star : (s + strlen(s));

  auto getTailFields = [&](int nFields, char *buf, size_t buflen) -> bool {
    int commas = 0;
    const char *p = end;
    while (p > s) {
      if (*(--p) == ',') {
        if (++commas == nFields) break;
      }
    }
    if (commas < nFields) return false;
    size_t len = (size_t)(end - (p + 1));
    if (len >= buflen) len = buflen - 1;
    memcpy(buf, p + 1, len);
    buf[len] = '\0';
    return true;
  };

  {
    char tail[64];
    if (getTailFields(4, tail, sizeof(tail))) {
      char *a = tail;
      char *b = strchr(a, ','); if (!b) goto try3; *b++ = '\0';
      char *c = strchr(b, ','); if (!c) goto try3; *c++ = '\0';
      char *d = strchr(c, ','); if (!d) goto try3; *d++ = '\0';
      int sysId = *d ? atoi(d) : 0;
      if (sysId >= 1 && sysId <= 9 && *a && *b && *c) {
        outP = atof(a); outH = atof(b); outV = atof(c);
        return true;
      }
    }
  }

try3:
  {
    char tail[48];
    if (!getTailFields(3, tail, sizeof(tail))) return false;
    char *a = tail;
    char *b = strchr(a, ','); if (!b) return false; *b++ = '\0';
    char *c = strchr(b, ','); if (!c) return false; *c++ = '\0';
    outP = *a ? atof(a) : NAN;
    outH = *b ? atof(b) : NAN;
    outV = *c ? atof(c) : NAN;
    return true;
  }
}

// ── NMEA coordinate helpers ───────────────────────────────────────────────────
static double dmToDeg(const char *dm, char hemi) {
  if (!dm || !*dm) return NAN;
  double v   = atof(dm);
  int    deg = (int)(v / 100.0);
  double mn  = v - (deg * 100.0);
  double out = deg + mn / 60.0;
  if (hemi == 'S' || hemi == 'W') out = -out;
  return out;
}

static bool looksLikeDM(const char *dm) {
  if (!dm || !*dm) return false;
  int nd = 0;
  for (int i = 0; dm[i] && dm[i] != '.'; i++) {
    if (dm[i] < '0' || dm[i] > '9') return false;
    nd++;
  }
  return nd >= 4;
}