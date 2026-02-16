#include <Arduino.h>
#include <Adafruit_GPS.h>
#include <string.h>

/*
  GPS Protocol Scope (current + planned)
  --------------------------------------
  Current firmware in this file is for Adafruit_GPS + NMEA modules that support
  PMTK commands (for example Adafruit Ultimate GPS / MTK33xx style modules).
  The setup sequence below uses PMTK_* commands, so this build is intended for
  that protocol family.

  Planned support:
  - Add a u-blox-specific implementation for modules that use UBX protocol.
  - At flash/build time, let the user select which GPS protocol target to use.
  - Optionally provide a combined firmware variant that can handle both
    protocol families with detection/config selection at startup.
*/

// ESP32 RX pin (connect to GPS TX)
#define GPS_RX_PIN  7
// ESP32 TX pin (connect to GPS RX)
#define GPS_TX_PIN  8

#define GPS_BAUD_DEFAULT 9600

// Use HardwareSerial (Serial1) for the GPS
Adafruit_GPS GPS(&Serial1);
unsigned long lastHeartbeatMs = 0;
uint32_t activeGpsBaud = GPS_BAUD_DEFAULT;

struct GsvTelemetry {
  int satsInView = -1;
  float snrAvg = 0.0f;
  int snrMax = 0;
  int snrSamples = 0;
  int snrMaxEver = 0;
  int snrBars[16] = {0};
  int snrBarCount = 0;
  unsigned long lastUpdateMs = 0;
};

GsvTelemetry gsv;

struct PreFixAssessment {
  int score = 0;            // 0..100
  const char *stage = "No signal";
};

static int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static PreFixAssessment computePreFixAssessment() {
  if (GPS.fix) {
    PreFixAssessment fixed;
    fixed.score = 100;
    fixed.stage = "Fixed";
    return fixed;
  }

  int score = 0;
  int satsInView = gsv.satsInView > 0 ? gsv.satsInView : 0;
  int satsUsed = GPS.satellites > 0 ? GPS.satellites : 0;

  score += clampInt(satsInView * 4, 0, 28);
  score += clampInt(satsUsed * 8, 0, 32);

  if (gsv.snrAvg >= 30.0f) score += 20;
  else if (gsv.snrAvg >= 20.0f) score += 14;
  else if (gsv.snrAvg >= 10.0f) score += 8;
  else if (gsv.snrAvg > 0.0f) score += 3;

  if (GPS.HDOP > 0.0f && GPS.HDOP <= 1.5f) score += 20;
  else if (GPS.HDOP > 0.0f && GPS.HDOP <= 2.5f) score += 14;
  else if (GPS.HDOP > 0.0f && GPS.HDOP <= 5.0f) score += 8;
  else if (GPS.HDOP > 0.0f && GPS.HDOP <= 10.0f) score += 3;

  if (gsv.lastUpdateMs > 0 && (millis() - gsv.lastUpdateMs) < 4000UL) score += 5;
  score = clampInt(score, 0, 95);  // reserve 100 for true fix

  const char *stage = "No signal";
  if (score >= 80) stage = "Very close";
  else if (score >= 60) stage = "Close to fix";
  else if (score >= 40) stage = "Tracking";
  else if (score >= 20) stage = "Weak acquisition";

  PreFixAssessment out;
  out.score = score;
  out.stage = stage;
  return out;
}

static bool probeGpsBaud(uint32_t baud, uint32_t timeoutMs) {
  Serial1.end();
  delay(30);
  Serial1.begin(baud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  GPS.begin(baud);

  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    char c = GPS.read();
    if (c == '$') {
      return true;
    }
    delay(1);
  }
  return false;
}

static void parseGsvSentence(const char *nmea) {
  if (!nmea || strlen(nmea) < 7) return;
  if (!(nmea[0] == '$' && nmea[3] == 'G' && nmea[4] == 'S' && nmea[5] == 'V')) return;

  char buf[128];
  strncpy(buf, nmea, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  char *star = strchr(buf, '*');
  if (star) *star = '\0';

  // GSV fields:
  // 1 total msgs, 2 msg num, 3 sats in view, then groups of 4 where field 4 is SNR.
  int field = 0;
  int snrCount = 0;
  int snrSum = 0;
  int snrMax = 0;
  int bars[16] = {0};
  int barCount = 0;
  char *save = nullptr;
  char *tok = strtok_r(buf, ",", &save);
  while (tok) {
    if (field == 3) {
      gsv.satsInView = atoi(tok);
    }
    if (field >= 7 && ((field - 7) % 4 == 0) && tok[0] != '\0') {
      int snr = atoi(tok);
      if (snr > 0) {
        snrCount++;
        snrSum += snr;
        if (snr > snrMax) snrMax = snr;
        if (barCount < 16) {
          bars[barCount++] = snr;
        }
      }
    }
    field++;
    tok = strtok_r(nullptr, ",", &save);
  }

  if (snrCount > 0) {
    gsv.snrSamples = snrCount;
    gsv.snrAvg = (float)snrSum / (float)snrCount;
    gsv.snrMax = snrMax;
    if (snrMax > gsv.snrMaxEver) gsv.snrMaxEver = snrMax;
  } else {
    gsv.snrSamples = 0;
    gsv.snrAvg = 0.0f;
    gsv.snrMax = 0;
  }
  gsv.snrBarCount = barCount;
  for (int i = 0; i < barCount; i++) {
    gsv.snrBars[i] = bars[i];
  }

  gsv.lastUpdateMs = millis();
}

void setup() {
  // Debug serial at 115200 for Serial Monitor
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    ; // wait for serial port on boards with native USB
  }
  Serial.println(F("Adafruit GPS test"));

  // Auto-detect common UART rates so we can recover from unknown module settings.
  if (probeGpsBaud(9600, 2500)) {
    activeGpsBaud = 9600;
  } else if (probeGpsBaud(115200, 2500)) {
    activeGpsBaud = 115200;
  } else {
    activeGpsBaud = GPS_BAUD_DEFAULT;
    Serial.println(F("Warning: no NMEA detected during probe; defaulting to 9600"));
  }

  Serial.println(F("Keeping detected GPS baud (no forced baud switch)"));

  Serial1.end();
  delay(30);
  Serial1.begin(activeGpsBaud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  GPS.begin(activeGpsBaud);
  Serial.print(F("GPS UART active baud: "));
  Serial.println(activeGpsBaud);

  // Include richer quality data (GSA/GSV) so we can inspect reception quality.
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
}

void loop() {
  if (millis() - lastHeartbeatMs >= 2000UL) {
    lastHeartbeatMs = millis();
    Serial.print(F("alive: waiting for GPS data... (gps_baud="));
    Serial.print(activeGpsBaud);
    Serial.println(F(")"));
  }
  
  GPS.read();
  if (GPS.newNMEAreceived()) {
    char *last = GPS.lastNMEA();
    parseGsvSentence(last);

    if (!GPS.parse(GPS.lastNMEA())) {
      return;
    }
    Serial.println(F("---"));
    Serial.print(F("Fix: "));
    Serial.println(GPS.fix ? F("yes") : F("no"));
    Serial.print(F("Satellites: "));
    Serial.println((int)GPS.satellites);
    Serial.print(F("Fix Quality: "));
    Serial.println((int)GPS.fixquality);
    Serial.print(F("Fix 3D: "));
    Serial.println((int)GPS.fixquality_3d);
    Serial.print(F("DOP: P="));
    Serial.print(GPS.PDOP, 1);
    Serial.print(F(" H="));
    Serial.print(GPS.HDOP, 1);
    Serial.print(F(" V="));
    Serial.println(GPS.VDOP, 1);
    Serial.print(F("Sats In View: "));
    if (gsv.satsInView >= 0) Serial.println(gsv.satsInView);
    else Serial.println(F("n/a"));
    Serial.print(F("SNR: avg="));
    Serial.print(gsv.snrAvg, 1);
    Serial.print(F(" max="));
    Serial.print(gsv.snrMax);
    Serial.print(F(" samples="));
    Serial.print(gsv.snrSamples);
    Serial.print(F(" max_ever="));
    Serial.println(gsv.snrMaxEver);
    Serial.print(F("SNR Bars: "));
    if (gsv.snrBarCount == 0) {
      Serial.println(F("none"));
    } else {
      for (int i = 0; i < gsv.snrBarCount; i++) {
        if (i) Serial.print(',');
        Serial.print(gsv.snrBars[i]);
      }
      Serial.println();
    }
    const PreFixAssessment assess = computePreFixAssessment();
    Serial.print(F("Pre-Fix Score: "));
    Serial.print(assess.score);
    Serial.println(F("/100"));
    Serial.print(F("Pre-Fix Stage: "));
    Serial.println(assess.stage);
    if (GPS.fix) {
      Serial.print(F("Location: "));
      Serial.print(GPS.latitudeDegrees, 6);
      Serial.print(F(", "));
      Serial.println(GPS.longitudeDegrees, 6);
      Serial.print(F("Altitude: "));
      Serial.print(GPS.altitude);
      Serial.println(F(" m"));
      Serial.print(F("Speed: "));
      Serial.println(GPS.speed);
    }
    Serial.print(F("Time: "));
    Serial.print(GPS.hour, DEC);
    Serial.print(':');
    Serial.print(GPS.minute, DEC);
    Serial.print(':');
    Serial.print(GPS.seconds, DEC);
    Serial.print(F(" Date: "));
    Serial.print(GPS.day, DEC);
    Serial.print('/');
    Serial.print(GPS.month, DEC);
    Serial.print("/20");
    Serial.println(GPS.year, DEC);
  }
}
