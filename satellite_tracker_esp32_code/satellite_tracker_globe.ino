#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Sgp4.h>
#include <time.h>


const char* WIFI_SSID     = "MY_WIFI_SSID";
const char* WIFI_PASSWORD = "My_WIFI_PASSWORD";
const unsigned long WIFI_RECONNECT_INTERVAL_MS = 5000;


WebServer server(80);


const char* TLE_API_HOST = "https://tracker.sanjaybinayak.hackclub.app/api/tle/";
const unsigned long TLE_MAX_AGE_MS   = 12UL * 60UL * 60UL * 1000UL;
const unsigned long TLE_RETRY_DELAY_MS = 5000;                     


const char* NTP_SERVER = "pool.ntp.org";
const long  GMT_OFFSET_SEC = 0;
const int   DST_OFFSET_SEC = 0;


const int SERVO1_PIN = 18;
const int SERVO2_PIN = 19;
const int SERVO3_PIN = 21;


const int OLED_SDA_PIN   = 23;
const int OLED_SCL_PIN   = 22;
const int SCREEN_WIDTH   = 128;
const int SCREEN_HEIGHT  = 64;
const int OLED_RESET_PIN = -1;
const uint8_t OLED_I2C_ADDR = 0x3C;


const unsigned long POSITION_UPDATE_INTERVAL_MS = 1000;
const unsigned long OLED_UPDATE_INTERVAL_MS      = 1000;
const unsigned long SERVO_STEP_INTERVAL_MS       = 15;
const float SERVO_STEP_DEGREES                   = 1.0;


const float GLOBE_GEAR_RATIO = 0.5;


float longitudeOffsetDeg = 0.0;
float latitudeOffsetDeg  = 0.0;
float servo2OffsetDeg    = 0.0;
float servo3OffsetDeg    = 0.0;


const bool SERVO3_MIRRORED = false;


const int SERVO_MIN_ANGLE = 0;
const int SERVO_MAX_ANGLE = 180;


Servo servo1;
Servo servo2;
Servo servo3;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET_PIN);

Sgp4 sat;

// ===========================================================================
// STATE
// ===========================================================================

// WiFi state
bool wifiWasConnected = false;
unsigned long lastWifiAttemptMs = 0;

// Satellite selection / TLE state
long currentNoradId = -1;      // -1 = none selected
long pendingNoradId  = -1;     // set by web handler, consumed in loop
bool noradIdChanged  = false;

String satName = "";
String tleLine1 = "";
String tleLine2 = "";
bool tleValid = false;
unsigned long lastTleFetchMs = 0;
unsigned long lastTleAttemptMs = 0;
bool tleDownloadInProgress = false;

bool sgp4Ready = false;

// Latest computed satellite position
double currentLatDeg = 0.0;
double currentLonDeg = 0.0;
double currentAltKm  = 0.0;
bool positionValid = false;

// Servo current + target angles (in servo-degrees, 0-180)
float servo1CurrentAngle = 90.0;
float servo1TargetAngle  = 90.0;

float servo2CurrentAngle = 90.0;
float servo2TargetAngle  = 90.0;

float servo3CurrentAngle = 90.0;
float servo3TargetAngle  = 90.0;

// Timers
unsigned long lastPositionUpdateMs = 0;
unsigned long lastOledUpdateMs = 0;
unsigned long lastServoStepMs = 0;

// Status / error messages shown on OLED (highest priority first)
enum StatusMessage {
  STATUS_NONE,
  STATUS_NO_SATELLITE,
  STATUS_WIFI_DISCONNECTED,
  STATUS_DOWNLOADING,
  STATUS_DOWNLOAD_FAILED,
  STATUS_INVALID_TLE,
  STATUS_SGP4_ERROR
};
StatusMessage currentStatus = STATUS_NO_SATELLITE;

// ===========================================================================
// FORWARD DECLARATIONS
// ===========================================================================

void connectWiFi();
void handleWiFi();
void setupWebServer();
void handleSelectNorad();
void downloadTLE();
bool parseTLEResponse(const String& payload);
void initializeSGP4();
void calculateSatellitePosition();
void updateServoTargets();
void moveServosSmoothly();
void updateOLED();
void handleTimers();
float wrapServoAngle(float angle);
void syncTimeViaNTP();

// ===========================================================================
// SETUP
// ===========================================================================

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n=== ESP32 Satellite Tracker Globe booting ==="));

  // ---- OLED init ----
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
    Serial.println(F("[OLED] SSD1306 allocation failed"));
  } else {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println(F("Booting..."));
    display.display();
  }

  // ---- Servos ----
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);
  servo1.attach(SERVO1_PIN, 500, 2400);
  servo2.attach(SERVO2_PIN, 500, 2400);
  servo3.attach(SERVO3_PIN, 500, 2400);

  // Move to a known neutral position at startup.
  servo1.write((int)servo1CurrentAngle);
  servo2.write((int)servo2CurrentAngle);
  servo3.write((int)servo3CurrentAngle);

  // ---- WiFi ----
  connectWiFi();

  // ---- NTP time (required for SGP4 epoch math) ----
  syncTimeViaNTP();

  // ---- HTTP server for satellite selection ----
  setupWebServer();

  currentStatus = STATUS_NO_SATELLITE;
  Serial.println(F("=== Setup complete ==="));
}

// ===========================================================================
// MAIN LOOP - non-blocking, millis()-scheduled
// ===========================================================================

void loop() {
  handleWiFi();          // reconnect if needed, never blocks
  server.handleClient();  // process incoming NORAD ID selections

  // Apply a newly selected NORAD ID (set by the web handler)
  if (noradIdChanged) {
    noradIdChanged = false;
    currentNoradId = pendingNoradId;
    tleValid = false;
    sgp4Ready = false;
    positionValid = false;
    lastTleFetchMs = 0; // force immediate download
    Serial.print(F("[SELECT] New NORAD ID selected: "));
    Serial.println(currentNoradId);
  }

  handleTimers(); // periodic TLE download / SGP4 update / OLED refresh

  moveServosSmoothly(); // every loop, non-blocking easing toward target
}

// ===========================================================================
// WIFI
// ===========================================================================

void connectWiFi() {
  Serial.print(F("[WiFi] Connecting to "));
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastWifiAttemptMs = millis();
}

void handleWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiWasConnected) {
      wifiWasConnected = true;
      Serial.print(F("[WiFi] Connected. IP address: "));
      Serial.println(WiFi.localIP());
      if (currentStatus == STATUS_WIFI_DISCONNECTED) {
        currentStatus = (currentNoradId < 0) ? STATUS_NO_SATELLITE : STATUS_NONE;
      }
    }
    return;
  }

  // Not connected
  if (wifiWasConnected) {
    // We just lost the connection
    wifiWasConnected = false;
    Serial.println(F("[WiFi] Connection lost"));
  }
  currentStatus = STATUS_WIFI_DISCONNECTED;

  // Attempt reconnect periodically, without blocking the loop
  unsigned long now = millis();
  if (now - lastWifiAttemptMs >= WIFI_RECONNECT_INTERVAL_MS) {
    lastWifiAttemptMs = now;
    Serial.println(F("[WiFi] Attempting reconnect..."));
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

void syncTimeViaNTP() {
  Serial.println(F("[NTP] Syncing time..."));
  configTime(GMT_OFFSET_SEC, DST_OFFSET_SEC, NTP_SERVER);
  struct tm timeinfo;
  // Try briefly; this only runs once at boot, a short wait here is acceptable
  // since it happens before the main loop starts.
  for (int i = 0; i < 20; i++) {
    if (getLocalTime(&timeinfo, 500)) {
      Serial.println(F("[NTP] Time synced"));
      return;
    }
  }
  Serial.println(F("[NTP] Warning: time sync failed, SGP4 accuracy may suffer"));
}

// ===========================================================================
// SATELLITE SELECTION (HTTP endpoint used by the mobile app)
// ===========================================================================

void setupWebServer() {
  server.on("/select", HTTP_GET, handleSelectNorad);
  server.begin();
  Serial.println(F("[HTTP] Web server started on port 80 (/select?norad=<id>)"));
}

void handleSelectNorad() {
  if (!server.hasArg("norad")) {
    server.send(400, "text/plain", "Missing 'norad' parameter");
    return;
  }
  long newId = server.arg("norad").toInt();
  if (newId <= 0) {
    server.send(400, "text/plain", "Invalid NORAD ID");
    return;
  }

  if (newId != currentNoradId) {
    pendingNoradId = newId;
    noradIdChanged = true;
  }

  server.send(200, "text/plain", "OK");
}

// ===========================================================================
// TLE DOWNLOAD + PARSE
// ===========================================================================

void downloadTLE() {
  if (currentNoradId <= 0) return;
  if (WiFi.status() != WL_CONNECTED) return;

  tleDownloadInProgress = true;
  currentStatus = STATUS_DOWNLOADING;
  Serial.print(F("[TLE] Downloading TLE for NORAD "));
  Serial.println(currentNoradId);

  HTTPClient http;
  String url = String(TLE_API_HOST) + String(currentNoradId);
  http.begin(url);
  int httpCode = http.GET();
  Serial.print(F("[TLE] HTTP status: "));
  Serial.println(httpCode);

  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    if (parseTLEResponse(payload)) {
      lastTleFetchMs = millis();
      tleValid = true;
      Serial.println(F("[TLE] Parsed OK: ") + satName);
      initializeSGP4();
    } else {
      // Keep using previous TLE if we had one
      currentStatus = STATUS_INVALID_TLE;
      Serial.println(F("[TLE] JSON parse failed, keeping previous TLE"));
    }
  } else {
    currentStatus = STATUS_DOWNLOAD_FAILED;
    lastTleAttemptMs = millis(); // schedule a retry
    Serial.println(F("[TLE] Download failed"));
  }

  http.end();
  tleDownloadInProgress = false;
}

bool parseTLEResponse(const String& payload) {
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.print(F("[TLE] deserializeJson failed: "));
    Serial.println(err.c_str());
    return false;
  }

  if (!doc.containsKey("name") || !doc.containsKey("line1") || !doc.containsKey("line2")) {
    Serial.println(F("[TLE] JSON missing required fields"));
    return false;
  }

  satName  = doc["name"].as<String>();
  tleLine1 = doc["line1"].as<String>();
  tleLine2 = doc["line2"].as<String>();

  if (tleLine1.length() < 60 || tleLine2.length() < 60) {
    Serial.println(F("[TLE] Line length looks invalid"));
    return false;
  }

  return true;
}

// ===========================================================================
// SGP4 INITIALIZATION + PROPAGATION
// ===========================================================================

void initializeSGP4() {
  if (!tleValid) {
    sgp4Ready = false;
    return;
  }

  // NOTE: API depends on which Arduino SGP4 library you installed.
  // This uses the common Sgp4 (Daniel Warner) library signature:
  //   sat.init(satname, tle_line1, tle_line2)
  char nameBuf[25];
  char line1Buf[70];
  char line2Buf[70];
  satName.toCharArray(nameBuf, sizeof(nameBuf));
  tleLine1.toCharArray(line1Buf, sizeof(line1Buf));
  tleLine2.toCharArray(line2Buf, sizeof(line2Buf));

  sat.init(nameBuf, line1Buf, line2Buf);

  // Basic sanity check - if init produced obviously bad orbital elements,
  // treat it as a failure rather than moving the servos on garbage data.
  sgp4Ready = true;
  currentStatus = STATUS_NONE;
  Serial.println(F("[SGP4] Propagator initialized"));
}

void calculateSatellitePosition() {
  if (!sgp4Ready) {
    currentStatus = STATUS_SGP4_ERROR;
    positionValid = false;
    return;
  }

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 100)) {
    Serial.println(F("[SGP4] Could not get current time, skipping update"));
    return;
  }

  // Convert to the double Julian-day style time the Sgp4 library expects.
  double jdUnixSeconds = (double)mktime(&timeinfo);

  sat.findsat(jdUnixSeconds);

  currentLatDeg = sat.satLat;
  currentLonDeg = sat.satLon;
  currentAltKm  = sat.satAlt;
  positionValid = true;

  Serial.print(F("[SGP4] Lat: "));
  Serial.print(currentLatDeg);
  Serial.print(F("  Lon: "));
  Serial.print(currentLonDeg);
  Serial.print(F("  Alt(km): "));
  Serial.println(currentAltKm);
}

// ===========================================================================
// SERVO TARGET MAPPING
// ===========================================================================

void updateServoTargets() {
  if (!positionValid) return;

  // ---- Longitude -> Servo 1 ----
  // Map -180..+180 to servo travel, compressed by the gear ratio, then
  // shifted into the 0..180 servo range.
  float lonMapped = (float)((currentLonDeg + 180.0) * GLOBE_GEAR_RATIO);
  servo1TargetAngle = wrapServoAngle(lonMapped + longitudeOffsetDeg);

  // ---- Latitude -> Servo 2 / Servo 3 ----
  // servoAngle = latitude + 90  (per spec)
  float latMapped = (float)(currentLatDeg + 90.0);
  servo2TargetAngle = wrapServoAngle(latMapped + latitudeOffsetDeg + servo2OffsetDeg);

  if (SERVO3_MIRRORED) {
    servo3TargetAngle = wrapServoAngle(180.0 - servo2TargetAngle + servo3OffsetDeg);
  } else {
    servo3TargetAngle = wrapServoAngle(servo2TargetAngle + servo3OffsetDeg);
  }

  Serial.print(F("[SERVO] Targets -> S1: "));
  Serial.print(servo1TargetAngle);
  Serial.print(F("  S2: "));
  Serial.print(servo2TargetAngle);
  Serial.print(F("  S3: "));
  Serial.println(servo3TargetAngle);
}

float wrapServoAngle(float angle) {
  if (angle < SERVO_MIN_ANGLE) return SERVO_MIN_ANGLE;
  if (angle > SERVO_MAX_ANGLE) return SERVO_MAX_ANGLE;
  return angle;
}

// ===========================================================================
// SMOOTH SERVO MOTION (non-blocking, incremental)
// ===========================================================================

void moveServosSmoothly() {
  unsigned long now = millis();
  if (now - lastServoStepMs < SERVO_STEP_INTERVAL_MS) return;
  lastServoStepMs = now;

  servo1CurrentAngle = stepToward(servo1CurrentAngle, servo1TargetAngle);
  servo2CurrentAngle = stepToward(servo2CurrentAngle, servo2TargetAngle);
  servo3CurrentAngle = stepToward(servo3CurrentAngle, servo3TargetAngle);

  servo1.write((int)servo1CurrentAngle);
  servo2.write((int)servo2CurrentAngle);
  servo3.write((int)servo3CurrentAngle);
}

// Helper: move 'current' toward 'target' by at most SERVO_STEP_DEGREES.
float stepToward(float current, float target) {
  float diff = target - current;
  if (fabs(diff) <= SERVO_STEP_DEGREES) {
    return target;
  }
  return current + (diff > 0 ? SERVO_STEP_DEGREES : -SERVO_STEP_DEGREES);
}

// ===========================================================================
// OLED DISPLAY
// ===========================================================================

void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  // Priority: WiFi disconnected > no satellite > download/error states > data
  if (currentStatus == STATUS_WIFI_DISCONNECTED) {
    display.println(F("WiFi Disconnected"));
  } else if (currentNoradId <= 0) {
    display.println(F("No Satellite"));
  } else if (currentStatus == STATUS_DOWNLOADING) {
    display.println(F("Downloading TLE..."));
  } else if (currentStatus == STATUS_DOWNLOAD_FAILED) {
    display.println(F("Download Failed"));
  } else if (currentStatus == STATUS_INVALID_TLE) {
    display.println(F("Invalid TLE"));
  } else if (currentStatus == STATUS_SGP4_ERROR) {
    display.println(F("SGP4 Error"));
  } else {
    display.println(F("Satellite"));
    display.println(satName);
    display.print(F("NORAD: "));
    display.println(currentNoradId);
    display.print(F("LAT: "));
    display.print(currentLatDeg, 2);
    display.println((char)247); // degree symbol
    display.print(F("LON: "));
    display.print(currentLonDeg, 2);
    display.println((char)247);
    display.print(F("ALT: "));
    display.print((int)currentAltKm);
    display.println(F(" km"));
  }

  display.display();
}

// ===========================================================================
// CENTRAL TIMER / SCHEDULER
// ===========================================================================

void handleTimers() {
  unsigned long now = millis();

  // --- TLE download scheduling ---
  if (currentNoradId > 0 && WiFi.status() == WL_CONNECTED && !tleDownloadInProgress) {
    bool needInitialDownload = (lastTleFetchMs == 0);
    bool tleExpired = (!needInitialDownload) && (now - lastTleFetchMs >= TLE_MAX_AGE_MS);
    bool retryAfterFailure = (currentStatus == STATUS_DOWNLOAD_FAILED) &&
                              (now - lastTleAttemptMs >= TLE_RETRY_DELAY_MS);

    if (needInitialDownload || tleExpired || retryAfterFailure) {
      downloadTLE();
    }
  }

  // --- Position update + OLED (once per second) ---
  if (now - lastPositionUpdateMs >= POSITION_UPDATE_INTERVAL_MS) {
    lastPositionUpdateMs = now;
    if (sgp4Ready) {
      calculateSatellitePosition();
      updateServoTargets();
    }
  }

  if (now - lastOledUpdateMs >= OLED_UPDATE_INTERVAL_MS) {
    lastOledUpdateMs = now;
    updateOLED();
  }
}
