/*
  Smart Trash Bin - ESP32

  Boards/Libraries:
  - Board: ESP32 Dev Module
  - Built-in libraries: WiFi, HTTPClient, WiFiClientSecure, Wire

  Wiring:
  - HC-SR04 Trig  -> GPIO 5
  - HC-SR04 Echo  -> GPIO 18
    Important: HC-SR04 Echo is 5V. Use a voltage divider or level shifter for ESP32.
  - Buzzer        -> GPIO 19
  - Button        -> GPIO 4, connect the other side to GND
  - LED + 220 ohm -> GPIO 23
  - MPU-6050 SDA  -> GPIO 21
  - MPU-6050 SCL  -> GPIO 22
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <math.h>

const char* WIFI_SSID = "may.warun";
const char* WIFI_PASSWORD = "0955817117";

// Paste your Apps Script Web App URL ending with /exec
const char* SCRIPT_URL = "https://script.google.com/macros/s/AKfycbw3dNePJT4_pRbWsEqL1p8ynOVEgsyvLWY-HP0jKNxpVnRp0uiaO4m2Ww6WmITXnp05/exec";

const int TRIG_PIN = 5;
const int ECHO_PIN = 18;
const int BUZZER_PIN = 19;
const int BUTTON_PIN = 4;
const int LED_PIN = 23;

// true  = button connects GPIO4 to GND when pressed
// false = button connects GPIO4 to 3.3V when pressed
const bool BUTTON_ACTIVE_LOW = true;
const bool DEBUG_BUTTON = true;

const float BIN_HEIGHT_CM = 30.0;
const float FULL_PERCENT = 85.0;
const float FALL_ANGLE_DEG = 45.0;

const unsigned long SEND_INTERVAL_MS = 10000;
const unsigned long ALARM_POLL_INTERVAL_MS = 3000;
const unsigned long BUTTON_DEBOUNCE_MS = 50;
const unsigned long BUTTON_DEBUG_INTERVAL_MS = 500;
const unsigned long BUTTON_PRESS_COOLDOWN_MS = 500;

const uint8_t MPU_ADDR = 0x68;

bool systemOn = true;
bool remoteAlarmEnabled = true;
bool buzzerOn = false;

unsigned long lastSendMs = 0;
unsigned long lastAlarmPollMs = 0;
unsigned long lastButtonMs = 0;
unsigned long lastButtonDebugMs = 0;
unsigned long lastButtonPressAcceptedMs = 0;
int lastButtonReading = HIGH;
int stableButtonState = HIGH;
unsigned long buttonPressCount = 0;

float uprightAx = 0;
float uprightAy = 0;
float uprightAz = 1;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("===== Smart Trash ESP32 boot =====");

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, BUTTON_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(TRIG_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  lastButtonReading = digitalRead(BUTTON_PIN);
  stableButtonState = lastButtonReading;
  printButtonConfig();
  printSystemDebug("setup");

  Wire.begin(21, 22);
  initMPU6050();
  calibrateMPU6050();
  connectWiFi();
}

void loop() {
  handleButton();

  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  unsigned long now = millis();
  if (now - lastAlarmPollMs >= ALARM_POLL_INTERVAL_MS) {
    lastAlarmPollMs = now;
    pollRemoteAlarm();
  }

  float wasteHeightCm = -1;
  float wastePercent = 0;
  float tiltDeg = 0;
  String wasteStatus = "-";
  String binStatus = "-";
  String systemStatus = systemOn ? "เปิด" : "ปิด";

  if (systemOn) {
    float distanceCm = readUltrasonicCm();
    if (distanceCm >= 0) {
      distanceCm = constrain(distanceCm, 0, BIN_HEIGHT_CM);
      wasteHeightCm = BIN_HEIGHT_CM - distanceCm;
      wastePercent = (wasteHeightCm / BIN_HEIGHT_CM) * 100.0;
    }

    wasteStatus = getWasteStatus(wastePercent);
    tiltDeg = readTiltDegrees();
    binStatus = tiltDeg > FALL_ANGLE_DEG ? "ล้ม" : "ปกติ";

    buzzerOn = (binStatus == "ล้ม") && remoteAlarmEnabled;
    digitalWrite(BUZZER_PIN, buzzerOn ? HIGH : LOW);
    updateLed(wastePercent);
  } else {
    buzzerOn = false;
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
  }

  if (now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;
    sendData(wasteHeightCm, wasteStatus, binStatus, systemStatus, buzzerOn);
  }
}

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.print("Connecting WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < 20000) {
    handleButton();
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connection failed");
  }
}

void handleButton() {
  int reading = digitalRead(BUTTON_PIN);
  bool pressedNow = isButtonPressed(reading);
  printButtonDebug(reading, pressedNow);

  if (reading != lastButtonReading) {
    lastButtonMs = millis();
    lastButtonReading = reading;
    if (DEBUG_BUTTON) {
      Serial.print("[BUTTON] raw changed to ");
      Serial.println(reading == HIGH ? "HIGH" : "LOW");
    }

    if (pressedNow && millis() - lastButtonPressAcceptedMs >= BUTTON_PRESS_COOLDOWN_MS) {
      lastButtonPressAcceptedMs = millis();
      stableButtonState = reading;
      buttonPressCount++;
      if (DEBUG_BUTTON) {
        Serial.print("[BUTTON] press accepted immediately, count = ");
        Serial.println(buttonPressCount);
      }
      toggleSystem();
    }
    return;
  }

  if (millis() - lastButtonMs > BUTTON_DEBOUNCE_MS) {
    stableButtonState = reading;
  }
}

bool isButtonPressed(int state) {
  return BUTTON_ACTIVE_LOW ? state == LOW : state == HIGH;
}

void printButtonConfig() {
  if (!DEBUG_BUTTON) return;

  Serial.print("[BUTTON] pin GPIO");
  Serial.print(BUTTON_PIN);
  Serial.print(" mode: ");
  Serial.println(BUTTON_ACTIVE_LOW ? "INPUT_PULLUP, pressed = LOW" : "INPUT_PULLDOWN, pressed = HIGH");
  Serial.print("[BUTTON] initial raw state: ");
  Serial.println(stableButtonState == HIGH ? "HIGH" : "LOW");
}

void printButtonDebug(int reading, bool pressedNow) {
  if (!DEBUG_BUTTON || millis() - lastButtonDebugMs < BUTTON_DEBUG_INTERVAL_MS) return;

  lastButtonDebugMs = millis();
  Serial.print("[DEBUG] buttonRaw=");
  Serial.print(reading == HIGH ? "HIGH" : "LOW");
  Serial.print(" interpreted=");
  Serial.print(pressedNow ? "PRESSED" : "RELEASED");
  Serial.print(" stable=");
  Serial.print(stableButtonState == HIGH ? "HIGH" : "LOW");
  Serial.print(" system=");
  Serial.print(systemOn ? "ON" : "OFF");
  Serial.print(" wifi=");
  Serial.print(WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED");
  Serial.print(" pressCount=");
  Serial.println(buttonPressCount);
}

void printSystemDebug(const char* source) {
  if (!DEBUG_BUTTON) return;

  Serial.print("[SYSTEM] source=");
  Serial.print(source);
  Serial.print(" system=");
  Serial.print(systemOn ? "ON" : "OFF");
  Serial.print(" led=");
  Serial.print(digitalRead(LED_PIN) == HIGH ? "HIGH" : "LOW");
  Serial.print(" buzzer=");
  Serial.print(digitalRead(BUZZER_PIN) == HIGH ? "HIGH" : "LOW");
  Serial.print(" wifi=");
  Serial.println(WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED");
}

void toggleSystem() {
  systemOn = !systemOn;
  Serial.print("[SYSTEM] toggled to ");
  Serial.println(systemOn ? "ON" : "OFF");

  if (!systemOn) {
    buzzerOn = false;
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    sendData(-1, "-", "-", "ปิด", false);
  } else {
    lastSendMs = 0;
  }
  printSystemDebug("toggleSystem");
}

void initMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(100);
}

bool readAccelG(float& ax, float& ay, float& az) {
  int16_t axRaw, ayRaw, azRaw;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  Wire.requestFrom(MPU_ADDR, (uint8_t)6, (uint8_t)true);
  if (Wire.available() < 6) {
    return false;
  }

  axRaw = (Wire.read() << 8) | Wire.read();
  ayRaw = (Wire.read() << 8) | Wire.read();
  azRaw = (Wire.read() << 8) | Wire.read();

  ax = axRaw / 16384.0;
  ay = ayRaw / 16384.0;
  az = azRaw / 16384.0;
  return true;
}

void calibrateMPU6050() {
  Serial.println("Keep the bin upright. Calibrating MPU-6050...");

  float sumAx = 0;
  float sumAy = 0;
  float sumAz = 0;
  int samples = 0;

  for (int i = 0; i < 80; i++) {
    float ax, ay, az;
    if (readAccelG(ax, ay, az)) {
      sumAx += ax;
      sumAy += ay;
      sumAz += az;
      samples++;
    }
    delay(20);
  }

  if (samples == 0) {
    Serial.println("MPU calibration failed. Using default Z axis.");
    uprightAx = 0;
    uprightAy = 0;
    uprightAz = 1;
    return;
  }

  uprightAx = sumAx / samples;
  uprightAy = sumAy / samples;
  uprightAz = sumAz / samples;

  float magnitude = sqrt(uprightAx * uprightAx + uprightAy * uprightAy + uprightAz * uprightAz);
  if (magnitude > 0.01) {
    uprightAx /= magnitude;
    uprightAy /= magnitude;
    uprightAz /= magnitude;
  }

  Serial.print("Upright vector: ");
  Serial.print(uprightAx, 3);
  Serial.print(", ");
  Serial.print(uprightAy, 3);
  Serial.print(", ");
  Serial.println(uprightAz, 3);
}

float readUltrasonicCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) {
    Serial.println("Ultrasonic timeout");
    return -1;
  }

  return duration * 0.0343 / 2.0;
}

float readTiltDegrees() {
  float ax, ay, az;
  if (!readAccelG(ax, ay, az)) {
    return 0;
  }

  float magnitude = sqrt(ax * ax + ay * ay + az * az);
  if (magnitude <= 0.01) return 0;

  ax /= magnitude;
  ay /= magnitude;
  az /= magnitude;

  float cosAngle = ax * uprightAx + ay * uprightAy + az * uprightAz;
  cosAngle = fabs(cosAngle);
  cosAngle = constrain(cosAngle, 0.0, 1.0);
  return acos(cosAngle) * 180.0 / PI;
}

String getWasteStatus(float percent) {
  if (percent >= FULL_PERCENT) return "เต็มแล้ว";
  return "ยังไม่เต็ม";
}

void updateLed(float wastePercent) {
  if (wastePercent >= FULL_PERCENT) {
    digitalWrite(LED_PIN, HIGH);
    return;
  }

  digitalWrite(LED_PIN, LOW);
}

void pollRemoteAlarm() {
  if (WiFi.status() != WL_CONNECTED) return;

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  String url = String(SCRIPT_URL) + "?action=getAlarm";
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  if (!http.begin(client, url)) return;

  int code = http.GET();
  if (code == 200) {
    String payload = http.getString();
    remoteAlarmEnabled = payload.indexOf("\"alarmEnabled\":true") >= 0;
    Serial.print("Remote alarm enabled: ");
    Serial.println(remoteAlarmEnabled ? "true" : "false");
  } else {
    Serial.print("Alarm poll HTTP error: ");
    Serial.println(code);
  }

  http.end();
}

void sendData(float wasteHeightCm, const String& wasteStatus, const String& binStatus,
              const String& systemStatus, bool alarmActive) {
  if (WiFi.status() != WL_CONNECTED) return;

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  if (!http.begin(client, SCRIPT_URL)) return;
  http.addHeader("Content-Type", "text/plain; charset=utf-8");

  String heightValue = wasteHeightCm >= 0 ? String(wasteHeightCm, 2) : "null";
  String json = "{";
  json += "\"action\":\"log\",";
  json += "\"alarm\":\"" + String(alarmActive ? "เปิด" : "ปิด") + "\",";
  json += "\"wasteHeightCm\":" + heightValue + ",";
  json += "\"wasteStatus\":\"" + wasteStatus + "\",";
  json += "\"binStatus\":\"" + binStatus + "\",";
  json += "\"systemStatus\":\"" + systemStatus + "\"";
  json += "}";

  int code = http.POST(json);
  String response = http.getString();

  Serial.print("POST code: ");
  Serial.println(code);
  Serial.println(response);

  http.end();
}
