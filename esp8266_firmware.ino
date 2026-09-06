// =============================================================================
// esp8266_firmware.ino
//
// Features:
// - Non-blocking motion control (timed moves only — dur_ms required)
// - Watchdog: stops motors if server goes silent (motorsActive flag)
// - Emergency stop (hard override, latching)
// - Motor enable / disable
// - Magnet control (N-MOSFET via 4.7k gate resistor — GPIO0 boot safe)
// - HTTP JSON command interface
//
// IP reserved in router via MAC binding — no static config in firmware.
// =============================================================================

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>

// ===== PIN DEFINITIONS =====
#define D0 16
#define D1 5
#define D2 4
#define D3 0
#define D4 2
#define D5 14
#define D6 12
#define D7 13
#define D8 15

// ===== WIFI CONFIG =====
const char* ssid     = " "; //Use your wifi name inside the inverted commas, no spaces. 
const char* password = " ";//Use your wifi password inside the inverted commas, no spaces. 

ESP8266WebServer server(80);

// ===== PIN ASSIGNMENTS =====
#define MAGNET_PIN D3   // GPIO0 — N-MOSFET gate via 4.7k, must use resistor for safe boot

#define AIN1 D1         // GPIO5
#define AIN2 D2         // GPIO4
#define BIN1 D5         // GPIO14
#define BIN2 D6         // GPIO12

#define PWMA D7         // GPIO13 — Left motor PWM
#define PWMB D4         // GPIO2  — Right motor PWM

// ===== STATE VARIABLES =====
unsigned long moveEndTime = 0;
bool isMoving             = false;  // true only during a timed move
bool motorsActive         = false;  // true whenever motors are physically running
bool emergencyStop        = false;
bool motorsEnabled        = true;

// ===== WATCHDOG =====
unsigned long lastCmdTime          = 0;
const unsigned long CMD_TIMEOUT_MS = 500; // ~2-3x camera loop latency

// =============================================================================
// MOTOR CONTROL
// =============================================================================
void setMotorA(int speed) {
  if (!motorsEnabled || emergencyStop) speed = 0;
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    analogWrite(PWMA, speed);
  } else if (speed < 0) {
    digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -speed);
  } else {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, 0);
  }
}

void setMotorB(int speed) {
  if (!motorsEnabled || emergencyStop) speed = 0;
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    analogWrite(PWMB, speed);
  } else if (speed < 0) {
    digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -speed);
  } else {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, 0);
  }
}

void stopMotors() {
  setMotorA(0);
  setMotorB(0);
  isMoving     = false;
  motorsActive = false;
}

// =============================================================================
// HTTP HANDLERS
// =============================================================================
void handleRoot() {
  String html = "<h2>Robot Online</h2>";
  html += "<p>IP: "  + WiFi.localIP().toString() + "</p>";
  html += "<p>MAC: " + WiFi.macAddress()          + "</p>";
  html += "<p><a href='/ping'>Ping</a></p>";
  server.send(200, "text/html", html);
}

void handlePing() {
  server.send(200, "text/plain", "OK");
}

void handleCmd() {
  if (!server.hasArg("plain")) {
    server.send(400, "text/plain", "No body");
    return;
  }

  if (server.arg("plain").length() > 256) {
    server.send(413, "text/plain", "Payload too large");
    return;
  }

  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, server.arg("plain"))) {
    server.send(400, "text/plain", "Bad JSON");
    return;
  }

  const char* cmd = doc["cmd"];
  if (!cmd) {
    server.send(400, "text/plain", "Missing cmd");
    return;
  }

  // ================= MOVE =================
  if (strcmp(cmd, "MOVE") == 0) {

    // Validate before updating watchdog — blocked commands don't reset timer
    if (emergencyStop || !motorsEnabled) {
      server.send(403, "text/plain", "Blocked");
      return;
    }

    int speed_l = doc["speed_l"] | 0;
    int speed_r = doc["speed_r"] | 0;
    int dur_ms  = doc["dur_ms"]  | 0;

    // Validate fully before touching hardware or resetting watchdog
    bool willMove = (speed_l != 0 || speed_r != 0);
    if (!willMove) {
      server.send(400, "text/plain", "Zero speed not allowed");
      return;
    }

    if (dur_ms <= 0) {
      server.send(400, "text/plain", "dur_ms required");
      return;
    }

    lastCmdTime = millis();  // reset watchdog only after full validation

    setMotorA(speed_l);
    setMotorB(speed_r);
    motorsActive = true;  // set only after motors physically start

    moveEndTime = millis() + dur_ms;
    isMoving    = true;

    server.send(200, "text/plain", "OK");
  }

  // ================= STOP =================
  else if (strcmp(cmd, "STOP") == 0) {
    stopMotors();
    server.send(200, "text/plain", "STOPPED");
  }

  // ================= EMERGENCY STOP =================
  else if (strcmp(cmd, "EMERGENCY_STOP") == 0) {
    emergencyStop = true;
    stopMotors();
    digitalWrite(MAGNET_PIN, LOW);
    server.send(200, "text/plain", "EMERGENCY");
  }

  else if (strcmp(cmd, "CLEAR_EMERGENCY") == 0) {
    emergencyStop = false;
    lastCmdTime   = millis();  // reset watchdog when resuming
    server.send(200, "text/plain", "CLEARED");
  }

  // ================= MOTOR CONTROL =================
  else if (strcmp(cmd, "MOTOR_ON") == 0) {
    motorsEnabled = true;
    server.send(200, "text/plain", "MOTORS ON");
  }

  else if (strcmp(cmd, "MOTOR_OFF") == 0) {
    motorsEnabled = false;
    stopMotors();
    server.send(200, "text/plain", "MOTORS OFF");
  }

  // ================= MAGNET =================
  else if (strcmp(cmd, "MAGNET_ON") == 0) {
    if (emergencyStop) {
      server.send(403, "text/plain", "Blocked");
      return;
    }
    digitalWrite(MAGNET_PIN, HIGH);
    server.send(200, "text/plain", "MAGNET ON");
  }

  else if (strcmp(cmd, "MAGNET_OFF") == 0) {
    digitalWrite(MAGNET_PIN, LOW);
    server.send(200, "text/plain", "MAGNET OFF");
  }

  // ================= UNKNOWN =================
  else {
    server.send(400, "text/plain", "Unknown cmd");
  }
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);

  pinMode(AIN1,       OUTPUT);
  pinMode(AIN2,       OUTPUT);
  pinMode(BIN1,       OUTPUT);
  pinMode(BIN2,       OUTPUT);
  pinMode(PWMA,       OUTPUT);
  pinMode(PWMB,       OUTPUT);
  pinMode(MAGNET_PIN, OUTPUT);

  // Safe state on power-on / reboot
  stopMotors();
  digitalWrite(MAGNET_PIN, LOW);

  // Set PWM frequency for TB6612FNG
  analogWriteFreq(1000);  // 1 kHz — reduces motor noise vs default
  analogWriteRange(255); // match 0-255 speed range (ESP8266 default is 0-1023)

  // Initialise watchdog so it doesn't fire before server connects
  lastCmdTime = millis();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("[WiFi] Connecting");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] Connected! IP: %s  MAC: %s\n",
                  WiFi.localIP().toString().c_str(),
                  WiFi.macAddress().c_str());
  } else {
    Serial.println("\n[WiFi] FAILED to connect.");
  }

  server.on("/",     HTTP_GET,  handleRoot);
  server.on("/ping", HTTP_GET,  handlePing);
  server.on("/cmd",  HTTP_POST, handleCmd);

  server.begin();
  Serial.println("[HTTP] Server started.");
}

// =============================================================================
// LOOP
// =============================================================================
void loop() {
  server.handleClient();

  // Timed move: stop when dur_ms expires
  if (isMoving && (long)(millis() - moveEndTime) >= 0) {
    stopMotors();
    lastCmdTime = millis();  // prevent watchdog firing immediately after timed move
  }

  // Watchdog: if motors are physically running and server goes silent, stop
  if (!emergencyStop && !isMoving && motorsActive && (millis() - lastCmdTime > CMD_TIMEOUT_MS)) {
    stopMotors();
    Serial.println("[WDT] No command received — motors stopped.");
  }
}
