#include <Arduino.h>
#include <DHT.h>
#include <LiquidCrystal.h>

// =================== WIFI SECTION ===================
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>

#define WIFI_SSID     "Loumia"
#define WIFI_PASSWORD "gzqi8181"

WebServer server(80);
// ====================================================


// ------------------- Pin Mapping -------------------
// DHT: we keep only ONE temperature sensor now
#define PIN_DHT1         23      // single DHT11
#define DHTTYPE          DHT11

// PIR / presence indicator
#define PIN_PIR          15
#define PIN_PRES_LED     33      // presence LED

// Stepper (door)
#define PIN_STEP_IN1     19
#define PIN_STEP_IN2     18
#define PIN_STEP_IN3     5
#define PIN_STEP_IN4     17

// Fire sensor (digital, active LOW assumed) – ONLY FIRE2 NOW
#define PIN_FIRE2        35

// Single buzzer
#define BUZZER1          13      // only one buzzer now

// Fire alert LED (LED at 22 blinks when fire)
#define PIN_FIRE_ALERT_LED 22

// Room buttons + LEDs (room lights)
int pinButton1 = 34;     // GPIO2 (fixed from 50 – invalid pin)
int pinButton2 = 0;     // GPIO0
int pinLED1    = 4;     // GPIO4
int pinLED2    = 16;    // GPIO16

// Fan relay SRD-05VDC control input
#define PIN_FAN_RELAY    21   // GPIO21 -> IN of relay module

// ----------------- Relay Logic --------------------
// You set this in your latest code (active HIGH)
#define RELAY_ACTIVE_LEVEL    HIGH
#define RELAY_INACTIVE_LEVEL  LOW

// ----------------- Timing & Thresholds -------------
const unsigned long TEMP_PERIOD_MS       = 2000UL;
// PIR period was 10000, now half -> 5000
const unsigned long PIR_PERIOD_MS        = 5000UL;    // Check PIR every 5 seconds
const unsigned long PIR_LED_DURATION_MS  = 10000UL;   // 10 seconds indicator LED
const unsigned long FIRE_LATCH_MS        = 10000UL;
const unsigned long DOOR_WAIT_MS         = 10000UL;   // 10 seconds auto-close
const unsigned long DEBOUNCE_MS          = 50UL;

const int           STEPS_QUARTER        = 1024;
const unsigned long STEP_INTERVAL_US     = 2000UL;

const float         TEMP_FAN_ON_C        = 27.0f;     // Fan ON when T > 27°C

// -------------- Globals / Objects ------------------
DHT dht1(PIN_DHT1, DHTTYPE);

// LCD wiring (RW tied to GND in hardware)
#define LCD_RS 25
#define LCD_E  32
#define LCD_D4 26
#define LCD_D5 27
#define LCD_D6 14
#define LCD_D7 2

LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

const uint8_t STEP_SEQ[8][4] = {
  {1,0,0,0},
  {1,1,0,0},
  {0,1,0,0},
  {0,1,1,0},
  {0,0,1,0},
  {0,0,1,1},
  {0,0,0,1},
  {1,0,0,1}
};

int stepIndex = 0;

unsigned long lastTempReadMs = 0;
unsigned long lastPirReadMs  = 0;
unsigned long lastStepUs     = 0;

bool          presenceDetected = false;
unsigned long pirDetectedAtMs  = 0;

// Fire latching – ONLY FIRE2
unsigned long fire2DetectedAt  = 0;
bool          fire2Latched     = false;

// Pending alert for the APP (fire, etc.)
String pendingAlert = "";

// ----------------- Motor State ---------------------
enum MotorPhase {
  PHASE_IDLE,
  PHASE_FORWARD,   // door opening
  PHASE_WAIT,      // door open, waiting
  PHASE_BACKWARD   // door closing
};

MotorPhase    motorPhase = PHASE_IDLE;
int           stepsDone  = 0;
unsigned long waitEndMs  = 0;

// -------- Button debounce / state for each room ----
bool led1State = false;
bool led2State = false;

bool lastBtn1Reading = HIGH;
bool lastBtn2Reading = HIGH;
bool stableBtn1State = HIGH;
bool stableBtn2State = HIGH;

unsigned long lastBtn1Change = 0;
unsigned long lastBtn2Change = 0;

// ----------------- Fan Control Mode ----------------
enum FanMode {
  FAN_MODE_AUTO,
  FAN_MODE_FORCED_OFF,
  FAN_MODE_FORCED_ON
};

FanMode fanMode = FAN_MODE_AUTO;   // Default: automatic based on temperature

// Runtime states for status reporting
bool fanState    = false; // true = ON
bool buz2State   = false; // sensor2 latching state (only sensor now)

// Last measured active temperature (°C) used for fan logic/status
float lastActiveTemp = NAN;


// ================= WIFI COMMAND HANDLER =================
void sendPlain(const String &msg) {
  server.send(200, "text/plain", msg);
}

void handleCommand(String cmdRaw) {
  // Keep original for LCD text
  String original = cmdRaw;
  original.trim();

  // Lowercase for logic
  String cmd = original;
  cmd.trim();
  cmd.toLowerCase();

  // Handle "are you connected ?" variations
  if (cmd == "are you connected ?" ||
      cmd == "are you connected?"  ||
      cmd == "are you connected") {
    if (WiFi.status() == WL_CONNECTED) {
      sendPlain("Yes, I am connected.");
    } else {
      sendPlain("No, I am not connected.");
    }
    return;
  }


  if (cmd == "lights on") {
    if (led1State && led2State) {
      sendPlain("Lights are already ON.");
    } else {
      led1State = true;
      led2State = true;
      digitalWrite(pinLED1, HIGH);
      digitalWrite(pinLED2, HIGH);
      sendPlain("Lights are now ON.");
    }

  } else if (cmd == "lights off") {
    if (!led1State && !led2State) {
      sendPlain("Lights are already OFF.");
    } else {
      led1State = false;
      led2State = false;
      digitalWrite(pinLED1, LOW);
      digitalWrite(pinLED2, LOW);
      sendPlain("Lights are now OFF.");
    }

  } else if (cmd == "open door") {
    if (motorPhase == PHASE_IDLE) {
      stepsDone  = 0;
      motorPhase = PHASE_FORWARD;
      sendPlain("Door is opening.");
    } else if (motorPhase == PHASE_WAIT) {
      sendPlain("Door is already open.");
    } else {
      sendPlain("Door is currently moving. Please wait.");
    }

  } else if (cmd == "close door") {
    if (motorPhase == PHASE_WAIT) {
      stepsDone  = 0;
      motorPhase = PHASE_BACKWARD;
      sendPlain("Door is closing.");
    } else if (motorPhase == PHASE_IDLE) {
      sendPlain("Door is already closed.");
    } else {
      sendPlain("Door is currently moving. Please wait.");
    }

  } else if (cmd == "fan on") {
    fanMode  = FAN_MODE_FORCED_ON;
    fanState = true;
    digitalWrite(PIN_FAN_RELAY, RELAY_ACTIVE_LEVEL);
    sendPlain("Fan is now ON.");

  } else if (cmd == "fan off") {
    fanMode  = FAN_MODE_FORCED_OFF;
    fanState = false;
    digitalWrite(PIN_FAN_RELAY, RELAY_INACTIVE_LEVEL);
    sendPlain("Fan is now OFF.");

  } else if (cmd == "fan auto") {
    fanMode = FAN_MODE_AUTO;
    sendPlain("Fan is now in AUTO mode.");

  } else {
    sendPlain("Unknown command.");
  }
}
// =========================================================


// ------------------- Helper Functions ---------------------
// Temp LEDs are removed in this hardware config; keep stub for compatibility
void setTempLEDs(float tC) {
  // No temperature status LEDs in this version
}

void doOneHalfStepCW() {
  digitalWrite(PIN_STEP_IN1, STEP_SEQ[stepIndex][0]);
  digitalWrite(PIN_STEP_IN2, STEP_SEQ[stepIndex][1]);
  digitalWrite(PIN_STEP_IN3, STEP_SEQ[stepIndex][2]);
  digitalWrite(PIN_STEP_IN4, STEP_SEQ[stepIndex][3]);
  stepIndex = (stepIndex + 1) & 0x07;
}

void doOneHalfStepCCW() {
  stepIndex = (stepIndex - 1) & 0x07;
  digitalWrite(PIN_STEP_IN1, STEP_SEQ[stepIndex][0]);
  digitalWrite(PIN_STEP_IN2, STEP_SEQ[stepIndex][1]);
  digitalWrite(PIN_STEP_IN3, STEP_SEQ[stepIndex][2]);
  digitalWrite(PIN_STEP_IN4, STEP_SEQ[stepIndex][3]);
}

void stopMotor() {
  digitalWrite(PIN_STEP_IN1, LOW);
  digitalWrite(PIN_STEP_IN2, LOW);
  digitalWrite(PIN_STEP_IN3, LOW);
  digitalWrite(PIN_STEP_IN4, LOW);
}

// -------- Room buttons / LEDs (exclusive control) --------
void setupRoomControls() {
  pinMode(pinButton1, INPUT_PULLUP);
  pinMode(pinButton2, INPUT_PULLUP);

  pinMode(pinLED1, OUTPUT);
  pinMode(pinLED2, OUTPUT);

  led1State = false;
  led2State = false;

  digitalWrite(pinLED1, LOW);
  digitalWrite(pinLED2, LOW);

  lastBtn1Reading = digitalRead(pinButton1);
  stableBtn1State = lastBtn1Reading;
  lastBtn1Change  = millis();

  lastBtn2Reading = digitalRead(pinButton2);
  stableBtn2State = lastBtn2Reading;
  lastBtn2Change  = millis();
}

void updateRoomControls(unsigned long nowMs) {
  // BUTTON 1 → LED 1 only
  int raw1 = digitalRead(pinButton1);
  if (raw1 != lastBtn1Reading) {
    lastBtn1Reading = raw1;
    lastBtn1Change  = nowMs;
  }
  if ((nowMs - lastBtn1Change) > DEBOUNCE_MS) {
    if (stableBtn1State == HIGH && raw1 == LOW) {
      led1State = !led1State;
      digitalWrite(pinLED1, led1State ? HIGH : LOW);
      Serial.println("[BTN1] LED1 toggled");
    }
    stableBtn1State = raw1;
  }

  // BUTTON 2 → LED 2 only
  int raw2 = digitalRead(pinButton2);
  if (raw2 != lastBtn2Reading) {
    lastBtn2Reading = raw2;
    lastBtn2Change  = nowMs;
  }
  if ((nowMs - lastBtn2Change) > DEBOUNCE_MS) {
    if (stableBtn2State == HIGH && raw2 == LOW) {
      led2State = !led2State;
      digitalWrite(pinLED2, led2State ? HIGH : LOW);
      Serial.println("[BTN2] LED2 toggled");
    }
    stableBtn2State = raw2;
  }
}

// ================== SETUP =====================
void setup() {
  Serial.begin(115200);
  delay(200);

  dht1.begin();

  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_PRES_LED, OUTPUT);

  pinMode(PIN_STEP_IN1, OUTPUT);
  pinMode(PIN_STEP_IN2, OUTPUT);
  pinMode(PIN_STEP_IN3, OUTPUT);
  pinMode(PIN_STEP_IN4, OUTPUT);
  stopMotor();

  // Only FIRE2 now
  pinMode(PIN_FIRE2, INPUT);

  pinMode(BUZZER1, OUTPUT);
  digitalWrite(BUZZER1, LOW);

  // Fire alert LED on pin 22
  pinMode(PIN_FIRE_ALERT_LED, OUTPUT);
  digitalWrite(PIN_FIRE_ALERT_LED, LOW);

  setupRoomControls();   // room buttons + LEDs

  // Fan relay
  pinMode(PIN_FAN_RELAY, OUTPUT);
  digitalWrite(PIN_FAN_RELAY, RELAY_INACTIVE_LEVEL); // fan OFF initially

  fanMode = FAN_MODE_AUTO;
  fanState = false;

  setTempLEDs(NAN);

  // LCD init
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Smart Home Ready");

  // ================= WIFI SETUP (DHCP only) =================
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to WiFi ");
  Serial.print(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }
  Serial.println(" connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // Simple endpoints
  server.on("/", HTTP_GET, []() {
    sendPlain("ESP32 Smart Home is online.");
  });

  // GET /command?cmd=...
  server.on("/command", HTTP_GET, []() {
    if (!server.hasArg("cmd")) {
      sendPlain("Missing command parameter.");
      return;
    }
    handleCommand(server.arg("cmd"));
  });

  // POST /command  with plain text body like "lights on", "open door", etc.
  server.on("/command", HTTP_POST, []() {
    String body = server.arg("plain");
    if (body.length() == 0) {
      sendPlain("Command is empty.");
      return;
    }
    handleCommand(body);
  });

  // GET /alert  -> used by the APP to check for pending alerts (fire, etc.)
  server.on("/alert", HTTP_GET, []() {
    if (pendingAlert.length() == 0) {
      sendPlain("No alerts.");
    } else {
      sendPlain(pendingAlert);
      pendingAlert = "";   // Clear after reading
    }
  });

  // GET /status -> JSON with all entity states + temperature
  server.on("/status", HTTP_GET, []() {
    // Door state as string
    String doorState;
    switch (motorPhase) {
      case PHASE_IDLE:      doorState = "closed";  break;
      case PHASE_WAIT:      doorState = "open";    break;
      case PHASE_FORWARD:   doorState = "opening"; break;
      case PHASE_BACKWARD:  doorState = "closing"; break;
      default:              doorState = "unknown"; break;
    }

    // Fan mode as string
    String fanModeStr;
    if (fanMode == FAN_MODE_AUTO)            fanModeStr = "auto";
    else if (fanMode == FAN_MODE_FORCED_ON)  fanModeStr = "forced_on";
    else if (fanMode == FAN_MODE_FORCED_OFF) fanModeStr = "forced_off";
    else                                     fanModeStr = "unknown";

    bool alarmActive = (buz2State);  // only sensor2 now

    // Temperature field: numeric if valid, null if not
    String tempField;
    if (isnan(lastActiveTemp)) {
      tempField = "null";
    } else {
      tempField = String(lastActiveTemp, 1); // one decimal, e.g., 24.5
    }

    String json = "{";
    json += "\"lights\":{";
      json += "\"room1\":\"" + String(led1State ? "on" : "off") + "\","; 
      json += "\"room2\":\"" + String(led2State ? "on" : "off") + "\"";
    json += "},";

    json += "\"door\":\"" + doorState + "\",";

    json += "\"fan\":{";
      json += "\"state\":\"" + String(fanState ? "on" : "off") + "\","; 
      json += "\"mode\":\"" + fanModeStr + "\"";
    json += "},";

    json += "\"alarm\":{";
      json += "\"active\":" + String(alarmActive ? "true" : "false") + ",";
      // sensor1 removed; app will default to false via optBoolean
      json += "\"sensor2\":" + String(buz2State ? "true" : "false");
    json += "},";

    json += "\"presence\":" + String(presenceDetected ? "true" : "false") + ",";

    json += "\"temperature\":" + tempField;

    json += "}";

    server.send(200, "application/json", json);
  });

  server.begin();
  Serial.println("HTTP server started");
  // ==================================================

  Serial.println("System ready.");
}

// ================= LOOP =======================
void loop() {
  unsigned long nowMs = millis();
  unsigned long nowUs = micros();

  // -------- PIR & presence LED (indicator, checked every 5s) --------
  if (nowMs - lastPirReadMs >= PIR_PERIOD_MS) {
    lastPirReadMs = nowMs;
    bool pir = digitalRead(PIN_PIR);

    if (pir) { // assuming HIGH = motion
      presenceDetected = true;
      pirDetectedAtMs  = nowMs;
      Serial.println("[PIR] Presence detected (indicator LED only).");
    }
  }

  if (presenceDetected && (nowMs - pirDetectedAtMs >= PIR_LED_DURATION_MS)) {
    presenceDetected = false;
  }
  digitalWrite(PIN_PRES_LED, presenceDetected ? HIGH : LOW);

  // -------- Temperature & Fan every 2 s --------
  if (nowMs - lastTempReadMs >= TEMP_PERIOD_MS) {
    lastTempReadMs = nowMs;

    float tC1 = dht1.readTemperature();
    float h1  = dht1.readHumidity();

    if (!isnan(tC1)) {
      Serial.print("[DHT1] T=");
      Serial.print(tC1, 1);
      Serial.print(" C | H=");
      Serial.print(h1, 1);
      Serial.println(" %");
    }

    // Temp LEDs (stub)
    setTempLEDs(tC1);

    // Fan control: AUTO or FORCED
    float activeTemp = tC1;    // only one sensor now
    lastActiveTemp = activeTemp;

    bool fanShouldBeOn = false;
    if (fanMode == FAN_MODE_AUTO) {
      fanShouldBeOn = (!isnan(activeTemp) && activeTemp > TEMP_FAN_ON_C);
    } else if (fanMode == FAN_MODE_FORCED_ON) {
      fanShouldBeOn = true;
    } else if (fanMode == FAN_MODE_FORCED_OFF) {
      fanShouldBeOn = false;
    }

    fanState = fanShouldBeOn;
    digitalWrite(PIN_FAN_RELAY, fanShouldBeOn ? RELAY_ACTIVE_LEVEL : RELAY_INACTIVE_LEVEL);

    Serial.print("[FAN] Mode=");
    if (fanMode == FAN_MODE_AUTO)            Serial.print("AUTO");
    else if (fanMode == FAN_MODE_FORCED_ON)  Serial.print("FORCED_ON");
    else if (fanMode == FAN_MODE_FORCED_OFF) Serial.print("FORCED_OFF");
    Serial.print(" | activeTemp=");
    Serial.print(activeTemp);
    Serial.print(" => ");
    Serial.println(fanShouldBeOn ? "ON" : "OFF");
  }

  // -------- Fire detector (ONLY FIRE2) & buzzer + alert LED --------
  bool fire2 = digitalRead(PIN_FIRE2);  // active LOW

  if (!fire2) { // triggered
    fire2Latched    = true;
    fire2DetectedAt = nowMs;
    Serial.println("FIRE FIRE FIRE! [FIRE2]");
    pendingAlert = "Fire detected by sensor 2.";
  }

  if (fire2Latched && (nowMs - fire2DetectedAt >= FIRE_LATCH_MS)) {
    fire2Latched = false;
  }

  bool anyFireLatched = fire2Latched;

  // Map latch state to "sensor2" in /status
  buz2State = fire2Latched;

  // Single buzzer reacts to any fire (only FIRE2 now)
  digitalWrite(BUZZER1, anyFireLatched ? HIGH : LOW);

  // 🔥 LED at pin 22 blinks when fire is detected
  if (anyFireLatched) {
    bool ledOn = ((nowMs / 250) % 2) == 0;   // ~4 Hz blink
    digitalWrite(PIN_FIRE_ALERT_LED, ledOn ? HIGH : LOW);
  } else {
    digitalWrite(PIN_FIRE_ALERT_LED, LOW);
  }

  // -------- Room buttons / LEDs (manual control) --------
  updateRoomControls(nowMs);

  // ---------- MOTOR / DOOR STATE MACHINE ----------
  switch (motorPhase) {
    case PHASE_FORWARD:   // door opening
      if (stepsDone < STEPS_QUARTER) {
        if (nowUs - lastStepUs >= STEP_INTERVAL_US) {
          lastStepUs = nowUs;
          // CCW for open
          doOneHalfStepCCW();
          stepsDone++;
        }
      } else {
        stopMotor();
        Serial.println("[DOOR] Door fully open. Waiting to auto-close...");

        // When door opens, turn on room lights
        led1State = true;
        led2State = true;
        digitalWrite(pinLED1, HIGH);
        digitalWrite(pinLED2, HIGH);
        Serial.println("[LIGHTS] Door opened -> lights ON.");

        waitEndMs  = nowMs + DOOR_WAIT_MS;   // 10 seconds
        motorPhase = PHASE_WAIT;
      }
      break;

    case PHASE_WAIT:      // hold open, then close automatically
      if ((long)(nowMs - waitEndMs) >= 0) {
        Serial.println("[DOOR] Auto-close timer -> closing door.");
        stepsDone  = 0;
        motorPhase = PHASE_BACKWARD;
      }
      break;

    case PHASE_BACKWARD:  // door closing
      if (stepsDone < STEPS_QUARTER) {
        if (nowUs - lastStepUs >= STEP_INTERVAL_US) {
          lastStepUs = nowUs;
          // CW for close
          doOneHalfStepCW();
          stepsDone++;
        }
      } else {
        stopMotor();
        Serial.println("[DOOR] Door closed. Idle.");
        motorPhase = PHASE_IDLE;
      }
      break;

    case PHASE_IDLE:
    default:
      break;
  }

  // ============ HANDLE WIFI HTTP REQUESTS ============
  server.handleClient();
  // ===================================================

  delay(1);
}
