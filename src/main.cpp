/*
  ESP32 Input/Output Voltage + Current Monitor
  --------------------------------------------------------------------
  - Creates its own WiFi Access Point (no router needed)
  - Serves a live web dashboard (HTML built inline, no LittleFS/SPIFFS)
  - Reads INPUT voltage+current and OUTPUT voltage+current
  - Shows the same readings on a 16x2 I2C LCD

  Hardware assumptions (CHANGE THESE TO MATCH YOUR ACTUAL SENSORS):
  - Current sensors: ACS712-type analog Hall sensor (output centered
    at ~Vcc/2, linear mV/A). Swap the math in readCurrent() if you're
    using something else (INA219, shunt+amp module, ACS758, etc.)
  - Voltage sensors: simple resistive divider into an ADC pin
  - LCD: generic 16x2 character LCD on a PCF8574 I2C backpack
    (I2C address is usually 0x27 or 0x3F - check yours if blank)

  Wiring (change pins to whatever's free on your board):
    IN_VOLT_PIN   -> 34   (ADC1, input-only pin, safe choice on ESP32)
    IN_CURR_PIN   -> 35
    OUT_VOLT_PIN  -> 32
    OUT_CURR_PIN  -> 33
    RELAY_PIN     -> 25   (drives relay module IN pin; relay sits on OUT/battery path)
    IR_SENSOR_PIN -> 26   (digital obstacle/proximity module, e.g. FC-51; detects car present)
    LCD SDA       -> 21   (default ESP32 I2C SDA)
    LCD SCL       -> 22   (default ESP32 I2C SCL)
    LCD VCC       -> 5V
    LCD GND       -> GND

  Arduino IDE setup:
    Board: "ESP32 Dev Module" (or whatever your board is)
    Install "ESP32" board package via Boards Manager if not already done
    Install library: "LiquidCrystal I2C" by Frank de Brabander
      (Sketch > Include Library > Manage Libraries > search "LiquidCrystal I2C")
*/

#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>
// ---------------- USER CONFIG ----------------

// WiFi Access Point credentials
const char* AP_SSID = "PowerMonitor";
const char* AP_PASS = "12345678";   // must be 8+ chars, or "" for open network

// Sensor pins
const int IN_VOLT_PIN  = 34;
const int IN_CURR_PIN  = 35;
const int OUT_VOLT_PIN = 32;
const int OUT_CURR_PIN = 33;

// LCD config: address, columns, rows
// Common addresses: 0x27 or 0x3F. Run an I2C scanner sketch if unsure.
const uint8_t LCD_I2C_ADDR = 0x27;
const uint8_t LCD_COLS = 16;
const uint8_t LCD_ROWS = 2;

// ---- Calibration ----
const float ADC_MAX   = 4095.0;   // ESP32 ADC is 12-bit
const float ADC_VREF  = 3.3;      // ESP32 ADC reference voltage

// Voltage divider: Vin = Vadc * (R1+R2)/R2
// Example: R1 = 30k (top, to the high voltage), R2 = 7.5k (bottom, to GND)
// -> ratio = 5.0 means a 15V input reads as 3.0V at the ADC pin
const float IN_VOLT_DIVIDER_RATIO  = 5.0;
const float OUT_VOLT_DIVIDER_RATIO = 5.0;

// ACS712 current sensor
// - ACS712-05B: 185 mV/A,  zero-current output = Vcc/2
// - ACS712-20A: 100 mV/A
// - ACS712-30A: 66  mV/A
const float ACS_SENSITIVITY_V_PER_A = 0.100;  // set to match your module (20A version here)
const float ACS_ZERO_OFFSET_V       = ADC_VREF / 2.0; // volts at 0A, adjust after testing

const unsigned long SAMPLE_INTERVAL_MS = 200;
const unsigned long LCD_UPDATE_MS      = 500;

// Battery pack voltage range (assumes the battery sits on the OUTPUT side).
// 0% / 100% points - tune to your pack chemistry & cell count.
// Example here: 3S lead-acid/Li-ion-ish 12V pack.
const float BATT_MIN_V = 10.5;
const float BATT_MAX_V = 12.6;
const float CHARGE_CURRENT_THRESHOLD_A = 0.05; // above this, count as "charging"

// ---- Relay (load/charge disconnect for resting-voltage check) ----
const int RELAY_PIN = 25;
const bool RELAY_ACTIVE_HIGH = true; // false if relay module is active-LOW

const unsigned long RELAY_CHECK_INTERVAL_MS = 60000; // how often to cut the relay and check resting voltage
const unsigned long RELAY_SETTLE_MS         = 300;   // wait after cutoff for voltage to settle before sampling
const unsigned long RELAY_OFF_DURATION_MS   = 800;   // total time relay stays off per check (must be > RELAY_SETTLE_MS)

// ---- IR obstacle/proximity sensor (car presence) ----
const int IR_SENSOR_PIN = 26;
const bool IR_ACTIVE_LOW = true; // most FC-51/LM393 modules pull LOW when object detected
const unsigned long IR_DEBOUNCE_MS = 100; // ignore state flips shorter than this

// ------------------------------------------------

WebServer server(80);
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

float inVoltage = 0, inCurrent = 0, outVoltage = 0, outCurrent = 0;
unsigned long lastSample = 0;
unsigned long lastLcdUpdate = 0;

enum RelayState { RELAY_STATE_ON, RELAY_STATE_OFF };
RelayState relayState = RELAY_STATE_ON;
unsigned long relayStateChangedAt = 0;
float restingBattVoltage = 0; // battery voltage sampled with relay off (no load), 0 = not sampled yet
bool restingSampledThisCycle = false;

bool carDetected = false;
bool carDetectedPrev = false;
bool carDetectedRaw = false;
unsigned long carDetectedRawChangedAt = 0;

void setRelay(bool on) {
  int onLevel = RELAY_ACTIVE_HIGH ? HIGH : LOW;
  digitalWrite(RELAY_PIN, on ? onLevel : !onLevel);
}

bool readIrRaw() {
  int level = digitalRead(IR_SENSOR_PIN);
  return IR_ACTIVE_LOW ? (level == LOW) : (level == HIGH);
}

// ---------- Sensor reading helpers ----------

float readVoltage(int pin, float dividerRatio) {
  int raw = analogRead(pin);
  float vAdc = (raw / ADC_MAX) * ADC_VREF;
  return vAdc * dividerRatio;
}

float readCurrent(int pin) {
  int raw = analogRead(pin);
  float vAdc = (raw / ADC_MAX) * ADC_VREF;
  float amps = (vAdc - ACS_ZERO_OFFSET_V) / ACS_SENSITIVITY_V_PER_A;
  return amps;
}

float batteryPercent(float voltage) {
  float pct = (voltage - BATT_MIN_V) / (BATT_MAX_V - BATT_MIN_V) * 100.0;
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return pct;
}

// ---------- LCD ----------

void updateLcd() {
  char line0[17];
  char line1[17];
  // "I 12.34V 0.50A" style, fits in 16 chars
  snprintf(line0, sizeof(line0), "I %5.2fV %4.2fA", inVoltage, inCurrent);
  snprintf(line1, sizeof(line1), "O %5.2fV %4.2fA", outVoltage, outCurrent);

  lcd.setCursor(0, 0);
  lcd.print(line0);
  lcd.setCursor(0, 1);
  lcd.print(line1);
}

// ---------- Web page (served from LittleFS: data/index.html) ----------

void handleData() {
  // Prefer resting (no-load) voltage for SOC once we've sampled it - under load,
  // voltage sag/surge from OUT current makes battPct unreliable.
  float pct = batteryPercent(restingBattVoltage > 0 ? restingBattVoltage : outVoltage);
  bool charging = outCurrent > CHARGE_CURRENT_THRESHOLD_A;

  String json = "{";
  json += "\"inV\":" + String(inVoltage, 3) + ",";
  json += "\"inI\":" + String(inCurrent, 3) + ",";
  json += "\"outV\":" + String(outVoltage, 3) + ",";
  json += "\"outI\":" + String(outCurrent, 3) + ",";
  json += "\"battPct\":" + String(pct, 1) + ",";
  json += "\"charging\":" + String(charging ? "true" : "false") + ",";
  json += "\"relayOn\":" + String(relayState == RELAY_STATE_ON && carDetected ? "true" : "false") + ",";
  json += "\"restingV\":" + String(restingBattVoltage, 3) + ",";
  json += "\"carDetected\":" + String(carDetected ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

// ---------- Setup / loop ----------

void setup() {
  Serial.begin(115200);

  analogReadResolution(12); // ESP32 default, explicit for clarity

  pinMode(RELAY_PIN, OUTPUT);
  relayStateChangedAt = millis();

  pinMode(IR_SENSOR_PIN, INPUT);
  carDetectedRaw = readIrRaw();
  carDetected = carDetectedRaw;
  carDetectedPrev = carDetected;
  carDetectedRawChangedAt = millis();
  setRelay(carDetected);

  Wire.begin(); // default SDA=21, SCL=22 on most ESP32 boards
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Power Monitor");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed.");
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP started. Connect to WiFi: ");
  Serial.println(AP_SSID);
  Serial.print("Then open: http://");
  Serial.println(WiFi.softAPIP());

  server.serveStatic("/", LittleFS, "/index.html");
  server.on("/data", handleData);
  server.begin();

  delay(1000);
  lcd.clear();
}

void loop() {
  server.handleClient();

  unsigned long now = millis();

  if (now - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = now;
    inVoltage  = readVoltage(IN_VOLT_PIN, IN_VOLT_DIVIDER_RATIO);
    inCurrent  = readCurrent(IN_CURR_PIN);
    outVoltage = readVoltage(OUT_VOLT_PIN, OUT_VOLT_DIVIDER_RATIO);
    outCurrent = readCurrent(OUT_CURR_PIN);
  }

  if (now - lastLcdUpdate >= LCD_UPDATE_MS) {
    lastLcdUpdate = now;
    updateLcd();
  }

  // Debounce the IR obstacle sensor - ignore flips shorter than IR_DEBOUNCE_MS.
  bool irRaw = readIrRaw();
  if (irRaw != carDetectedRaw) {
    carDetectedRaw = irRaw;
    carDetectedRawChangedAt = now;
  } else if (carDetectedRaw != carDetected && now - carDetectedRawChangedAt >= IR_DEBOUNCE_MS) {
    carDetected = carDetectedRaw;
  }

  if (carDetected != carDetectedPrev) {
    carDetectedPrev = carDetected;
    relayState = RELAY_STATE_ON;
    relayStateChangedAt = now;
    setRelay(carDetected); // car just arrived -> engage relay; car just left -> open it
  }

  if (!carDetected) {
    // No car parked - relay stays open, skip the periodic resting-voltage check.
  } else if (relayState == RELAY_STATE_ON && now - relayStateChangedAt >= RELAY_CHECK_INTERVAL_MS) {
    // Periodically cut the relay to sample OUT voltage with no load/charge current,
    // giving a true resting battery voltage for SOC instead of one skewed by sag/surge.
    relayState = RELAY_STATE_OFF;
    relayStateChangedAt = now;
    restingSampledThisCycle = false;
    setRelay(false);
  } else if (relayState == RELAY_STATE_OFF) {
    if (!restingSampledThisCycle && now - relayStateChangedAt >= RELAY_SETTLE_MS) {
      restingBattVoltage = readVoltage(OUT_VOLT_PIN, OUT_VOLT_DIVIDER_RATIO);
      restingSampledThisCycle = true;
    }
    if (now - relayStateChangedAt >= RELAY_OFF_DURATION_MS) {
      relayState = RELAY_STATE_ON;
      relayStateChangedAt = now;
      setRelay(true);
    }
  }
}
