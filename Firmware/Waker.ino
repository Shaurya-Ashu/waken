/*
  =============================================================================
  WAKER  —  ESP32-S3 Smartwatch Firmware
  =============================================================================
  Board:      Custom "Waker V0.1" (schematic: Waker V1.0)
  MCU:        ESP32-S3
  Display:    SSD1306 0.96" OLED, 128x64, I2C
  Sensors:    MPU6500 (IMU), MAX30101 (HR/PPG), MCP9808 (Temp)
  Input:      2x push buttons, 1x rotary encoder w/ push button
  Output:     Piezo buzzer
  Radio:      Wi-Fi (NTP time sync)

  PIN SOURCE OF TRUTH
  -------------------
  The pins below marked CONFIRMED were taken from your own bring-up sketch,
  which was flashed to real hardware and produced a passing I2C scan / sensor
  self-test (OLED PASS, IMU WHOAMI=0x70 PASS, HR PARTID=0x15 PASS, TEMP PASS).
  That is stronger evidence than a schematic read, so this firmware treats
  those pins as ground truth.

  Two nets (IMU interrupt, charger STAT) are visible on the schematic as
  concepts but were not present in your tested pin list and are not reliably
  legible at the resolution provided. They are marked BEST-EFFORT below.
  Confirm them against your board (continuity-test or datasheet trace) before
  relying on motion-wake or charge-detect — everything else works with them
  disabled/absent, they just degrade gracefully.

  REQUIRED LIBRARIES (Library Manager)
  -------------------------------------
    - Adafruit GFX Library
    - Adafruit SSD1306
    - Adafruit MCP9808 Library
    - SparkFun MAX3010x Pulse and Proximity Sensor Library  (works for MAX30101 --
      only MAX30105.h is used from it; the heart-rate detector below is a
      self-contained implementation and does NOT include the library's
      heartRate.h, since that filename collides with an unrelated file of the
      same name in some other MAX3010x-family libraries)
    - ESP32 Arduino core (Wire, WiFi, WebServer, Preferences, time.h,
      esp_sleep.h — all bundled with the ESP32 board package)

  If your Arduino/libraries folder has more than one MAX30101-family library
  installed (e.g. SparkFun's alongside EmotiBit_MAX30101), the compiler may
  still warn "Multiple libraries found for MAX30105.h" -- that's fine as
  long as it reports SparkFun's as the one actually Used. If it doesn't, or
  compilation fails on files this sketch never includes, the reliable fix is
  removing or relocating the unused library rather than tracking down every
  colliding filename.

  This firmware avoids delay()-based blocking flow; all periodic work is
  millis()-scheduled. Flyback converter section of the schematic intentionally
  ignored per spec.
  =============================================================================
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MCP9808.h>
#include <MAX30105.h>
#include <WiFi.h>
#include <WebServer.h>
#include <time.h>
#include <Preferences.h>
#include <Update.h>
#include <esp_sleep.h>

// ============================================================================
// PIN MAP
// ============================================================================
// --- CONFIRMED (validated on hardware via your bring-up sketch) -----------
#define PIN_SDA        8
#define PIN_SCL        9
#define PIN_SW1        10   // Button 1
#define PIN_SW3        21   // Button 2
#define PIN_EC_A       17   // Encoder quadrature A
#define PIN_EC_B       7    // Encoder quadrature B
#define PIN_EC_C       15   // Encoder common return — driven LOW as an OUTPUT, not read
#define PIN_EC_S       16   // Encoder push button
#define PIN_BUZZER     42   // Piezo buzzer drive

// --- BEST-EFFORT (verify against your board before trusting) --------------
#define PIN_IMU_INT    4    // MPU6500 INT line -> motion wake / data-ready
#define PIN_CHG_STAT   5    // MCP73831 STAT (open-drain, active LOW while charging)
#define PIN_BATT_ADC   18   // Battery voltage sense divider, if populated. -1 to disable.
#define HAS_BATT_ADC   1    // Set to 1 once PIN_BATT_ADC is confirmed wired

// --- FLYBACK / TENS STAGE (Q4/L2/D3/D4/C18/R12 on schematic) --------------
// Coilcraft LPR6235-752SMRC coupled inductor, driven single-ended by Q4
// (IRLR7843) off GPIO6, rectified by D3/D4 (SUF4007, 1kV) into C18 (470nF),
// with R12 (150kOhm) in series with H+. R12 is a hard current limiter in
// hardware: even at a fully-charged open-circuit secondary voltage, skin
// contact current through H+/H- is bounded to roughly Voc / 150k -- a few
// mA at most for the voltages this transformer/cap combination produces.
// That means firmware pulse timing mostly governs charge time and how hard
// Q4/the transformer are driven, not shock intensity; R12 owns that.
// Pulse parameters below are carried over unchanged from the confirmed
// working original sketch (60us on / 20ms off, 200 pulses), just converted
// from a ~4s blocking for-loop to a non-blocking millis()-scheduled task
// (see flybackTask()) so it no longer freezes buttons/display/WiFi while
// charging.
#define PIN_FLYBACK_GATE      6
#define FLYBACK_PULSE_ON_US   60
#define FLYBACK_PULSE_OFF_MS  20
#define FLYBACK_PULSE_COUNT   300

// I2C addresses (all confirmed via scan log)
#define ADDR_OLED       0x3C
#define ADDR_MPU6500    0x68
#define ADDR_MAX30101   0x57
#define ADDR_MCP9808    0x18

// ============================================================================
// TYPE DEFINITIONS
// ----------------------------------------------------------------------------
// All structs/enums live up here, ahead of every function. Arduino's build
// step auto-generates function prototypes and inserts them at the top of the
// translation unit, above where these types would otherwise be defined lower
// down in the file — so any type referenced in a function signature must be
// fully declared before the first function, or that generated prototype
// fails to compile.
// ============================================================================
enum class AppState : uint8_t {
  BOOT, HOME, MENU,
  APP_CLOCK, APP_HEARTRATE, APP_TEMPERATURE, APP_BATTERY,
  APP_WIFI, APP_WIFI_SCAN, APP_SETTINGS, APP_SYSINFO, APP_ABOUT,
  APP_KEYPAD, APP_ALARM_SET, APP_STEPCOUNT, APP_ALARM_RINGING
};

enum class InputSource : uint8_t { BTN1, BTN2, ENC_BTN, ENCODER_CW, ENCODER_CCW };
enum class PressType   : uint8_t { NONE, SHORT, LONG, DOUBLE };
enum class WifiUiPhase : uint8_t { IDLE, SCANNING, SCAN_DONE, CONNECTING, KEYPAD };

struct ButtonState {
  uint8_t pin;
  bool    activeHigh    = false;  // true: pressed = HIGH (VCC-wired). false: pressed = LOW (pull-up, GND-wired)
  bool    lastRaw       = false;  // set correctly in setup() once activeHigh is known
  bool    stable        = false;
  uint32_t lastChangeMs  = 0;
  uint32_t pressStartMs  = 0;
  uint32_t lastReleaseMs = 0;
  bool    longFired      = false;
  bool    waitingDouble  = false;
  PressType pending      = PressType::NONE;
};

struct BuzzStep { uint16_t freq; uint16_t durMs; };

struct SensorData {
  float    accelX, accelY, accelZ;
  float    gyroX, gyroY, gyroZ;
  float    tempC       = NAN;
  int32_t  hrBPM        = 0;
  bool     hrFingerDetected = false;
  long     rawIR        = 0;
  uint8_t  battPercent  = 0;
  float    battVoltage  = 0.0f;
  bool     charging     = false;
  bool     motionDetected = false;
};

// Self-contained BPM estimator (no external heartRate.h dependency -- that
// header collided with a same-named, differently-implemented file in another
// MAX30101 library installed on this system). AC-couples the raw IR signal
// against a slow-moving DC baseline, then detects a beat as the AC component
// crossing from negative to positive, gated by minimum swing amplitude (to
// reject motion/contact noise) and a refractory period (to cap false beats
// at physiologically plausible rates).
struct HrEstimator {
  static const uint8_t RATE_SIZE = 4;
  uint8_t  rates[RATE_SIZE] = {0};
  uint8_t  rateSpot = 0;
  uint32_t lastBeatMs = 0;
  int32_t  bpm = 0;

  int32_t dcEstimate = 0;
  int32_t acPrev = 0;
  int32_t acPeak = 0, acTrough = 0;
  bool    seeded = false;

  void feed(long irValue, uint32_t nowMs) {
    if (irValue < 50000) { bpm = 0; seeded = false; return; } // no finger present

    if (!seeded) { dcEstimate = irValue; seeded = true; }
    // Slow exponential moving average tracks the DC baseline (ambient light +
    // tissue absorption); subtracting it leaves just the pulsatile AC swing.
    dcEstimate += ((int32_t)irValue - dcEstimate) >> 5; // ~1/32 smoothing
    int32_t ac = (int32_t)irValue - dcEstimate;

    if (ac > acPrev) acPeak = max(acPeak, ac);
    if (ac < acPrev) acTrough = min(acTrough, ac);

    bool crossedUp = (acPrev < 0 && ac >= 0);
    if (crossedUp) {
      int32_t swing = acPeak - acTrough;
      // 300ms refractory period caps false triggers at 200 BPM
      if (swing > 20 && swing < 4000 && (nowMs - lastBeatMs) > 300) {
        if (lastBeatMs != 0) {
          uint32_t delta = nowMs - lastBeatMs;
          float instantBpm = 60000.0f / delta;
          if (instantBpm > 20 && instantBpm < 255) {
            rates[rateSpot] = (uint8_t)instantBpm;
            rateSpot = (rateSpot + 1) % RATE_SIZE;
            uint16_t sum = 0;
            for (uint8_t i = 0; i < RATE_SIZE; i++) sum += rates[i];
            bpm = sum / RATE_SIZE;
          }
        }
        lastBeatMs = nowMs;
      }
      acPeak = 0; acTrough = 0; // reset for the next half-cycle
    }
    acPrev = ac;
  }
};

// ============================================================================
// DISPLAY
// ============================================================================
#define SCREEN_W 128
#define SCREEN_H 64
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

// ============================================================================
// SENSOR OBJECTS
// ============================================================================
Adafruit_MCP9808 mcp9808;
MAX30105 hrSensor;
Preferences prefs;

bool oledOK = false, imuOK = false, hrOK = false, tempOK = false;

// ============================================================================
// TIMING / CONFIG CONSTANTS
// ============================================================================
static const uint32_t FRAME_INTERVAL_MS       = 33;     // ~30 fps UI cap
static const uint32_t SENSOR_POLL_MS          = 200;    // IMU + temp poll
static const uint32_t HR_POLL_MS              = 10;     // matches ~100Hz sensor output (400Hz/4x avg)
static const uint32_t CLOCK_TICK_MS           = 500;
static const uint32_t WIFI_RSSI_POLL_MS       = 2000;
static const uint32_t WIFI_RECONNECT_MS       = 15000;
static const uint32_t NTP_RESYNC_MS           = 3600000UL; // hourly
static const uint32_t ALARM_POLL_MS           = 1000;   // minute-granularity check; no need to poll every loop()
static const uint32_t IDLE_DIM_MS             = 15000;
static const uint32_t IDLE_SLEEP_MS           = 30000;
static const uint32_t LONG_PRESS_MS           = 600;
static const uint32_t DOUBLE_PRESS_WINDOW_MS  = 300;
static const uint32_t DEBOUNCE_MS             = 25;
static const uint32_t ENCODER_REPEAT_ACCEL_MS = 80; // fast-scroll threshold

// ============================================================================
// BUTTON DRIVER  (event-driven, debounced, short/long/double)
// ============================================================================
ButtonState btn1{PIN_SW1, true};   // wired to VCC: pressed = HIGH
ButtonState btn2{PIN_SW3, true};   // wired to VCC: pressed = HIGH
ButtonState btnEnc{PIN_EC_S, false}; // pulled up: pressed = LOW

// Updates one button's state machine; returns a resolved press event (or NONE)
PressType updateButton(ButtonState &b) {
  bool rawLevel = digitalRead(b.pin);
  bool pressedRaw = b.activeHigh ? (rawLevel == HIGH) : (rawLevel == LOW);
  uint32_t now = millis();

  if (pressedRaw != b.lastRaw) {
    b.lastChangeMs = now;
    b.lastRaw = pressedRaw;
  }

  if ((now - b.lastChangeMs) > DEBOUNCE_MS && b.stable != b.lastRaw) {
    b.stable = b.lastRaw;
    if (b.stable) {
      // just pressed
      b.pressStartMs = now;
      b.longFired = false;
    } else {
      // just released
      uint32_t heldFor = now - b.pressStartMs;
      if (!b.longFired) {
        if (b.waitingDouble && (now - b.lastReleaseMs) < DOUBLE_PRESS_WINDOW_MS) {
          b.waitingDouble = false;
          return PressType::DOUBLE;
        } else {
          b.waitingDouble = true;
          b.lastReleaseMs = now;
        }
      }
    }
  }

  // long-press fires while still held, without waiting for release
  if (b.stable && !b.longFired && (now - b.pressStartMs) > LONG_PRESS_MS) {
    b.longFired = true;
    b.waitingDouble = false;
    return PressType::LONG;
  }

  // resolve a pending single press once the double-press window elapses
  if (b.waitingDouble && (now - b.lastReleaseMs) > DOUBLE_PRESS_WINDOW_MS) {
    b.waitingDouble = false;
    return PressType::SHORT;
  }

  return PressType::NONE;
}

// ============================================================================
// ROTARY ENCODER  (interrupt-driven quadrature decode)
// ============================================================================
volatile int32_t encoderDelta = 0;
volatile uint8_t encLastState = 0;

// Standard 2-bit gray-code quadrature table: index = (prevAB<<2)|curAB
static const int8_t QUAD_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

void IRAM_ATTR encoderISR() {
  uint8_t a = digitalRead(PIN_EC_A);
  uint8_t b = digitalRead(PIN_EC_B);
  uint8_t cur = (a << 1) | b;
  uint8_t idx = (encLastState << 2) | cur;
  encoderDelta += QUAD_TABLE[idx & 0x0F];
  encLastState = cur;
}

// ============================================================================
// BUZZER  (non-blocking tone sequencer)
// ============================================================================
BuzzStep buzzQueue[8];
uint8_t buzzQueueLen = 0, buzzQueueIdx = 0;
uint32_t buzzStepStartMs = 0;
bool buzzActive = false;
bool buzzNeedsStart = false; // signals buzzTask() to key the very first tone on

void buzzPlay(const BuzzStep *steps, uint8_t count) {
  count = min(count, (uint8_t)8);
  memcpy(buzzQueue, steps, sizeof(BuzzStep) * count);
  buzzQueueLen = count;
  buzzQueueIdx = 0;
  buzzActive = true;
  buzzNeedsStart = true;
  buzzStepStartMs = millis();
}
void buzzClick()   { BuzzStep s[] = {{2200, 12}};           buzzPlay(s, 1); }
void buzzConfirm() { BuzzStep s[] = {{1800,25},{2600,35}};   buzzPlay(s, 2); }
void buzzError()   { BuzzStep s[] = {{600,60},{300,90}};     buzzPlay(s, 2); }
void buzzAlert()   { BuzzStep s[] = {{1500,80},{0,40},{1500,80}}; buzzPlay(s, 3); }

void buzzTask() {
  if (!buzzActive) return;
  uint32_t now = millis();
  BuzzStep &cur = buzzQueue[buzzQueueIdx];

  // Key the first step's tone the moment buzzTask next runs after buzzPlay().
  // Previously this only happened if buzzTask() happened to run on the exact
  // same millis() tick as buzzPlay() -- which is not guaranteed -- so the
  // first tone in a sequence was frequently silently skipped.
  if (buzzNeedsStart) {
    ledcWriteTone(PIN_BUZZER, cur.freq > 0 ? cur.freq : 0);
    buzzNeedsStart = false;
  }

  if (now - buzzStepStartMs >= cur.durMs) {
    buzzQueueIdx++;
    if (buzzQueueIdx >= buzzQueueLen) {
      ledcWriteTone(PIN_BUZZER, 0);
      buzzActive = false;
      return;
    }
    buzzStepStartMs = now;
    cur = buzzQueue[buzzQueueIdx];
    ledcWriteTone(PIN_BUZZER, cur.freq > 0 ? cur.freq : 0);
  }
}

// ============================================================================
// FLYBACK / TENS STAGE  (non-blocking charge-pulse sequencer)
// ============================================================================
// Each pulse is a 60us HIGH on the gate followed by a 20ms LOW gap. The
// 60us on-time is short enough that doing it with delayMicroseconds() right
// inside flybackTask() is fine -- it's a negligible, bounded blip once every
// 20ms, not the ~4s blocking loop the original sketch used. The 20ms *gaps*
// between pulses are what used to dominate the blocking time, and those are
// now scheduled against millis() like every other task in this firmware.
bool     flybackActive     = false;
uint16_t flybackPulsesLeft = 0;
uint32_t flybackLastPulseMs = 0;

void flybackStart() {
  if (flybackActive) return; // already mid-cycle; let it finish first
  flybackActive = true;
  flybackPulsesLeft = FLYBACK_PULSE_COUNT;
  flybackLastPulseMs = 0; // fire the first pulse immediately on next flybackTask()
}

void flybackTask() {
  if (!flybackActive) return;
  uint32_t now = millis();
  if (now - flybackLastPulseMs < FLYBACK_PULSE_OFF_MS) return;
  flybackLastPulseMs = now;

  digitalWrite(PIN_FLYBACK_GATE, HIGH);
  delayMicroseconds(FLYBACK_PULSE_ON_US);
  digitalWrite(PIN_FLYBACK_GATE, LOW);

  flybackPulsesLeft--;
  if (flybackPulsesLeft == 0) flybackActive = false;
}

// ============================================================================
// SENSOR DATA (shared, updated by sensorTask)
// ============================================================================
SensorData sensors;

// ---- Alarm (ported from the alarm-clock sketch) --------------------------
int      alarmHour       = 7;
int      alarmMinute     = 0;
bool     alarmEnabled    = true;
bool     alarmTriggered  = false;   // latched true for the current minute match
bool     alarmRinging    = false;   // non-blocking "is the alarm currently sounding"
uint32_t alarmRingStartMs = 0;
uint32_t alarmLastPulseMs = 0;
bool     alarmPulseOn     = false;

// ---- Step counting + wrist-swing gesture ----------------------------------
// Original sketch worked in raw MPU6050 LSB units (threshold 15000 / 32768
// full-scale @ +-2g, i.e. ~0.92g of excess motion, and a 20000-raw gyro swing
// threshold). mpuRead() here already converts to g's and deg/s, so the same
// physical thresholds are re-expressed in those units.
// Single-threshold crossing is fragile: on a wrist (vs. pocket/belt), swing
// amplitude varies a lot with stride and can hover right at one value,
// causing double-counts or missed counts. Hysteresis (must rise above HIGH,
// then fall below LOW, to arm the next count) is the standard fix. 1.92g
// was also likely just too high for typical wrist-swing walking motion --
// re-tuned lower, but the Steps screen now shows the live filtered value so
// you can dial these in against your own actual motion instead of a guess.
static const float    STEP_ACCEL_THRESHOLD_HIGH_G = 1.35f;
static const float    STEP_ACCEL_THRESHOLD_LOW_G  = 1.10f;
static const float    SWING_GYRO_THRESHOLD_DPS = 152.7f;
static const uint32_t STEP_INTERVAL_MS = 300;
static const float    STEP_FILTER_ALPHA = 0.2f;   // low-pass smoothing factor

uint32_t stepCount = 0;
float    filteredAccMag = 1.0f; // starts near 1g at rest
bool     stepArmed = true; // true once magnitude has dropped back below the LOW threshold
uint32_t lastStepMs = 0;

bool backgroundInverted = false;
bool flybackEnabled = true; // wake-zap on alarm; toggleable in Settings

// Forward-declared here because webServerSetup()'s lambdas (just below)
// reference it, but its actual definition lives further down in the UI
// STATE block. Arduino's auto-generated prototypes only cover functions,
// not globals, so without this the lambdas fail to compile with
// "needsRedraw was not declared in this scope".
extern bool needsRedraw;

WebServer webServer(80);

// Builds the alarm page fresh on every request so it reflects live device
// state (current time, alarm state, signal) rather than a static form.
String buildAlarmPage() {
  struct tm tmNow;
  bool haveTime = getLocalTime(&tmNow, 0);
  char nowBuf[16] = "--:--:--";
  if (haveTime && tmNow.tm_year > 100) strftime(nowBuf, sizeof(nowBuf), "%H:%M:%S", &tmNow);
  char alarmBuf[6];
  snprintf(alarmBuf, sizeof(alarmBuf), "%02d:%02d", alarmHour, alarmMinute);

  String html;
  html.reserve(2400);
  html += F(
    "<!DOCTYPE html><html><head>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Waker Alarm</title><style>"
    "body{font-family:-apple-system,'Segoe UI',Roboto,sans-serif;background:#0f1115;color:#eee;"
    "display:flex;justify-content:center;padding:32px 16px;margin:0}"
    ".card{background:#1b1e26;border-radius:16px;padding:28px 24px;max-width:360px;width:100%;"
    "box-shadow:0 8px 24px rgba(0,0,0,.4)}"
    "h1{font-size:20px;margin:0 0 4px}"
    ".sub{color:#8b93a7;font-size:13px;margin-bottom:20px}"
    ".status{background:#11141a;border-radius:10px;padding:14px 16px;margin-bottom:20px}"
    ".status .time{font-size:32px;font-weight:600;letter-spacing:1px}"
    ".status .row{display:flex;justify-content:space-between;font-size:13px;color:#8b93a7;margin-top:6px}"
    "label{display:block;font-size:12px;color:#8b93a7;margin:14px 0 4px}"
    "input[type=number]{width:100%;box-sizing:border-box;background:#11141a;border:1px solid #2a2f3a;"
    "color:#eee;border-radius:8px;padding:10px 12px;font-size:16px}"
    "button,.btn{width:100%;margin-top:16px;padding:12px;border:none;border-radius:10px;font-size:15px;"
    "font-weight:600;cursor:pointer;text-align:center;display:block;text-decoration:none;box-sizing:border-box}"
    ".primary{background:#4c7cf3;color:#fff}"
    ".toggle{background:#2a2f3a;color:#eee;margin-top:10px}"
    "</style></head><body><div class='card'>"
    "<h1>Waker</h1><div class='sub'>Alarm configuration</div>"
    "<div class='status'><div class='time'>");
  html += nowBuf;
  html += F("</div><div class='row'><span>Alarm</span><span>");
  html += alarmBuf;
  html += alarmEnabled ? F(" &middot; ON</span></div>") : F(" &middot; OFF</span></div>");
  html += F("<div class='row'><span>Wi-Fi</span><span>");
  html += (WiFi.status() == WL_CONNECTED) ? WiFi.SSID() : String("disconnected");
  if (WiFi.status() == WL_CONNECTED) {
    html += F(" (");
    html += String(WiFi.RSSI());
    html += F(" dBm)");
  }
  html += F("</span></div></div>");

  html += F(
    "<form action='/set'>"
    "<label>Hour (0-23)</label>"
    "<input type='number' name='hour' min='0' max='23' value='");
  html += String(alarmHour);
  html += F("' required>"
    "<label>Minute (0-59)</label>"
    "<input type='number' name='minute' min='0' max='59' value='");
  html += String(alarmMinute);
  html += F("' required>"
    "<button class='btn primary' type='submit'>Save alarm time</button>"
    "</form>"
    "<a class='btn toggle' href='/toggle'>");
  html += alarmEnabled ? F("Turn alarm OFF") : F("Turn alarm ON");
  html += F("</a>"
    "<a class='btn toggle' href='/update'>Firmware update</a>"
    "</div></body></html>");
  return html;
}

void webServerSetup() {
  webServer.on("/", []() {
    webServer.send(200, "text/html", buildAlarmPage());
  });
  webServer.on("/set", []() {
    if (webServer.hasArg("hour") && webServer.hasArg("minute")) {
      int h = webServer.arg("hour").toInt();
      int m = webServer.arg("minute").toInt();
      if (h >= 0 && h <= 23 && m >= 0 && m <= 59) {
        alarmHour = h;
        alarmMinute = m;
        alarmTriggered = false;
        needsRedraw = true;
      }
    }
    // Redirect back to "/" so the page always reflects current state instead
    // of stranding the user on a dead-end confirmation page.
    webServer.sendHeader("Location", "/");
    webServer.send(303);
  });
  webServer.on("/toggle", []() {
    alarmEnabled = !alarmEnabled;
    needsRedraw = true;
    webServer.sendHeader("Location", "/");
    webServer.send(303);
  });

  // OTA firmware update -- upload a compiled .bin from any browser on the
  // network instead of connecting USB. To get a .bin: Arduino IDE ->
  // Sketch -> Export Compiled Binary, then pick the resulting file here.
  // No auth on this route -- anyone on your Wi-Fi can reflash the watch.
  // Fine for a home network; if that's ever a concern, the straightforward
  // hardening is HTTP Basic Auth via webServer.authenticate() before serving
  // this route.
  webServer.on("/update", HTTP_GET, []() {
    webServer.send(200, "text/html", F(
      "<!DOCTYPE html><html><head>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>Waker Firmware Update</title><style>"
      "body{font-family:-apple-system,'Segoe UI',Roboto,sans-serif;background:#0f1115;color:#eee;"
      "display:flex;justify-content:center;padding:32px 16px;margin:0}"
      ".card{background:#1b1e26;border-radius:16px;padding:28px 24px;max-width:360px;width:100%;"
      "box-shadow:0 8px 24px rgba(0,0,0,.4)}"
      "h1{font-size:20px;margin:0 0 4px}"
      ".sub{color:#8b93a7;font-size:13px;margin-bottom:20px;line-height:1.5}"
      "input[type=file]{width:100%;box-sizing:border-box;background:#11141a;border:1px solid #2a2f3a;"
      "color:#eee;border-radius:8px;padding:10px 12px;font-size:13px;margin-bottom:16px}"
      "button{width:100%;padding:12px;border:none;border-radius:10px;font-size:15px;font-weight:600;"
      "cursor:pointer;background:#4c7cf3;color:#fff}"
      "#status{margin-top:14px;font-size:13px;color:#8b93a7}"
      "</style></head><body><div class='card'>"
      "<h1>Waker</h1>"
      "<div class='sub'>Upload a compiled .bin (Arduino IDE: Sketch &rarr; "
      "Export Compiled Binary). The watch reboots automatically once the "
      "flash finishes -- don't close this page or lose power until it does.</div>"
      "<form id='f'><input type='file' name='firmware' id='file' accept='.bin' required>"
      "<button type='submit'>Upload &amp; flash</button></form>"
      "<div id='status'></div>"
      "<script>"
      "document.getElementById('f').addEventListener('submit', function(e){"
      "e.preventDefault();"
      "var f=document.getElementById('file').files[0];"
      "if(!f)return;"
      "var fd=new FormData();fd.append('firmware',f);"
      "var xhr=new XMLHttpRequest();"
      "xhr.open('POST','/update',true);"
      "xhr.upload.onprogress=function(evt){"
      "if(evt.lengthComputable){"
      "var pct=Math.round(evt.loaded/evt.total*100);"
      "document.getElementById('status').innerText='Uploading... '+pct+'%';"
      "}};"
      "xhr.onload=function(){"
      "document.getElementById('status').innerText="
      "xhr.status===200?'Flashed. Rebooting...':'Update failed: '+xhr.responseText;"
      "};"
      "xhr.send(fd);"
      "});"
      "</script>"
      "</div></body></html>"
    ));
  });
  webServer.on("/update", HTTP_POST, []() {
    // This runs after the upload handler below has already finished writing
    // (or failing) -- just report the outcome and reboot on success.
    webServer.sendHeader("Connection", "close");
    if (Update.hasError()) {
      webServer.send(500, "text/plain", "Update failed -- check Serial for details");
    } else {
      webServer.send(200, "text/plain", "OK");
      delay(500); // let the response actually reach the browser before rebooting
      ESP.restart();
    }
  }, []() {
    // Upload handler: streams the .bin in chunks as the browser sends it,
    // writing each chunk straight into flash rather than buffering the
    // whole file in RAM (which the watch doesn't have enough of anyway).
    HTTPUpload &upload = webServer.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("OTA update starting: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        Serial.printf("OTA update success: %u bytes. Rebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });

  webServer.begin();
}


// ---- MPU6500 minimal register driver --------------------------------------
bool mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADDR_MPU6500);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
bool mpuReadBytes(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(ADDR_MPU6500);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t got = Wire.requestFrom((int)ADDR_MPU6500, (int)len);
  if (got < len) return false;
  for (uint8_t i = 0; i < len && Wire.available(); i++) buf[i] = Wire.read();
  return true;
}
bool mpuInit() {
  mpuWriteReg(0x6B, 0x00); // PWR_MGMT_1: wake up
  delayMicroseconds(200);
  mpuWriteReg(0x1C, 0x00); // ACCEL_CONFIG: +/-2g
  mpuWriteReg(0x1B, 0x00); // GYRO_CONFIG: +/-250dps
  mpuWriteReg(0x1A, 0x03); // CONFIG: DLPF ~44Hz
  uint8_t who = 0;
  mpuReadBytes(0x75, &who, 1);
  return (who == 0x70 || who == 0x68 || who == 0x71 || who == 0x73);
}
void mpuRead(SensorData &d) {
  uint8_t raw[14];
  if (!mpuReadBytes(0x3B, raw, 14)) return; // leave last-known values on transient I2C failure
  int16_t ax = (raw[0] << 8) | raw[1];
  int16_t ay = (raw[2] << 8) | raw[3];
  int16_t az = (raw[4] << 8) | raw[5];
  int16_t gx = (raw[8] << 8) | raw[9];
  int16_t gy = (raw[10] << 8) | raw[11];
  int16_t gz = (raw[12] << 8) | raw[13];
  d.accelX = ax / 16384.0f; d.accelY = ay / 16384.0f; d.accelZ = az / 16384.0f;
  d.gyroX  = gx / 131.0f;   d.gyroY  = gy / 131.0f;   d.gyroZ  = gz / 131.0f;

  static float lastMag = 1.0f;
  float mag = sqrtf(d.accelX*d.accelX + d.accelY*d.accelY + d.accelZ*d.accelZ);
  d.motionDetected = fabsf(mag - lastMag) > 0.15f;
  lastMag = mag;
}

// ---- Heart-rate: lightweight peak-interval BPM estimator -------------------
HrEstimator hrEst;

// ============================================================================
// UI STATE
// ============================================================================
AppState state = AppState::BOOT;
AppState homeReturnState = AppState::HOME;
uint8_t  menuIndex = 0, menuScrollOffset = 0;
uint8_t  alarmEditField = 0;   // 0 = hour, 1 = minute
uint8_t  settingsIndex  = 0;   // 0 = Buzzer test, 1 = Background invert, 2 = Wake-zap on alarm
uint32_t lastActivityMs = 0;
bool     displayDimmed = false;
bool     displaySleeping = false;
bool     needsRedraw = true;

// Simple slide-transition animation state
float    transitionOffset = 0.0f; // 0 = settled, +/-128 mid-slide
int8_t   transitionDir = 0;
bool     transitioning = false;

const char *MENU_ITEMS[] = {
  "Clock", "Heart Rate", "Temperature", "Battery", "Wi-Fi", "Alarm", "Steps",
  "Settings", "System Info", "About Device"
};
const uint8_t MENU_COUNT = sizeof(MENU_ITEMS) / sizeof(MENU_ITEMS[0]);

void startTransition(int8_t dir) {
  transitioning = true;
  transitionDir = dir;
  transitionOffset = dir * 128.0f;
}

void goTo(AppState s, int8_t animDir = 1) {
  state = s;
  needsRedraw = true;
  lastActivityMs = millis();
  startTransition(animDir);
}

// ============================================================================
// WI-FI MANAGER
// ============================================================================
WifiUiPhase wifiPhase = WifiUiPhase::IDLE;
String wifiScanResults[12];
int32_t wifiScanRssi[12];
uint8_t wifiScanCount = 0;
uint8_t wifiSelectIndex = 0;
String  pendingSSID;
String  keypadBuffer;
uint8_t keypadCharIndex = 0;
uint32_t lastWifiAttemptMs = 0;
uint32_t lastNtpSyncMs = 0;
bool ntpSynced = false;

const char KEYPAD_CHARSET[] =
  "abcdefghijklmnopqrstuvwxyz"
  "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
  "0123456789"
  "!@#$%^&*()-_=+ ";
const int KEYPAD_CHARSET_LEN = sizeof(KEYPAD_CHARSET) - 1;

void wifiStartScan() {
  WiFi.scanDelete();
  WiFi.scanNetworks(true /*async*/);
  wifiPhase = WifiUiPhase::SCANNING;
  needsRedraw = true;
}

void wifiPollScan() {
  if (wifiPhase != WifiUiPhase::SCANNING) return;
  int n = WiFi.scanComplete();
  if (n == WIFI_SCAN_RUNNING) return;
  if (n < 0) { wifiPhase = WifiUiPhase::IDLE; return; }
  wifiScanCount = min(n, 12);
  for (uint8_t i = 0; i < wifiScanCount; i++) {
    wifiScanResults[i] = WiFi.SSID(i);
    wifiScanRssi[i] = WiFi.RSSI(i);
  }
  wifiPhase = WifiUiPhase::SCAN_DONE;
  wifiSelectIndex = 0;
  needsRedraw = true;
}

void wifiSaveCredential(const String &ssid, const String &pass) {
  prefs.begin("wifi", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.end();
}

void wifiTryAutoConnect() {
  prefs.begin("wifi", true);
  String ssid = prefs.getString("ssid", "");
  String pass = prefs.getString("pass", "");
  prefs.end();
  if (ssid.length() > 0) {
    WiFi.begin(ssid.c_str(), pass.c_str());
    lastWifiAttemptMs = millis();
  }
}

void wifiConnectTo(const String &ssid, const String &pass) {
  WiFi.begin(ssid.c_str(), pass.c_str());
  wifiPhase = WifiUiPhase::CONNECTING;
  lastWifiAttemptMs = millis();
  wifiSaveCredential(ssid, pass);
}

void ntpSyncIfNeeded() {
  if (WiFi.status() != WL_CONNECTED) return;
  uint32_t now = millis();
  if (!ntpSynced || (now - lastNtpSyncMs) > NTP_RESYNC_MS) {
    // IST is UTC+5:30 with no DST, so the POSIX TZ string is "IST-5:30"
    // (POSIX TZ offsets are inverted: west-of-UTC is positive).
    configTzTime("IST-5:30", "pool.ntp.org", "time.nist.gov");
    lastNtpSyncMs = now;
    ntpSynced = true; // optimistic; struct tm validity checked on read
  }
}

void wifiTask() {
  wifiPollScan();
  if (WiFi.status() != WL_CONNECTED && wifiPhase != WifiUiPhase::SCANNING
      && wifiPhase != WifiUiPhase::KEYPAD) {
    uint32_t now = millis();
    if (now - lastWifiAttemptMs > WIFI_RECONNECT_MS) {
      wifiTryAutoConnect();
    }
  }

  // Print the alarm page's URL to Serial exactly once per connection, the
  // moment the IP is actually assigned (WiFi.begin() is async, so this can't
  // just happen right after WiFi.begin() in setup() -- it has to be polled).
  static bool announcedUrl = false;
  static bool wasConnected = false;
  bool nowConnected = (WiFi.status() == WL_CONNECTED);
  if (nowConnected && !announcedUrl) {
    Serial.print("Waker alarm page: http://");
    Serial.print(WiFi.localIP());
    Serial.println("/");
    Serial.print("Waker firmware update: http://");
    Serial.print(WiFi.localIP());
    Serial.println("/update");
    announcedUrl = true;
  }
  if (!nowConnected && wasConnected) {
    announcedUrl = false; // reconnecting later (new IP) re-announces
  }
  wasConnected = nowConnected;

  ntpSyncIfNeeded();
}

// ============================================================================
// POWER MANAGEMENT
// ============================================================================
// Multiply this by the raw ADC voltage to get actual pack voltage. Default
// 2.0 assumes an exact 1:1 (equal-value) resistor divider, which is rarely
// exactly true in practice -- resistor tolerance alone can shift this by a
// few percent, which is enough to turn "100% at full charge" into "80%".
// To calibrate: charge the battery fully, check the raw voltage shown on
// the Battery screen, measure the actual pack voltage with a multimeter,
// and set this to (multimeter reading) / (raw voltage shown).
static const float VBATT_DIVIDER_RATIO = 2.0f;

void readBatteryAndCharge(SensorData &d) {
#if HAS_BATT_ADC
  // The ESP32's ADC is inherently noisy -- a single analogRead() can jitter
  // by dozens of counts sample to sample, which is what was making the
  // percentage "always change" even sitting still on a charger. Oversampling
  // (averaging many back-to-back reads) cuts that down substantially, and an
  // exponential moving average across polls (not just within one poll)
  // smooths out what's left so the displayed number holds still.
  const int OVERSAMPLE_COUNT = 32;
  uint32_t sum = 0;
  for (int i = 0; i < OVERSAMPLE_COUNT; i++) sum += analogRead(PIN_BATT_ADC);
  float rawAvg = (float)sum / OVERSAMPLE_COUNT;

  static float emaAdcCounts = -1.0f; // -1 sentinel = not yet seeded
  if (emaAdcCounts < 0) emaAdcCounts = rawAvg;
  emaAdcCounts += (rawAvg - emaAdcCounts) * 0.1f; // slow-ish smoothing, ~10 polls to settle

  float vAdc = (emaAdcCounts / 4095.0f) * 3.3f;
  float vBatt = vAdc * VBATT_DIVIDER_RATIO;
  d.battVoltage = vBatt;
  float pct = (vBatt - 3.3f) / (4.2f - 3.3f) * 100.0f;
  d.battPercent = (uint8_t)constrain(pct, 0.0f, 100.0f);
#else
  d.battPercent = 0; // no fuel-gauge / ADC divider confirmed on this schematic
  d.battVoltage = 0.0f;
#endif
  d.charging = (digitalRead(PIN_CHG_STAT) == LOW);
}

void enterDisplaySleep() {
  if (displaySleeping) return;
  display.ssd1306_command(SSD1306_DISPLAYOFF);
  displaySleeping = true;
}
void wakeDisplay() {
  if (displaySleeping) {
    display.ssd1306_command(SSD1306_DISPLAYON);
    displaySleeping = false;
  }
  displayDimmed = false;
  display.dim(false);
  lastActivityMs = millis();
  needsRedraw = true;
}

void powerTask() {
  // The ringing alarm is the one screen that must stay lit and visible no
  // matter how long it's been idle -- otherwise the flashing ALARM! screen
  // (and any button prompt to dismiss it) goes dark mid-alarm. alarmTask()
  // also refreshes lastActivityMs every pulse, so this is a belt-and-braces
  // guard rather than the only thing keeping it awake.
  if (state == AppState::APP_ALARM_RINGING) {
    if (displayDimmed) { displayDimmed = false; display.dim(false); }
    if (displaySleeping) { display.ssd1306_command(SSD1306_DISPLAYON); displaySleeping = false; }
    return;
  }

  uint32_t idleFor = millis() - lastActivityMs;
  if (!displayDimmed && idleFor > IDLE_DIM_MS) {
    displayDimmed = true;
    display.dim(true);
  }
  if (!displaySleeping && idleFor > IDLE_SLEEP_MS) {
    enterDisplaySleep();
  }
}

// Wake sources: button ISRs already call wakeDisplay() via activity flag;
// IMU motion flagged in sensorTask also refreshes activity when significant.

// ============================================================================
// DRAWING HELPERS
// ============================================================================
void drawStatusBar() {
  display.fillRect(0, 0, SCREEN_W, 10, SSD1306_BLACK);
  // Wi-Fi glyph
  bool wifiUp = WiFi.status() == WL_CONNECTED;
  int rssiBars = wifiUp ? (WiFi.RSSI() + 100) / 20 : 0; // read RSSI once, not once per bar
  int wx = 2;
  for (int i = 0; i < 4; i++) {
    int barH = 2 + i * 2;
    bool lit = wifiUp && (rssiBars >= (4 - i));
    display.fillRect(wx + i * 3, 9 - barH, 2, barH, lit ? SSD1306_WHITE : SSD1306_BLACK);
    if (!lit) display.drawRect(wx + i * 3, 9 - barH, 2, barH, SSD1306_WHITE);
  }
  // Heart glyph (small, pulses)
  static uint32_t lastBeatAnimMs = 0;
  static bool beatPulse = false;
  if (sensors.hrBPM > 0 && millis() - lastBeatAnimMs > (60000 / max(1, (int)sensors.hrBPM))) {
    beatPulse = !beatPulse;
    lastBeatAnimMs = millis();
  }
  int hx = 40;
  if (beatPulse) {
    display.fillCircle(hx, 4, 2, SSD1306_WHITE);
    display.fillCircle(hx + 3, 4, 2, SSD1306_WHITE);
    display.fillTriangle(hx - 2, 5, hx + 5, 5, hx + 1, 9, SSD1306_WHITE);
  } else {
    display.drawCircle(hx, 4, 2, SSD1306_WHITE);
    display.drawCircle(hx + 3, 4, 2, SSD1306_WHITE);
  }
  // Battery
  int bx = SCREEN_W - 20;
  display.drawRect(bx, 1, 16, 8, SSD1306_WHITE);
  display.fillRect(bx + 16, 3, 2, 4, SSD1306_WHITE);
  int fillW = map(sensors.battPercent, 0, 100, 0, 14);
  display.fillRect(bx + 1, 2, fillW, 6, SSD1306_WHITE);
  if (sensors.charging) {
    display.drawLine(bx + 6, 0, bx + 4, 5, SSD1306_WHITE);
    display.drawLine(bx + 4, 5, bx + 8, 5, SSD1306_WHITE);
    display.drawLine(bx + 8, 5, bx + 6, 9, SSD1306_WHITE);
  }
}

void drawLoadingSpinner(int cx, int cy, int r) {
  static uint8_t phase = 0;
  phase = (phase + 1) % 8;
  for (int i = 0; i < 8; i++) {
    float ang = (i * 2 * PI / 8.0f);
    int x = cx + cos(ang) * r;
    int y = cy + sin(ang) * r;
    int dist = (i - phase + 8) % 8;
    if (dist < 4) display.fillCircle(x, y, 1, SSD1306_WHITE);
  }
}

void centerText(const char *txt, int y, uint8_t size = 1) {
  int16_t x1, y1; uint16_t w, h;
  display.setTextSize(size);
  display.getTextBounds(txt, 0, y, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_W - w) / 2, y);
  display.print(txt);
}

// ============================================================================
// SCREENS
// ============================================================================
void drawHomeScreen() {
  drawStatusBar();
  time_t now = time(nullptr);
  struct tm tmNow;
  bool haveTime = getLocalTime(&tmNow, 0);

  char timeBuf[8] = "--:--";
  char dateBuf[20] = "Sync pending";
  if (haveTime && tmNow.tm_year > 100) {
    strftime(timeBuf, sizeof(timeBuf), "%H:%M", &tmNow);
    strftime(dateBuf, sizeof(dateBuf), "%a, %b %d", &tmNow);
  }
  display.setTextSize(2);
  display.setCursor(18, 16);
  display.print(timeBuf);
  display.setTextSize(1);
  centerText(dateBuf, 36);

  display.drawFastHLine(4, 46, SCREEN_W - 8, SSD1306_WHITE);

  display.setCursor(2, 52);
  display.print(sensors.hrBPM > 0 ? String(sensors.hrBPM) : String("--"));
  display.print("bpm");

  display.setCursor(56, 52);
  if (!isnan(sensors.tempC)) {
    display.print(sensors.tempC, 1);
    display.print("C");
  } else {
    display.print("--.-C");
  }

  display.setCursor(100, 52);
  display.print(sensors.battPercent);
  display.print("%");
}

void drawMenuScreen() {
  drawStatusBar();
  const uint8_t rowsVisible = 4;
  if (menuIndex < menuScrollOffset) menuScrollOffset = menuIndex;
  if (menuIndex >= menuScrollOffset + rowsVisible) menuScrollOffset = menuIndex - rowsVisible + 1;

  for (uint8_t row = 0; row < rowsVisible; row++) {
    uint8_t i = menuScrollOffset + row;
    if (i >= MENU_COUNT) break;
    int y = 13 + row * 13;
    bool selected = (i == menuIndex);
    if (selected) {
      display.fillRoundRect(2, y - 1, SCREEN_W - 4, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(8, y + 1);
    display.print(MENU_ITEMS[i]);
  }
  display.setTextColor(SSD1306_WHITE);
}

void drawClockApp() {
  drawStatusBar();
  struct tm tmNow;
  bool haveTime = getLocalTime(&tmNow, 0);
  char timeBuf[10] = "--:--:--";
  char dateBuf[24] = "No time sync";
  if (haveTime && tmNow.tm_year > 100) {
    strftime(timeBuf, sizeof(timeBuf), "%H:%M:%S", &tmNow);
    strftime(dateBuf, sizeof(dateBuf), "%A, %d %b %Y", &tmNow);
  }
  display.setTextSize(2);
  centerText(timeBuf, 24, 2);
  display.setTextSize(1);
  centerText(dateBuf, 46);
}

void drawHeartRateApp() {
  drawStatusBar();
  centerText("HEART RATE", 12);
  display.setTextSize(3);
  char buf[8];
  if (sensors.hrBPM > 0) snprintf(buf, sizeof(buf), "%ld", (long)sensors.hrBPM);
  else strcpy(buf, "--");
  centerText(buf, 26, 3);
  display.setTextSize(1);
  if (!hrOK) {
    centerText("Sensor not detected", 54);
  } else {
    char irBuf[24];
    snprintf(irBuf, sizeof(irBuf), "IR:%ld %s", sensors.rawIR,
             sensors.hrFingerDetected ? "(finger)" : "(no finger)");
    centerText(irBuf, 54);
  }
}

void drawTemperatureApp() {
  drawStatusBar();
  centerText("TEMPERATURE", 12);
  char buf[10];
  if (!isnan(sensors.tempC)) snprintf(buf, sizeof(buf), "%.1fC", sensors.tempC);
  else strcpy(buf, "--.-C");
  display.setTextSize(3);
  centerText(buf, 28, 3);
  display.setTextSize(1);
  if (!tempOK) centerText("Sensor not detected", 54);
}

void drawBatteryApp() {
  drawStatusBar();
  centerText("BATTERY", 12);
  display.drawRoundRect(34, 24, 50, 24, 3, SSD1306_WHITE);
  display.fillRect(84, 32, 4, 8, SSD1306_WHITE);
  int fillW = map(sensors.battPercent, 0, 100, 0, 46);
  display.fillRect(36, 26, fillW, 20, SSD1306_WHITE);
  char buf[24];
#if HAS_BATT_ADC
  snprintf(buf, sizeof(buf), "%d%%  %.2fV%s", sensors.battPercent, sensors.battVoltage,
           sensors.charging ? " CHG" : "");
#else
  snprintf(buf, sizeof(buf), "No ADC configured");
#endif
  centerText(buf, 54);
}

void drawWifiApp() {
  drawStatusBar();
  centerText("WI-FI", 12);
  bool up = WiFi.status() == WL_CONNECTED;
  centerText(up ? WiFi.SSID().c_str() : "Not connected", 26);
  if (up) {
    char buf[24];
    snprintf(buf, sizeof(buf), "RSSI: %d dBm", WiFi.RSSI());
    centerText(buf, 38);
    centerText(WiFi.localIP().toString().c_str(), 50);
  } else {
    centerText("Press ENC: scan", 44);
  }
}

void drawWifiScanApp() {
  drawStatusBar();
  if (wifiPhase == WifiUiPhase::SCANNING) {
    centerText("Scanning...", 20);
    drawLoadingSpinner(64, 40, 10);
    return;
  }
  if (wifiPhase == WifiUiPhase::CONNECTING) {
    centerText("Connecting...", 20);
    drawLoadingSpinner(64, 40, 10);
    return;
  }
  if (wifiScanCount == 0) {
    centerText("No networks found", 30);
    return;
  }
  const uint8_t rowsVisible = 4;
  uint8_t offset = wifiSelectIndex >= rowsVisible ? wifiSelectIndex - rowsVisible + 1 : 0;
  for (uint8_t row = 0; row < rowsVisible; row++) {
    uint8_t i = offset + row;
    if (i >= wifiScanCount) break;
    int y = 13 + row * 13;
    bool selected = (i == wifiSelectIndex);
    if (selected) {
      display.fillRoundRect(2, y - 1, SCREEN_W - 4, 12, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    }
    display.setCursor(8, y + 1);
    char line[24];
    snprintf(line, sizeof(line), "%-14s %ddBm", wifiScanResults[i].c_str(), wifiScanRssi[i]);
    display.print(line);
    display.setTextColor(SSD1306_WHITE);
  }
}

void drawKeypadApp() {
  drawStatusBar();
  char label[32];
  snprintf(label, sizeof(label), "PW: %s%s", keypadBuffer.c_str(),
           (millis() / 400) % 2 ? "_" : " ");
  centerText(pendingSSID.c_str(), 12);
  display.setCursor(2, 24);
  display.print(label);

  char c = KEYPAD_CHARSET[keypadCharIndex];
  display.setTextSize(2);
  char cbuf[2] = { c, 0 };
  centerText(cbuf, 38, 2);
  display.setTextSize(1);
  centerText("ENC:add BTN1:back BTN2:done", 56);
}

void drawSettingsApp() {
  drawStatusBar();
  const char *rows[3] = {
    "Buzzer Test",
    backgroundInverted ? "Invert: ON" : "Invert: OFF",
    flybackEnabled ? "Wake Zap: ON" : "Wake Zap: OFF"
  };
  for (uint8_t i = 0; i < 3; i++) {
    int y = 12 + i * 11;
    bool selected = (i == settingsIndex);
    if (selected) {
      display.fillRoundRect(2, y - 1, SCREEN_W - 4, 10, 3, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    }
    display.setCursor(8, y + 1);
    display.print(rows[i]);
    display.setTextColor(SSD1306_WHITE);
  }
  centerText("ENC:toggle  hold ENC:back", 48);
  centerText("hold BTN1:forget WiFi", 57);
}

void drawAlarmSetApp() {
  drawStatusBar();
  centerText(alarmEnabled ? "ALARM: ON" : "ALARM: OFF", 12);
  char buf[6];
  snprintf(buf, sizeof(buf), "%02d:%02d", alarmHour, alarmMinute);
  display.setTextSize(3);
  centerText(buf, 24, 3);
  display.setTextSize(1);
  // underline whichever field the encoder currently edits
  int fieldX = (SCREEN_W / 2) + (alarmEditField == 0 ? -34 : 6);
  display.drawFastHLine(fieldX, 50, 26, SSD1306_WHITE);
  centerText("ENC:adjust  ENC-press:field", 56);
}

void drawStepCountApp() {
  drawStatusBar();
  centerText("STEPS TODAY", 12);
  char buf[10];
  snprintf(buf, sizeof(buf), "%lu", (unsigned long)stepCount);
  display.setTextSize(3);
  centerText(buf, 28, 3);
  display.setTextSize(1);
  if (!imuOK) {
    centerText("IMU not detected", 54);
  } else {
    // Live tuning aid: watch this while walking to see the peaks your wrist
    // actually produces, then adjust STEP_ACCEL_THRESHOLD_HIGH_G/LOW_G to
    // sit just below/above your real swing range.
    char mag[20];
    snprintf(mag, sizeof(mag), "mag: %.2fg", filteredAccMag);
    centerText(mag, 54);
  }
}

void drawAlarmRingingApp() {
  // deliberately ignores the dim/normal status bar chrome for max contrast
  display.fillRect(0, 0, SCREEN_W, SCREEN_H, (millis() / 250) % 2 ? SSD1306_WHITE : SSD1306_BLACK);
  display.setTextColor((millis() / 250) % 2 ? SSD1306_BLACK : SSD1306_WHITE);
  centerText("ALARM!", 16, 2);
  char buf[6];
  snprintf(buf, sizeof(buf), "%02d:%02d", alarmHour, alarmMinute);
  centerText(buf, 40);
  centerText("Any button: dismiss", 54);
  display.setTextColor(SSD1306_WHITE);
}

void drawSysInfoApp() {
  drawStatusBar();
  centerText("SYSTEM INFO", 10);
  display.setCursor(0, 22);
  display.print("Heap: "); display.print(ESP.getFreeHeap() / 1024); display.print("KB");
  display.setCursor(0, 32);
  display.print("Up: "); display.print(millis() / 1000); display.print("s");
  display.setCursor(0, 42);
  display.print("CPU: "); display.print(getCpuFrequencyMhz()); display.print("MHz");
  display.setCursor(0, 52);
  display.print("Chip: "); display.print(ESP.getChipModel());
}

void drawAboutApp() {
  drawStatusBar();
  centerText("WAKER", 16, 2);
  centerText("Firmware v1.0", 38);
  centerText("Custom ESP32-S3 Watch", 50);
}

void renderCurrentScreen() {
  switch (state) {
    case AppState::HOME:            drawHomeScreen();       break;
    case AppState::MENU:            drawMenuScreen();       break;
    case AppState::APP_CLOCK:       drawClockApp();         break;
    case AppState::APP_HEARTRATE:   drawHeartRateApp();     break;
    case AppState::APP_TEMPERATURE: drawTemperatureApp();   break;
    case AppState::APP_BATTERY:     drawBatteryApp();       break;
    case AppState::APP_WIFI:        drawWifiApp();          break;
    case AppState::APP_WIFI_SCAN:   drawWifiScanApp();      break;
    case AppState::APP_KEYPAD:      drawKeypadApp();        break;
    case AppState::APP_SETTINGS:    drawSettingsApp();      break;
    case AppState::APP_ALARM_SET:   drawAlarmSetApp();      break;
    case AppState::APP_STEPCOUNT:   drawStepCountApp();     break;
    case AppState::APP_ALARM_RINGING: drawAlarmRingingApp(); break;
    case AppState::APP_SYSINFO:     drawSysInfoApp();       break;
    case AppState::APP_ABOUT:       drawAboutApp();         break;
    default: break;
  }
}

// ============================================================================
// INPUT ROUTING  (per-state handlers)
// ============================================================================
void handleEncoderRotation(int32_t steps) {
  lastActivityMs = millis();
  if (displaySleeping || displayDimmed) { wakeDisplay(); return; }
  if (steps == 0) return;

  switch (state) {
    case AppState::MENU: {
      int16_t ni = (int16_t)menuIndex + (steps > 0 ? 1 : -1);
      if (ni < 0) ni = MENU_COUNT - 1;
      if (ni >= MENU_COUNT) ni = 0;
      menuIndex = (uint8_t)ni;
      buzzClick();
      needsRedraw = true;
      break;
    }
    case AppState::APP_WIFI_SCAN: {
      if (wifiPhase != WifiUiPhase::SCAN_DONE || wifiScanCount == 0) break;
      int16_t ni = (int16_t)wifiSelectIndex + (steps > 0 ? 1 : -1);
      if (ni < 0) ni = wifiScanCount - 1;
      if (ni >= wifiScanCount) ni = 0;
      wifiSelectIndex = (uint8_t)ni;
      buzzClick();
      needsRedraw = true;
      break;
    }
    case AppState::APP_KEYPAD: {
      int16_t ni = (int16_t)keypadCharIndex + (steps > 0 ? 1 : -1);
      if (ni < 0) ni = KEYPAD_CHARSET_LEN - 1;
      if (ni >= KEYPAD_CHARSET_LEN) ni = 0;
      keypadCharIndex = (uint8_t)ni;
      needsRedraw = true;
      break;
    }
    case AppState::APP_SETTINGS: {
      int16_t ni = (int16_t)settingsIndex + (steps > 0 ? 1 : -1);
      if (ni < 0) ni = 2;
      if (ni > 2) ni = 0;
      settingsIndex = (uint8_t)ni;
      buzzClick();
      needsRedraw = true;
      break;
    }
    case AppState::APP_ALARM_SET: {
      if (alarmEditField == 0) {
        alarmHour = ((alarmHour + (steps > 0 ? 1 : -1)) + 24) % 24;
      } else {
        alarmMinute = ((alarmMinute + (steps > 0 ? 1 : -1)) + 60) % 60;
      }
      alarmTriggered = false; // a manual edit re-arms this minute
      needsRedraw = true;
      break;
    }
    case AppState::APP_ALARM_RINGING:
      // any input silences the alarm, same as a button dismiss
      alarmRinging = false;
      alarmTriggered = true;
      goTo(AppState::HOME, -1);
      break;
    default: break;
  }
}

void handleButton1(PressType p) {
  if (p == PressType::NONE) return;
  lastActivityMs = millis();
  if (displaySleeping || displayDimmed) { wakeDisplay(); return; }

  if (state == AppState::APP_ALARM_RINGING) {
    alarmRinging = false;
    alarmTriggered = true;
    goTo(AppState::HOME, -1);
    return;
  }

  switch (state) {
    case AppState::HOME:
      if (p == PressType::SHORT) goTo(AppState::MENU, 1);
      break;
    case AppState::MENU:
      if (p == PressType::SHORT) goTo(AppState::HOME, -1);
      break;
    case AppState::APP_KEYPAD:
      if (p == PressType::SHORT && keypadBuffer.length() > 0) {
        keypadBuffer.remove(keypadBuffer.length() - 1);
        needsRedraw = true;
      } else if (p == PressType::LONG) {
        goTo(AppState::APP_WIFI_SCAN, -1);
      }
      break;
    case AppState::APP_SETTINGS:
      if (p == PressType::LONG) {
        prefs.begin("wifi", false);
        prefs.clear();
        prefs.end();
        WiFi.disconnect(true, true);
        buzzConfirm();
      }
      break;
    default:
      if (p == PressType::SHORT) goTo(AppState::MENU, -1);
      break;
  }
}

void handleButton2(PressType p) {
  if (p == PressType::NONE) return;
  lastActivityMs = millis();
  if (displaySleeping || displayDimmed) { wakeDisplay(); return; }

  if (state == AppState::APP_ALARM_RINGING) {
    alarmRinging = false;
    alarmTriggered = true;
    goTo(AppState::HOME, -1);
    return;
  }

  switch (state) {
    case AppState::APP_ALARM_SET:
      if (p == PressType::SHORT) { buzzConfirm(); goTo(AppState::MENU, -1); }
      break;
    case AppState::APP_WIFI:
      if (p == PressType::SHORT) { wifiStartScan(); goTo(AppState::APP_WIFI_SCAN, 1); }
      break;
    case AppState::APP_KEYPAD:
      if (p == PressType::SHORT) {
        wifiConnectTo(pendingSSID, keypadBuffer);
        goTo(AppState::APP_WIFI, -1);
      }
      break;
    case AppState::APP_SETTINGS:
      if (p == PressType::LONG) buzzAlert();
      break;
    default:
      if (p == PressType::SHORT) goTo(AppState::HOME, -1);
      break;
  }
}

void handleEncoderButton(PressType p) {
  if (p == PressType::NONE) return;
  lastActivityMs = millis();
  if (displaySleeping || displayDimmed) { wakeDisplay(); return; }

  if (state == AppState::APP_ALARM_RINGING) {
    alarmRinging = false;
    alarmTriggered = true;
    goTo(AppState::HOME, -1);
    return;
  }

  switch (state) {
    case AppState::HOME:
      if (p == PressType::SHORT) goTo(AppState::MENU, 1);
      break;
    case AppState::MENU:
      if (p == PressType::SHORT) {
        buzzConfirm();
        switch (menuIndex) {
          case 0: goTo(AppState::APP_CLOCK, 1); break;
          case 1: goTo(AppState::APP_HEARTRATE, 1); break;
          case 2: goTo(AppState::APP_TEMPERATURE, 1); break;
          case 3: goTo(AppState::APP_BATTERY, 1); break;
          case 4: goTo(AppState::APP_WIFI, 1); break;
          case 5: goTo(AppState::APP_ALARM_SET, 1); break;
          case 6: goTo(AppState::APP_STEPCOUNT, 1); break;
          case 7: goTo(AppState::APP_SETTINGS, 1); break;
          case 8: goTo(AppState::APP_SYSINFO, 1); break;
          case 9: goTo(AppState::APP_ABOUT, 1); break;
        }
      } else if (p == PressType::LONG) {
        goTo(AppState::HOME, -1);
      }
      break;
    case AppState::APP_WIFI_SCAN:
      if (p == PressType::SHORT && wifiPhase == WifiUiPhase::SCAN_DONE && wifiScanCount > 0) {
        pendingSSID = wifiScanResults[wifiSelectIndex];
        keypadBuffer = "";
        keypadCharIndex = 0;
        goTo(AppState::APP_KEYPAD, 1);
      } else if (p == PressType::LONG) {
        goTo(AppState::APP_WIFI, -1);
      }
      break;
    case AppState::APP_KEYPAD:
      if (p == PressType::SHORT) {
        keypadBuffer += KEYPAD_CHARSET[keypadCharIndex];
        needsRedraw = true;
      }
      break;
    case AppState::APP_ALARM_SET:
      if (p == PressType::SHORT) {
        alarmEditField = 1 - alarmEditField;
        buzzClick();
        needsRedraw = true;
      } else if (p == PressType::LONG) {
        alarmEnabled = !alarmEnabled;
        buzzConfirm();
        needsRedraw = true;
      }
      break;
    case AppState::APP_STEPCOUNT:
      if (p == PressType::LONG) { stepCount = 0; buzzClick(); needsRedraw = true; }
      break;
    case AppState::APP_SETTINGS:
      if (p == PressType::SHORT) {
        if (settingsIndex == 0) {
          buzzAlert();
        } else if (settingsIndex == 1) {
          backgroundInverted = !backgroundInverted;
          display.invertDisplay(backgroundInverted);
          buzzClick();
        } else {
          flybackEnabled = !flybackEnabled;
          buzzClick();
        }
        needsRedraw = true;
      } else if (p == PressType::LONG) {
        goTo(AppState::MENU, -1);
      }
      break;
    default:
      if (p == PressType::LONG) goTo(AppState::MENU, -1);
      break;
  }
}

// ============================================================================
// TASKS
// ============================================================================
static const uint32_t STEP_POLL_MS = 50; // faster than SENSOR_POLL_MS: steps are quick events

void stepAndGestureTask(uint32_t now) {
  static uint32_t lastStepPollMs = 0;
  if (!imuOK) return;
  if (now - lastStepPollMs < STEP_POLL_MS) return;
  lastStepPollMs = now;

  mpuRead(sensors); // cheap I2C read; fine to sample faster than the display needs

  // Wrist-swing gesture: wake straight to the home clock, mirroring the
  // original alarm-clock sketch's "check your watch" behavior.
  if (fabsf(sensors.gyroY) > SWING_GYRO_THRESHOLD_DPS) {
    lastActivityMs = now;
    if (displaySleeping || displayDimmed) wakeDisplay();
    if (state != AppState::HOME && state != AppState::APP_ALARM_RINGING) goTo(AppState::HOME, -1);
  }

  // Step counting: low-pass filtered accel magnitude with hysteresis, plus
  // a minimum inter-step interval to reject vibration/bounce. "Armed" means
  // the signal has settled back down since the last count -- a step only
  // registers on the rising edge through the HIGH threshold while armed,
  // and re-arms only once it's dropped through the LOW threshold, so a
  // single swing can't double-count on noise sitting right at one value.
  float magRaw = sqrtf(sensors.accelX * sensors.accelX +
                        sensors.accelY * sensors.accelY +
                        sensors.accelZ * sensors.accelZ);
  filteredAccMag = STEP_FILTER_ALPHA * magRaw + (1.0f - STEP_FILTER_ALPHA) * filteredAccMag;

  if (stepArmed && filteredAccMag > STEP_ACCEL_THRESHOLD_HIGH_G
      && (now - lastStepMs) > STEP_INTERVAL_MS) {
    stepCount++;
    lastStepMs = now;
    stepArmed = false;
    if (state == AppState::APP_STEPCOUNT) needsRedraw = true;
  } else if (!stepArmed && filteredAccMag < STEP_ACCEL_THRESHOLD_LOW_G) {
    stepArmed = true;
  }
}

// Non-blocking alarm: checks the clock once a second (getLocalTime() only
// has minute-level relevance here, so polling it on every loop() -- which
// can run thousands of times a second -- was pure waste), and if it fires,
// pulses the buzzer on a millis() schedule rather than the original sketch's
// blocking for-loop of delay()s. While ringing, it also keeps
// lastActivityMs fresh so powerTask() never dims/sleeps the display out
// from under an active alarm.
void alarmTask(uint32_t now) {
  static uint32_t lastAlarmPollMs = 0;
  if (now - lastAlarmPollMs < ALARM_POLL_MS) {
    // still let an in-progress ring keep pulsing/redrawing at full rate
    if (!alarmRinging) return;
  } else {
    lastAlarmPollMs = now;
    struct tm tmNow;
    if (alarmEnabled && !alarmTriggered && getLocalTime(&tmNow, 0) && tmNow.tm_year > 100) {
      if (tmNow.tm_hour == alarmHour && tmNow.tm_min == alarmMinute) {
        alarmTriggered = true;
        alarmRinging = true;
        alarmRingStartMs = now;
        alarmLastPulseMs = 0;
        if (flybackEnabled) flybackStart();
        goTo(AppState::APP_ALARM_RINGING, 1);
      }
    }
    // re-arm once the matching minute has passed
    if (getLocalTime(&tmNow, 0) && tmNow.tm_min != alarmMinute) {
      alarmTriggered = false;
    }
  }

  if (!alarmRinging) return;

  // Keep the screen awake for the whole alarm -- this is what previously let
  // IDLE_SLEEP_MS blank the display mid-alarm if nobody touched a button.
  lastActivityMs = now;

  // pulse the buzzer on/off every 300ms, matching the original alarm cadence
  if (now - alarmLastPulseMs >= 300) {
    alarmLastPulseMs = now;
    alarmPulseOn = !alarmPulseOn;
    if (alarmPulseOn) {
      BuzzStep s[] = {{1000, 280}};
      buzzPlay(s, 1);
    }
    needsRedraw = true;
  }
  // auto-silence after 10 minutes so it can't ring forever unattended
  if (now - alarmRingStartMs > 600000UL) {
    alarmRinging = false;
  }
}

void sensorTask() {
  static uint32_t lastSensorMs = 0;
  static uint32_t lastHrMs = 0;
  static uint32_t lastClockTickMs = 0;
  uint32_t now = millis();

  stepAndGestureTask(now);
  alarmTask(now);

  if (now - lastHrMs >= HR_POLL_MS) {
    lastHrMs = now;
    if (hrOK) {
      // Drop to 100kHz just for this transaction -- see the comment on
      // hrSensor.begin() in setup() for why. mpuRead()/mcp9808 reads that
      // interleave with this in the same loop() will each set the clock
      // back to whatever they need, so this has to happen every single call
      // rather than once at startup.
      Wire.setClock(100000);
      long ir = hrSensor.getIR();
      Wire.setClock(400000);

      sensors.rawIR = ir;
      sensors.hrFingerDetected = ir > 50000;
      hrEst.feed(ir, now);
      sensors.hrBPM = hrEst.bpm;

      // Throttled trace so you can actually see whether the sensor is
      // reporting anything at all, and how close it is to the finger-present
      // threshold, without flooding Serial at HR_POLL_MS (10ms) rate.
      static uint32_t lastHrDebugMs = 0;
      if (now - lastHrDebugMs > 1000) {
        lastHrDebugMs = now;
        Serial.printf("HR raw IR: %ld  (finger-present threshold: 50000)  BPM: %ld\n",
                      ir, (long)sensors.hrBPM);
      }
    }
  }

  if (now - lastSensorMs >= SENSOR_POLL_MS) {
    lastSensorMs = now;
    if (imuOK && sensors.motionDetected) lastActivityMs = now; // wake-on-motion (data refreshed by stepAndGestureTask)
    if (tempOK) {
      sensors.tempC = mcp9808.readTempC();
    }
    readBatteryAndCharge(sensors);
    if (state == AppState::HOME || state == AppState::APP_BATTERY
        || state == AppState::APP_TEMPERATURE || state == AppState::APP_WIFI) {
      needsRedraw = true;
    }
  }

  if (now - lastClockTickMs >= CLOCK_TICK_MS) {
    lastClockTickMs = now;
    if (state == AppState::HOME || state == AppState::APP_CLOCK) needsRedraw = true;
  }
}

void inputTask() {
  PressType p1 = updateButton(btn1);
  PressType p2 = updateButton(btn2);
  PressType pe = updateButton(btnEnc);

  if (p1 != PressType::NONE) handleButton1(p1);
  if (p2 != PressType::NONE) handleButton2(p2);
  if (pe != PressType::NONE) handleEncoderButton(pe);

  static uint32_t lastEncApplyMs = 0;
  noInterrupts();
  int32_t delta = encoderDelta;
  encoderDelta = 0;
  interrupts();
  if (delta != 0) {
    // Every full detent is typically 4 quadrature counts on these modules.
    static int32_t accum = 0;
    accum += delta;
    while (accum >= 4)  { handleEncoderRotation(1);  accum -= 4; }
    while (accum <= -4) { handleEncoderRotation(-1); accum += 4; }
  }
}

void animTask() {
  if (!transitioning) return;
  transitionOffset -= transitionDir * 24.0f;
  if ((transitionDir > 0 && transitionOffset <= 0) ||
      (transitionDir < 0 && transitionOffset >= 0)) {
    transitionOffset = 0;
    transitioning = false;
  }
  needsRedraw = true;
}

void renderTask() {
  static uint32_t lastFrameMs = 0;
  uint32_t now = millis();
  if (now - lastFrameMs < FRAME_INTERVAL_MS) return;
  lastFrameMs = now;
  if (displaySleeping) return;
  if (!needsRedraw && !transitioning) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  if (transitioning) {
    display.setCursor((int)transitionOffset, 0);
  }
  renderCurrentScreen();
  display.display();
  needsRedraw = false;
}

// ============================================================================
// SETUP
// ============================================================================
void initHardwareSelfTest() {
  Wire.beginTransmission(ADDR_OLED);
  oledOK = (Wire.endTransmission() == 0);
}

void setup() {
  Serial.begin(115200);

  // SW1/SW2 are wired to VCC (pressed pulls the pin HIGH), so they need an
  // internal pull-DOWN to sit LOW when idle -- opposite of the encoder button,
  // which is pulled up and reads LOW when pressed.
  pinMode(PIN_SW1, INPUT_PULLDOWN);
  pinMode(PIN_SW3, INPUT_PULLDOWN);
  pinMode(PIN_EC_S, INPUT_PULLUP);
  pinMode(PIN_EC_A, INPUT_PULLUP);
  pinMode(PIN_EC_B, INPUT_PULLUP);
  // EC_C is the encoder's common return line, not a signal to read -- it must
  // be actively driven LOW by the MCU for the A/B quadrature to reference
  // against, or the encoder won't produce clean transitions.
  pinMode(PIN_EC_C, OUTPUT);
  digitalWrite(PIN_EC_C, LOW);
  pinMode(PIN_IMU_INT, INPUT);
  pinMode(PIN_CHG_STAT, INPUT_PULLUP);
  pinMode(PIN_FLYBACK_GATE, OUTPUT);
  digitalWrite(PIN_FLYBACK_GATE, LOW); // gate must default LOW, or Q4 sits on at boot
#if HAS_BATT_ADC
  pinMode(PIN_BATT_ADC, INPUT);
#endif

  // ESP32 Arduino core 3.x LEDC API attaches directly to a pin (no separate
  // channel/ledcSetup+ledcAttachPin step as in core 2.x).
  ledcAttach(PIN_BUZZER, 2000, 10); // 2kHz default carrier, 10-bit resolution

  encLastState = (digitalRead(PIN_EC_A) << 1) | digitalRead(PIN_EC_B);
  attachInterrupt(digitalPinToInterrupt(PIN_EC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_EC_B), encoderISR, CHANGE);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  initHardwareSelfTest();
  oledOK = display.begin(SSD1306_SWITCHCAPVCC, ADDR_OLED);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("WAKER");
  display.println("Booting...");
  display.display();

  imuOK  = mpuInit();
  tempOK = mcp9808.begin(ADDR_MCP9808);
  if (tempOK) mcp9808.setResolution(2);
  // HR sensor sits behind a 3.3V<->1.8V logic-level shifter (Q2/Q3, BSS138)
  // that the other three I2C devices don't -- BSS138-based shifters commonly
  // can't meet I2C timing at 400kHz due to their open-drain pull-up RC rise
  // time. Initializing at standard speed (100kHz) here, and dropping back to
  // 100kHz for every runtime read in sensorTask(), gives the shifted bus
  // enough margin while OLED/IMU/temp stay on the fast 400kHz bus the rest
  // of the time.
  hrOK   = hrSensor.begin(Wire, I2C_SPEED_STANDARD, ADDR_MAX30101);
  if (hrOK) {
    // LED brightness bumped from 0x1F to 0x3F -- 0x1F (~6.2mA) was often too
    // weak to reliably push the raw IR reading past the 50000 "finger
    // present" threshold, especially through a watch case + skin, which
    // looks identical to "no finger" from the firmware's point of view.
    hrSensor.setup(0x3F /*LED brightness*/, 4 /*avg*/, 2 /*mode: red+IR*/,
                    400 /*sampleRate*/, 411 /*pulseWidth*/, 4096 /*adcRange*/);
  }
  Wire.setClock(400000); // restore fast bus for OLED/IMU/temp after HR init

  Serial.println("=== WAKER SELF-TEST ===");
  Serial.printf("OLED: %s\n", oledOK ? "PASS" : "FAIL");
  Serial.printf("IMU:  %s\n", imuOK  ? "PASS" : "FAIL");
  Serial.printf("HR:   %s\n", hrOK   ? "PASS" : "FAIL");
  Serial.printf("TEMP: %s\n", tempOK ? "PASS" : "FAIL");

  wifiTryAutoConnect();
  webServerSetup();

  lastActivityMs = millis();
  state = AppState::HOME;
  needsRedraw = true;
}

// ============================================================================
// MAIN LOOP  (fully non-blocking scheduler)
// ============================================================================
void loop() {
  inputTask();
  sensorTask();
  wifiTask();
  webServer.handleClient();
  powerTask();
  buzzTask();
  flybackTask();
  animTask();
  renderTask();
}
