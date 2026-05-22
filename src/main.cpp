// =============================================================================
//  nlOpenRotator — ESP32 TV/RCA Antenna Rotator Controller
//  Controls a small AC antenna rotator (e.g. RCA VH-226) via two relays
//  and a 24 V AC transformer.  Position is tracked by timed dead-reckoning
//  (no potentiometer required — 3-wire rotators supported).
//
//  Web UI  : http://192.168.4.1  (connect to "OpenRotator" WiFi AP)
//  Serial  : 115200 baud — type HELP for command list
//
//  See wiring.md for full wiring diagram and calibration instructions.
// =============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <LiquidCrystal_I2C.h>
#include "soc/rtc_cntl_reg.h"  // brownout detector control
#include "esp_system.h"          // esp_reset_reason()

// ─── Pin definitions ──────────────────────────────────────────────────────────
//  Relay module wiring (2× 1-channel, 5 V, active-LOW, optocoupler-isolated):
//    PIN_MOTOR_CW  → relay module IN1  (LOW = relay on = 24 V AC to CW winding)
//    PIN_MOTOR_CCW → relay module IN2  (LOW = relay on = 24 V AC to CCW winding)
//  Relay module VCC → ESP32 5 V (VIN).  Never energise both relays simultaneously.
#define PIN_MOTOR_CW   26   // digital output → relay IN1 (active LOW) — sweeps 0°→360°
#define PIN_MOTOR_CCW  25   // digital output → relay IN2 (active LOW) — returns to 0°
//  Optional status LED (built-in on most ESP32 devkits)
#define PIN_LED         2

// ─── LCD display (1602A via I2C backpack, PCF8574) ────────────────────────────
//  SDA = GPIO21, SCL = GPIO22 (ESP32 I2C defaults — no config needed)
//  Address auto-detected at boot: 0x27 (PCF8574, most common) or 0x3F (PCF8574A)
LiquidCrystal_I2C* lcd       = nullptr;  // allocated in setup() after I2C scan
bool               lcdPresent = false;

// ─── WiFi access-point credentials ───────────────────────────────────────────
const char* AP_SSID = "OpenRotator";
const char* AP_PASS = "rotator123";   // change to something stronger!

// ─── Calibration ─────────────────────────────────────────────────────────────
//  Time how long (in ms) the rotator takes to turn a known number of degrees,
//  then set MS_PER_DEG = elapsed_ms / degrees_turned.
//  Example: rotator does a full 360° in 45 seconds → MS_PER_DEG = 45000/360 = 125
//  Use the TIMECW / TIMECCW serial commands to measure this at runtime.
float MS_PER_DEG   = 163.0f;  // milliseconds per degree of rotation — calibrated 2026-05-07 (58690ms/360deg)

// ─── Tuning knobs ─────────────────────────────────────────────────────────────
float DEADBAND     = 4.0f;    // Stop within ±N degrees of target (increase if overshooting)
const unsigned long RELAY_COOLDOWN_MS  = 600;   // Min ms between relay direction changes (prevents inrush brownout)
const unsigned long NVS_HDG_INTERVAL_MS = 10000; // Min ms between NVS heading writes (throttle flash wear)

// ─── Physical limits (non-circular rotator has hard stops at both ends) ──────
//  Set these after calibrating, e.g. SETMIN 0  SETMAX 360
//  GOTO targets are clamped to [HDG_MIN, HDG_MAX]; heading never wraps.
float HDG_MIN      = 0.0f;    // CCW hard stop — rotator cannot go below this
float HDG_MAX      = 360.0f;  // CW  hard stop — rotator cannot go above this
float HDG_OFFSET   = 0.0f;    // Compass mount offset: added to physical heading for display
                               // e.g. 180 = physical 0° faces South; -90 = faces West

// ─── Runtime state ────────────────────────────────────────────────────────────
WebServer server(80);
float         currentHeading  = 0.0f;
float         targetHeading   = -1.0f;  // negative = no active target
String        rotatorStatus   = "IDLE";
bool          headingKnown    = false;  // false until SETHOME is called
int           motorDirection  = 0;      // 0=stopped, 1=CW, -1=CCW
float         motorStartHdg   = 0.0f;  // heading when motor last started
unsigned long motorStartMs    = 0;     // millis() when motor last started
unsigned long lastControlMs   = 0;
bool          timingMode      = false;   // true during TIMECW/TIMECCW calibration run
unsigned long timingStartMs   = 0;       // millis() when timing started
bool          freeRunMode     = false;   // true during manual CW/CCW/TIMECW/TIMECCW (no GOTO target)
unsigned long lastRelayChangeMs = 0;     // millis() of last relay on/off transition (cooldown guard)
unsigned long lastNvsHdgWriteMs = 0;     // millis() of last NVS heading write (throttle guard)
unsigned long lastLcdUpdateMs   = 0;     // millis() of last LCD refresh
String        currentTargetLabel = "";   // preset label shown on LCD during active GOTO
Preferences   prefs;                     // NVS storage for calibration + last heading
bool          hdgRestored     = false;   // true if heading was loaded from NVS on boot
bool          prefsReady      = false;   // true once prefs.begin() has been called in setup()
String        staSSID         = "";       // home WiFi SSID (empty = STA disabled)
String        staPass         = "";       // home WiFi password
bool          staConnected    = false;    // true once STA link is up
IPAddress     staIP;

// =============================================================================
//  Position tracking — timed dead-reckoning
//  Call updateHeading() at any time to refresh currentHeading from elapsed run time.
// =============================================================================
void updateHeading() {
    if (motorDirection == 0) return;
    unsigned long elapsed = millis() - motorStartMs;
    float delta = (float)elapsed / MS_PER_DEG;
    // Clamp to physical limits — heading is linear, not circular
    // CW (motorDirection=1) increases heading (0→360); CCW (motorDirection=-1) decreases it
    float newHdg = motorStartHdg + (motorDirection * delta);
    currentHeading = fmaxf(HDG_MIN, fminf(HDG_MAX, newHdg));
}

// =============================================================================
//  Motor control — relay-based, active-LOW (LOW = relay energised)
//  Never energise both relays simultaneously; always de-energise one before
//  energising the other to avoid briefly shorting the transformer secondary.
//  Each start/stop snapshots the heading and timestamp for dead-reckoning.
// =============================================================================
void motorStop() {
    updateHeading();                    // capture final position before stopping
    if (prefsReady && motorDirection != 0) {
        prefs.putBool("motordirty", false); // clean stop — heading is valid
    }
    motorDirection = 0;
    digitalWrite(PIN_MOTOR_CW,  HIGH);  // HIGH = relay off (active-LOW module)
    digitalWrite(PIN_MOTOR_CCW, HIGH);
    motorStartHdg = currentHeading;     // keep snapshot current
    rotatorStatus = "IDLE";
    digitalWrite(PIN_LED, LOW);
    if (headingKnown) {
        unsigned long now = millis();
        if (now - lastNvsHdgWriteMs >= NVS_HDG_INTERVAL_MS) {
            prefs.putFloat("lastheading", currentHeading);
            lastNvsHdgWriteMs = now;
        }
    }
}

void motorCW() {
    motorStop();                        // de-energise + capture heading first
    delay(200);                         // interlock: give relay contacts and AC motor field time to collapse
    if (prefsReady && headingKnown) {
        prefs.putFloat("lastheading", currentHeading); // snapshot position before moving
        prefs.putBool("motordirty",  true);            // mark motor as running
        lastNvsHdgWriteMs = millis();
    }
    lastRelayChangeMs = millis();
    motorStartHdg = currentHeading;
    motorStartMs  = millis();
    motorDirection = 1;
    digitalWrite(PIN_MOTOR_CW,  LOW);   // LOW = relay on → 24 V AC to CW winding
    rotatorStatus = "ROTATING_CW";
    digitalWrite(PIN_LED, HIGH);
}

void motorCCW() {
    motorStop();                        // de-energise + capture heading first
    delay(200);                         // interlock: give relay contacts and AC motor field time to collapse
    if (prefsReady && headingKnown) {
        prefs.putFloat("lastheading", currentHeading); // snapshot position before moving
        prefs.putBool("motordirty",  true);            // mark motor as running
        lastNvsHdgWriteMs = millis();
    }
    lastRelayChangeMs = millis();
    motorStartHdg = currentHeading;
    motorStartMs  = millis();
    motorDirection = -1;
    digitalWrite(PIN_MOTOR_CCW, LOW);   // LOW = relay on → 24 V AC to CCW winding
    rotatorStatus = "ROTATING_CCW";
    digitalWrite(PIN_LED, HIGH);
}

// =============================================================================
//  LCD — 1602A via I2C backpack (SDA=GPIO21, SCL=GPIO22)
//  Line 1: heading + preset label (if active) or "deg" when idle
//  Line 2: status and target
// =============================================================================
const unsigned long LCD_UPDATE_MS = 500;  // refresh interval (ms)

void updateLCD() {
    if (!lcdPresent || !lcd) return;

    auto toDisplay = [](float phys) {
        return fmodf(phys + HDG_OFFSET + 360.0f, 360.0f);
    };

    char line1[17], line2[17];

    // Line 1 — heading; show preset label on right when a named GOTO is active
    if (headingKnown) {
        if (currentTargetLabel.length() > 0) {
            snprintf(line1, sizeof(line1), "HDG:%5.1f %-6s",
                     toDisplay(currentHeading), currentTargetLabel.c_str());
        } else {
            snprintf(line1, sizeof(line1), "HDG: %5.1f deg  ", toDisplay(currentHeading));
        }
    } else {
        snprintf(line1, sizeof(line1), "HDG:  ---       ");
    }

    // Line 2 — motion status / target
    const char* dirStr = (motorDirection == 1) ? "CW " : "CCW";
    if (motorDirection != 0) {
        if (targetHeading >= 0.0f)
            snprintf(line2, sizeof(line2), ">%-5.1f  ROT %s ", toDisplay(targetHeading), dirStr);
        else
            snprintf(line2, sizeof(line2), "FREE ROT %s    ", dirStr);
    } else if (rotatorStatus == "AT_TARGET") {
        snprintf(line2, sizeof(line2), "AT TARGET       ");
    } else if (rotatorStatus == "RUNAWAY") {
        snprintf(line2, sizeof(line2), "!! RUNAWAY !!   ");
    } else if (rotatorStatus == "LIMIT_HIT") {
        snprintf(line2, sizeof(line2), "!! LIMIT HIT !! ");
    } else {
        snprintf(line2, sizeof(line2), "IDLE            ");
    }

    lcd->setCursor(0, 0);
    lcd->print(line1);
    lcd->setCursor(0, 1);
    lcd->print(line2);
}

// =============================================================================
//  Control loop  (called every 100 ms)
// =============================================================================
void controlLoop() {
    updateHeading();  // refresh currentHeading from elapsed motor run time

    // Persist heading every 3 s while rotating (reduces worst-case NVS lag on power loss)
    if (prefsReady && headingKnown && motorDirection != 0) {
        unsigned long now = millis();
        if (now - lastNvsHdgWriteMs >= 3000UL) {
            prefs.putFloat("lastheading", currentHeading);
            lastNvsHdgWriteMs = now;
        }
    }

    // Runaway guard — force-stop if motor has run longer than full arc + 50% margin
    // Skips during timingMode where an intentional full-arc run is expected
    if (motorDirection != 0 && !timingMode) {
        unsigned long maxRunMs = (unsigned long)((HDG_MAX - HDG_MIN) * MS_PER_DEG * 1.5f);
        if (millis() - motorStartMs > maxRunMs) {
            motorStop();
            freeRunMode        = false;
            targetHeading      = -1.0f;
            rotatorStatus      = "RUNAWAY";
            currentTargetLabel = "";
            Serial.println(F("ERR: RUNAWAY — motor force-stopped after exceeding max runtime."));
            return;
        }
    }

    // Physical limit guard — stop if heading has reached an end-stop
    // CW increases heading → stop at HDG_MAX; CCW decreases heading → stop at HDG_MIN
    // Skip ONLY during timingMode calibration runs where limits are intentionally bypassed
    if (!timingMode) {
        if (motorDirection == 1 && currentHeading >= HDG_MAX) {
            motorStop();
            freeRunMode   = false;
            rotatorStatus = "LIMIT_HIT";
            targetHeading = -1.0f;
            Serial.println(F("WARN: CW limit reached — motor stopped."));
            return;
        }
        if (motorDirection == -1 && currentHeading <= HDG_MIN) {
            motorStop();
            freeRunMode   = false;
            rotatorStatus = "LIMIT_HIT";
            targetHeading = -1.0f;
            Serial.println(F("WARN: CCW limit reached — motor stopped."));
            return;
        }
    }

    if (!headingKnown || targetHeading < 0.0f) {
        // No known reference or no active target — stop ONLY if not a free-run
        if (motorDirection != 0 && !freeRunMode) motorStop();
        return;
    }

    // Linear routing — rotator has physical end stops, no wrap-around
    // Always travel directly toward the target on the available arc
    float error = targetHeading - currentHeading;

    if (fabsf(error) <= DEADBAND) {
        motorStop();
        rotatorStatus = "AT_TARGET";
        targetHeading = -1.0f;          // arrived — clear target
    } else if (error > 0.0f) {
        if (motorDirection != 1) motorCW();    // CW increases heading
    } else {
        if (motorDirection != -1) motorCCW();  // CCW decreases heading
    }
}

// =============================================================================
//  Serial command processor
//  Commands (case-insensitive):
//    GOTO <deg>          Rotate to heading (0-359)  — requires SETHOME first
//    STOP                Halt immediately
//    STATUS              Print heading, target, state, ms/deg, deadband
//    SETHOME [deg]       Declare current position as <deg> (default 0) and enable GOTO
//    MSPERDEG <val>      Set milliseconds-per-degree speed constant
//    DEADBAND <deg>      Set arrival tolerance in degrees
//    HELP                List commands
// =============================================================================
String processCommand(const String& rawCmd) {
    String cmd = rawCmd;
    cmd.trim();
    // Handle SETWIFI before uppercasing so SSID and password preserve their case
    String cmdCheck = cmd;
    cmdCheck.toUpperCase();
    if (cmdCheck.startsWith("SETWIFI ")) {
        String rest = cmd.substring(8);
        rest.trim();
        int sp = rest.indexOf(' ');
        if (sp < 1) return "ERR Usage: SETWIFI <ssid> <password>";
        String ssid = rest.substring(0, sp);
        String pass = rest.substring(sp + 1);
        ssid.trim(); pass.trim();
        if (ssid.length() == 0) return "ERR SSID cannot be empty";
        prefs.putString("wifissid", ssid);
        prefs.putString("wifipass", pass);
        staSSID = ssid; staPass = pass;
        return "OK WiFi credentials saved. Reboot to connect to " + ssid;
    }
    cmd.toUpperCase();

    if (cmd == "CW") {
        targetHeading      = -1.0f;
        freeRunMode        = true;
        currentTargetLabel = "";
        motorCW();
        return "OK ROTATING_CW";
    }
    if (cmd == "CCW") {
        targetHeading      = -1.0f;
        freeRunMode        = true;
        currentTargetLabel = "";
        motorCCW();
        return "OK ROTATING_CCW";
    }
    if (cmd == "TIMECW") {
        targetHeading      = -1.0f;
        freeRunMode        = true;
        timingMode         = true;
        timingStartMs      = millis();
        currentTargetLabel = "";
        motorCW();
        return "OK TIMING_CW - press STOP when done to see elapsed time";
    }
    if (cmd == "TIMECCW") {
        targetHeading      = -1.0f;
        freeRunMode        = true;
        timingMode         = true;
        timingStartMs      = millis();
        currentTargetLabel = "";
        motorCCW();
        return "OK TIMING_CCW - press STOP when done to see elapsed time";
    }
    if (cmd.startsWith("HOMEDIR ")) {
        float bearing = fmodf(cmd.substring(8).toFloat() + 360.0f, 360.0f);
        motorStop();
        currentHeading     = 0.0f;
        motorStartHdg      = 0.0f;
        headingKnown       = true;
        hdgRestored        = false;
        HDG_OFFSET         = bearing;
        currentTargetLabel = "";
        prefs.putFloat("lastheading", 0.0f);
        prefs.putBool("hdgknown", true);
        prefs.putFloat("hdgoffset", HDG_OFFSET);
        return "OK HOMEDIR=" + String(bearing, 1) + "deg — CCW stop = " + String(bearing, 0) + "deg compass. GOTO enabled.";
    }
    if (cmd.startsWith("GOTO ")) {
        if (!headingKnown) return "ERR HEADING_UNKNOWN — run SETHOME first";
        // Input is a compass bearing; convert to physical heading by subtracting offset
        float displayDeg = cmd.substring(5).toFloat();
        float deg = fmodf(displayDeg - HDG_OFFSET + 360.0f, 360.0f);
        deg = fmaxf(HDG_MIN, fminf(HDG_MAX, deg));  // clamp to physical limits
        freeRunMode   = false;
        targetHeading = deg;
        return "OK GOING_TO=" + String(displayDeg, 1) + " (physical=" + String(deg, 1) + ")";
    }
    if (cmd == "STOP") {
        String resp = "OK STOPPED";
        int prevDir = motorDirection;  // capture before motorStop() clears it
        if (timingMode) {
            unsigned long elapsed = millis() - timingStartMs;
            timingMode = false;
            float sweep = HDG_MAX - HDG_MIN;
            resp += " | TIMED=" + String(elapsed) + "ms";
            if (sweep > 0.0f)
                resp += " (for " + String(sweep, 0) + "deg sweep set MSPERDEG " + String(elapsed / sweep, 1) + ")";
        // Auto-home: CCW end stop = HDG_MIN (0°) — only set on CCW run
        if (prevDir == -1) {
            float homePos = HDG_MIN;
            currentHeading = homePos;
            motorStartHdg  = homePos;
            headingKnown   = true;
            hdgRestored    = false;
            prefs.putFloat("lastheading", homePos);
            prefs.putBool("hdgknown", true);
            resp += " | HOME_AUTO=" + String(homePos, 0) + "deg";
        }
        }
        freeRunMode        = false;
        targetHeading      = -1.0f;
        currentTargetLabel = "";
        motorStop();
        return resp;
    }
    if (cmd == "STATUS") {
        auto toDisplay = [](float phys) {
            return fmodf(phys + HDG_OFFSET + 360.0f, 360.0f);
        };
        String hdgStr = headingKnown ? String(toDisplay(currentHeading), 1) : String("?");
        String tgt    = (targetHeading >= 0.0f) ? String(toDisplay(targetHeading), 1) : "NONE";
        unsigned long runMs = (motorDirection != 0) ? (millis() - motorStartMs) : 0UL;
        String cwStr  = digitalRead(PIN_MOTOR_CW)  == LOW ? "LOW" : "HIGH";
        String ccwStr = digitalRead(PIN_MOTOR_CCW) == LOW ? "LOW" : "HIGH";
        return "HDG=" + hdgStr +
               " TGT=" + tgt +
               " STS=" + rotatorStatus +
               " MS/DEG=" + String(MS_PER_DEG, 1) +
               " DB=" + String(DEADBAND, 1) +
               " CW=" + cwStr + " CCW=" + ccwStr +
               " RUNMS=" + String(runMs) +
               " UP=" + String(millis()) +
               " KNOWN=" + String(headingKnown ? 1 : 0) +
               " TIMING=" + String(timingMode ? 1 : 0) +
               " MIN=" + String(HDG_MIN, 1) +
               " MAX=" + String(HDG_MAX, 1) +
               " OFFSET=" + String(HDG_OFFSET, 1) +
               " RESTORED=" + String(hdgRestored ? 1 : 0);
    }
    if (cmd.startsWith("SETHOME")) {
        float deg = 0.0f;
        if (cmd.length() > 8) {
            deg = cmd.substring(8).toFloat();
            deg = fmaxf(HDG_MIN, fminf(HDG_MAX, deg));  // clamp to limits
        }
        motorStop();
        currentHeading = deg;
        motorStartHdg  = deg;
        headingKnown   = true;
        hdgRestored    = false;
        prefs.putFloat("lastheading", deg);
        prefs.putBool("hdgknown", true);
        return "OK HOME_SET=" + String(deg, 1);
    }
    if (cmd.startsWith("MSPERDEG ")) {
        float v = cmd.substring(9).toFloat();
        if (v <= 0.0f) return "ERR VALUE_MUST_BE_POSITIVE";
        MS_PER_DEG = v;
        prefs.putFloat("msperdeg", MS_PER_DEG);
        return "OK MS_PER_DEG=" + String(MS_PER_DEG, 1);
    }
    if (cmd.startsWith("DEADBAND ")) {
        DEADBAND = max(0.5f, cmd.substring(9).toFloat());
        return "OK DEADBAND=" + String(DEADBAND, 1);
    }
    if (cmd.startsWith("SETOFFSET ")) {
        HDG_OFFSET = fmodf(cmd.substring(10).toFloat() + 360.0f, 360.0f);
        prefs.putFloat("hdgoffset", HDG_OFFSET);
        return "OK HDG_OFFSET=" + String(HDG_OFFSET, 1) + " (physical 0deg now displays as " + String(HDG_OFFSET, 0) + "deg)";
    }
    if (cmd.startsWith("SETMIN ")) {
        HDG_MIN = cmd.substring(7).toFloat();
        prefs.putFloat("hdgmin", HDG_MIN);
        return "OK HDG_MIN=" + String(HDG_MIN, 1);
    }
    if (cmd.startsWith("SETMAX ")) {
        HDG_MAX = cmd.substring(7).toFloat();
        prefs.putFloat("hdgmax", HDG_MAX);
        return "OK HDG_MAX=" + String(HDG_MAX, 1);
    }
    if (cmd == "RESETCAL") {
        prefs.remove("msperdeg");
        prefs.remove("hdgmin");
        prefs.remove("hdgmax");
        prefs.remove("deadband");
        prefs.remove("hdgoffset");
        prefs.remove("lastheading");
        prefs.remove("hdgknown");
        prefs.remove("motordirty");
        MS_PER_DEG         = 163.0f;
        HDG_MIN            = 0.0f;
        HDG_MAX            = 360.0f;
        HDG_OFFSET         = 0.0f;
        DEADBAND           = 4.0f;
        headingKnown       = false;
        hdgRestored        = false;
        currentTargetLabel = "";
        motorStop();
        return "OK RESETCAL — calibration cleared, WiFi credentials preserved. Run HOMEDIR <bearing> or SETHOME to re-enable GOTO.";
    }
    if (cmd == "FACTORY") {
        prefs.clear();
        MS_PER_DEG   = 163.0f;
        HDG_MIN      = 0.0f;
        HDG_MAX      = 360.0f;
        HDG_OFFSET   = 0.0f;
        DEADBAND     = 4.0f;
        headingKnown = false;
        hdgRestored  = false;
        staSSID      = "";
        staPass      = "";
        motorStop();
        return "OK FACTORY_RESET -- all calibration and WiFi credentials cleared. Run SETHOME to re-enable GOTO.";
    }
    // SETWIFI handled above (before toUpperCase)
    if (cmd == "WIFICLEAR") {
        prefs.remove("wifissid"); prefs.remove("wifipass");
        staSSID = ""; staPass = "";
        return "OK WiFi credentials cleared. Reboot to return to AP-only mode.";
    }
    if (cmd == "WIFISTATUS") {
        String s = "AP IP=" + WiFi.softAPIP().toString();
        if (staSSID.length() > 0) {
            s += " | STA SSID=" + staSSID;
            s += staConnected ? (" IP=" + staIP.toString()) : " DISCONNECTED";
        } else {
            s += " | STA disabled (use SETWIFI to enable)";
        }
        return s;
    }
    if (cmd == "HELP") {
        return "Commands: CW | CCW | TIMECW | TIMECCW | GOTO <deg> | STOP | STATUS | "
               "SETHOME [deg] | HOMEDIR <bearing> | MSPERDEG <val> | DEADBAND <deg> | "
               "SETMIN <deg> | SETMAX <deg> | SETOFFSET <deg> | PINTEST | "
               "SETWIFI <ssid> <pass> | WIFICLEAR | WIFISTATUS | RESETCAL | FACTORY | HELP";
    }
    // PINTEST — pulse each output pin 2x so you can hear/see which relay fires
    if (cmd == "PINTEST") {
        motorStop();
        Serial.println(F("PINTEST: pulsing GPIO25 (2x 300ms) ..."));
        for (int i = 0; i < 2; i++) {
            digitalWrite(25, LOW);  delay(300);
            digitalWrite(25, HIGH); delay(300);
        }
        Serial.println(F("PINTEST: pulsing GPIO26 (2x 300ms) ..."));
        for (int i = 0; i < 2; i++) {
            digitalWrite(26, LOW);  delay(300);
            digitalWrite(26, HIGH); delay(300);
        }
        return "PINTEST done — which relay clicked on GPIO25 vs GPIO26?";
    }
    return "ERR UNKNOWN_CMD (type HELP)";
}

// =============================================================================
//  Embedded Web UI
// =============================================================================
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>OpenRotator</title>
<style>
:root{
  --bg:#0d1117;--sur:#161b22;--el:#21262d;--br:#30363d;--bs:#21262d;
  --tx:#e6edf3;--mu:#7d8590;--su:#484f58;--ac:#2f81f7;--ah:#58a6ff;
  --gn:#3fb950;--gd:#1a3a22;--rd:#f85149;--rdd:#3a1a1a;
  --am:#d29922;--ad:#3a2a00;--pu:#a371f7;--pd:#2a1a3a;--cy:#39c5cf;
}
*{box-sizing:border-box;margin:0;padding:0}
html,body{height:100%;overflow:hidden}
body{background:var(--bg);color:var(--tx);font-family:system-ui,-apple-system,sans-serif;font-size:13px;display:flex;flex-direction:column}
#hdr{flex:none;background:var(--sur);border-bottom:1px solid var(--br);padding:0 14px;height:46px;display:flex;align-items:center;gap:10px}
.logo{display:flex;align-items:center;gap:9px;flex:none}
.logo-ic{width:28px;height:28px;border-radius:50%;background:var(--el);border:1px solid var(--br);display:flex;align-items:center;justify-content:center;color:var(--ac);font-size:13px}
.logo-name{font-size:14px;font-weight:600;color:var(--tx);letter-spacing:.3px}
.logo-sub{font-size:10px;color:var(--mu);letter-spacing:.5px;text-transform:uppercase;display:block;margin-top:1px}
.hd{width:1px;height:22px;background:var(--br);flex:none}
.hsp{flex:1}
.hmeta{display:flex;align-items:center;gap:12px;font-size:10px;color:var(--su)}
#hdr-time{color:var(--ac);font-size:12px;letter-spacing:2px;font-variant-numeric:tabular-nums;min-width:70px;text-align:right}
.badge{padding:3px 9px;border-radius:4px;font-size:10px;font-weight:600;letter-spacing:.8px;text-transform:uppercase;border:1px solid transparent}
.IDLE,.AT_TARGET{background:var(--gd);color:var(--gn);border-color:#2a5a33}
.ROTATING_CW,.ROTATING_CCW{background:var(--ad);color:var(--am);border-color:#4a3800}
.LIMIT_HIT{background:var(--rdd);color:var(--rd);border-color:#5a2a2a}
#rb{flex:none;background:var(--ad);border-bottom:1px solid var(--am);color:var(--am);padding:5px 14px;font-size:11px;display:none;justify-content:space-between;align-items:center;animation:rblink 1.2s step-end infinite}
@keyframes rblink{0%,100%{opacity:1}50%{opacity:.6}}
#rb button{background:none;border:none;color:var(--am);cursor:pointer;font-size:14px}
#main{flex:1;display:flex;overflow:hidden;min-height:0}
#pL{flex:none;width:214px;background:var(--sur);border-right:1px solid var(--br);display:flex;flex-direction:column;padding:10px;gap:9px;overflow:hidden}
#pC{flex:none;width:246px;background:var(--bg);border-right:1px solid var(--br);display:flex;flex-direction:column;padding:10px;gap:7px;overflow-y:auto}
#pR{flex:1;min-width:0;display:flex;flex-direction:column;padding:10px;gap:9px;overflow:hidden}
.card{background:var(--sur);border:1px solid var(--br);border-radius:7px;overflow:hidden;flex:none}
.ch{padding:6px 11px;border-bottom:1px solid var(--bs);display:flex;align-items:center;justify-content:space-between;font-size:10px;font-weight:600;color:var(--mu);letter-spacing:.8px;text-transform:uppercase;background:var(--el)}
.cb{padding:9px 11px}
.cmp-card{background:var(--sur);border:1px solid var(--br);border-radius:7px;padding:9px 6px;display:flex;flex-direction:column;align-items:center;gap:7px}
#cmp{display:block;cursor:crosshair;user-select:none}
.brg-val{font-family:Consolas,monospace;font-size:36px;font-weight:700;color:var(--tx);letter-spacing:-2px;line-height:1;font-variant-numeric:tabular-nums}
.brg-unit{font-size:18px;color:var(--mu)}
.brg-sub{font-size:9px;color:var(--su);letter-spacing:1.5px;text-transform:uppercase;margin-top:2px}
.tgt-strip{width:100%;display:flex;justify-content:space-between;align-items:center;padding:5px 7px;background:var(--el);border-radius:5px;border:1px solid var(--bs)}
.tgl{font-size:10px;color:var(--mu);letter-spacing:.5px;text-transform:uppercase}
.tgv{font-family:monospace;font-size:14px;font-weight:600;color:var(--ah)}
.rl-row{display:flex}
.rl{flex:1;padding:7px 9px;display:flex;align-items:center;gap:6px}
.rl+.rl{border-left:1px solid var(--bs)}
.rld{width:8px;height:8px;border-radius:50%;flex:none;transition:background .2s,box-shadow .2s}
.rl.off .rld{background:var(--su)}
.rl.on  .rld{background:var(--rd);box-shadow:0 0 6px var(--rd)}
.rln{font-size:10px;font-weight:600;letter-spacing:.8px;text-transform:uppercase;color:var(--mu)}
.rls{font-size:10px;color:var(--su);font-family:monospace}
.rl.on .rln{color:var(--rd)}
.rl.on .rls{color:var(--am)}
.sec{font-size:10px;font-weight:600;color:var(--mu);letter-spacing:.8px;text-transform:uppercase;padding-bottom:4px;border-bottom:1px solid var(--bs);flex:none;margin-top:1px;display:flex;justify-content:space-between;align-items:flex-end}
.sec-hint{font-size:9px;font-weight:400;color:var(--su);letter-spacing:0;text-transform:none}
#ti{width:100%;padding:7px 9px;border-radius:6px;border:2px solid var(--br);background:var(--el);color:var(--tx);font-family:Consolas,monospace;font-size:28px;font-weight:700;text-align:center;outline:none;transition:border-color .15s;font-variant-numeric:tabular-nums}
#ti:focus{border-color:var(--ac)}
#ti::-webkit-inner-spin-button{opacity:.3}
.cardinals{display:grid;grid-template-columns:repeat(4,1fr);gap:4px}
.cb2{padding:4px 0;border-radius:4px;border:1px solid var(--bs);background:var(--el);color:var(--mu);cursor:pointer;font-family:inherit;display:flex;flex-direction:column;align-items:center;gap:1px;transition:border-color .12s,color .12s}
.cb2:hover{border-color:var(--br);color:var(--tx);background:var(--bs)}
.cd{font-size:11px;font-weight:700;color:var(--tx)}
.cv{font-size:9px;font-family:monospace}
#btnGoto{width:100%;padding:10px;border-radius:6px;border:none;background:var(--ac);color:#fff;font-size:13px;font-weight:600;cursor:pointer;font-family:inherit;letter-spacing:.4px;text-transform:uppercase;display:flex;align-items:center;justify-content:center;gap:9px;transition:background .12s}
#btnGoto:hover{background:var(--ah)}
#btnStop{width:100%;padding:9px;border-radius:6px;border:1px solid rgba(248,81,73,.55);background:var(--rdd);color:var(--rd);font-size:12px;font-weight:700;cursor:pointer;font-family:inherit;letter-spacing:1.2px;text-transform:uppercase;display:flex;align-items:center;justify-content:center;gap:9px;transition:background .12s}
#btnStop:hover{background:#4a1b1b;border-color:var(--rd)}
.kh{font-size:10px;font-weight:400;opacity:.5;letter-spacing:0;background:rgba(255,255,255,.07);border-radius:3px;padding:1px 5px}
.bg2{display:grid;grid-template-columns:1fr 1fr;gap:4px}
.btn{padding:7px 5px;border-radius:6px;border:1px solid var(--br);background:var(--el);color:var(--tx);font-size:11px;font-weight:500;cursor:pointer;font-family:inherit;letter-spacing:.3px;text-transform:uppercase;transition:background .12s;text-align:center}
.btn:hover{background:var(--br)}
.pu{color:var(--pu);border-color:var(--pd);background:var(--pd)}
.pu:hover{background:#3a2a5a;border-color:#6a4aaa}
.am{color:var(--am);border-color:var(--ad);background:var(--ad)}
.am:hover{background:#4a3a00}
.gn{color:var(--gn);border-color:var(--gd);background:var(--gd);width:100%}
.gn:hover{background:#2a5a32}
.pgrid{display:grid;grid-template-columns:1fr 1fr;gap:4px}
.ps{border:1px solid var(--bs);border-radius:6px;min-height:44px;cursor:pointer;display:flex;align-items:center;background:var(--el);transition:border-color .15s}
.ps.empty{justify-content:center;font-size:18px;color:var(--br)}
.ps.empty:hover{color:var(--su);border-color:var(--br)}
.ps.filled{padding:5px 7px;justify-content:space-between;border-color:var(--br)}
.ps.filled:hover{border-color:var(--ac)}
.psi{display:flex;flex-direction:column}
.psn{font-size:10px;font-weight:600}
.psd{font-family:monospace;font-size:13px;font-weight:700;color:var(--ah);margin-top:1px}
.pse{font-size:12px;color:var(--su);padding:2px;flex:none}
.pse:hover{color:var(--tx)}
.dg{display:grid;grid-template-columns:repeat(4,1fr);gap:0;padding:7px 11px}
.di{padding:5px 0;border-bottom:1px solid var(--bs);display:flex;flex-direction:column;gap:2px}
.di:nth-last-child(-n+4){border-bottom:none}
.dl{font-size:9px;color:var(--mu);letter-spacing:.5px;text-transform:uppercase}
.dv{font-family:Consolas,monospace;font-size:12px;font-weight:600;color:var(--cy)}
.dv.ok{color:var(--gn)}
.dv.wn{color:var(--am)}
.dv.er{color:var(--rd)}
#mapWrap{display:none;position:relative;border-top:1px solid var(--bs)}
#mapWrap img{width:100%;height:auto;display:block}
#mapCvs{position:absolute;top:0;left:0;width:100%;height:100%;pointer-events:none}
.tbtn{padding:2px 8px;background:var(--el);border:1px solid var(--br);border-radius:4px;font-size:10px;font-weight:500;color:var(--mu);cursor:pointer;font-family:inherit}
.tbtn:hover{color:var(--tx);border-color:var(--mu)}
.lcard{flex:1;min-height:0;display:flex;flex-direction:column;background:var(--sur);border:1px solid var(--br);border-radius:7px;overflow:hidden}
#log{flex:1;min-height:0;background:var(--bg);padding:7px 9px;overflow-y:auto;font-family:Consolas,monospace;font-size:11px;color:var(--mu)}
#log p{margin:1px 0;line-height:1.5}
.lt{color:var(--su);margin-right:5px;user-select:none;font-size:10px}
.lx{color:#d2a8ff}
.lr{color:var(--gn)}
.le{color:var(--rd)}
.li{color:var(--cy)}
.lftr{flex:none;border-top:1px solid var(--bs);padding:5px 9px;display:flex;gap:5px}
.lftr input{flex:1;padding:5px 7px;border-radius:5px;border:1px solid var(--br);background:var(--el);color:var(--tx);font-family:Consolas,monospace;font-size:11px;outline:none}
.lftr input:focus{border-color:var(--ac)}
.lftr input::placeholder{color:var(--su)}
.lftr .sbtn{padding:5px 12px;background:var(--ac);color:#fff;border:none;border-radius:5px;font-size:11px;font-weight:600;cursor:pointer;font-family:inherit}
.la{display:flex;gap:4px}
.la button{padding:1px 7px;background:var(--el);border:1px solid var(--br);border-radius:4px;font-size:10px;color:var(--mu);cursor:pointer;font-family:inherit}
.la button:hover{color:var(--tx);border-color:var(--mu)}
#pmask{display:none;position:fixed;inset:0;background:rgba(1,4,9,.75);z-index:999;align-items:center;justify-content:center}
#pmask.vis{display:flex}
#pmod{background:var(--sur);border:1px solid var(--br);border-radius:9px;box-shadow:0 16px 48px rgba(0,0,0,.6);padding:18px;min-width:252px}
#pmod .pmt{font-size:13px;font-weight:600;margin-bottom:12px}
#pmod label{display:block;font-size:10px;color:var(--mu);letter-spacing:.5px;text-transform:uppercase;margin-bottom:4px;margin-top:10px}
#pmod label:first-of-type{margin-top:0}
#pmod input{width:100%;padding:7px 9px;border-radius:5px;border:1px solid var(--br);background:var(--el);color:var(--tx);font-family:monospace;font-size:15px;outline:none}
#pmod input:focus{border-color:var(--ac)}
.pmb{display:flex;gap:6px;margin-top:14px}
#pmOk{flex:1;padding:8px;background:var(--ac);color:#fff;border:none;border-radius:5px;font-size:12px;font-weight:600;cursor:pointer;font-family:inherit}
#pmOk:hover{background:var(--ah)}
#pmCn{padding:8px 14px;background:var(--el);color:var(--mu);border:1px solid var(--br);border-radius:5px;font-size:12px;cursor:pointer;font-family:inherit}
#pmCn:hover{color:var(--tx)}
::-webkit-scrollbar{width:5px;height:5px}
::-webkit-scrollbar-track{background:transparent}
::-webkit-scrollbar-thumb{background:var(--br);border-radius:4px}
::-webkit-scrollbar-thumb:hover{background:var(--su)}
</style>
</head>
<body>
<div id="hdr">
  <div class="logo">
    <div class="logo-ic">&#x2299;</div>
    <div>
      <div class="logo-name">OpenRotator</div>
      <span class="logo-sub">NL-01 &middot; Azimuth Controller</span>
    </div>
  </div>
  <div class="hd"></div>
  <span id="sts2"></span>
  <div class="hsp"></div>
  <div class="hmeta">
    <span>EM77EB &nbsp;&middot;&nbsp; 38.032&deg;N 85.347&deg;W</span>
    <span id="hdr-time">00:00:00Z</span>
  </div>
</div>
<div id="rb">
  &#9888;&nbsp; Heading restored from NVS &mdash; verify antenna position before GOTO
  <button onclick="dismissRb()">&#x2715;</button>
</div>
<div id="main">
  <aside id="pL">
    <div class="cmp-card">
      <canvas id="cmp" width="186" height="186" title="Tap to set target heading"></canvas>
      <div style="text-align:center">
        <div><span class="brg-val" id="hdg">---</span><span class="brg-unit">&deg;</span></div>
        <div class="brg-sub">Bearing</div>
      </div>
      <div class="tgt-strip">
        <span class="tgl">Target</span>
        <span class="tgv" id="tgt">---&deg;</span>
      </div>
    </div>
    <div class="card">
      <div class="rl-row">
        <div class="rl off" id="relayA">
          <div class="rld"></div>
          <div><div class="rln">CW</div><div class="rls" id="rAs">OFF</div></div>
        </div>
        <div class="rl off" id="relayB">
          <div class="rld"></div>
          <div><div class="rln">CCW</div><div class="rls" id="rBs">OFF</div></div>
        </div>
      </div>
    </div>
  </aside>
  <div id="pC">
    <div class="sec">Navigate</div>
    <input type="number" id="ti" value="180" min="0" max="360" onkeydown="onTiKey(event)">
    <div class="cardinals">
      <button class="cb2" onclick="qSet(0)"><span class="cd">N</span><span class="cv">0&deg;</span></button>
      <button class="cb2" onclick="qSet(90)"><span class="cd">E</span><span class="cv">90&deg;</span></button>
      <button class="cb2" onclick="qSet(180)"><span class="cd">S</span><span class="cv">180&deg;</span></button>
      <button class="cb2" onclick="qSet(270)"><span class="cd">W</span><span class="cv">270&deg;</span></button>
    </div>
    <button id="btnGoto" onclick="sendGotoVal(-1)">GOTO <span class="kh">Enter &#x23CE;</span></button>
    <button id="btnStop" onclick="doStop()">&#9632; STOP &nbsp;<span class="kh">Esc</span></button>
    <div class="sec" style="margin-top:3px">Manual Rotation</div>
    <div class="bg2">
      <button class="btn pu" onclick="doCW()">&#9664; CW</button>
      <button class="btn pu" onclick="doCCW()">CCW &#9654;</button>
      <button class="btn am" onclick="doPost('/api/timecw','TIMECW')">&#9201; T-CW</button>
      <button class="btn am" onclick="doPost('/api/timeccw','TIMECCW')">&#9201; T-CCW</button>
    </div>
    <div class="sec" style="margin-top:3px">
      Stored Bearings
      <span class="sec-hint">tap = GOTO &nbsp;&middot;&nbsp; &#x270E; = edit</span>
    </div>
    <div class="pgrid" id="prs"></div>
    <div class="sec" style="margin-top:3px">Calibration</div>
    <button class="btn gn" onclick="doSethome()">Set Home</button>
  </div>
  <div id="pR">
    <div class="card">
      <div class="ch">Diagnostics</div>
      <div class="dg">
        <div class="di"><span class="dl">Runtime</span><span class="dv" id="dRt">---</span></div>
        <div class="di"><span class="dl">Uptime</span><span class="dv" id="dUp">---</span></div>
        <div class="di"><span class="dl">MS/Deg</span><span class="dv" id="dMd">---</span></div>
        <div class="di"><span class="dl">Deadband</span><span class="dv" id="dDb">---</span></div>
        <div class="di"><span class="dl">Known</span><span class="dv er" id="dKn">NO</span></div>
        <div class="di"><span class="dl">Timing</span><span class="dv" id="dTm">OFF</span></div>
        <div class="di"><span class="dl">Min Hdg</span><span class="dv" id="dMn">0&deg;</span></div>
        <div class="di"><span class="dl">Max Hdg</span><span class="dv" id="dMx">360&deg;</span></div>
      </div>
    </div>
    <div class="card">
      <div class="ch">Area Map <button class="tbtn" id="btnMapTog" onclick="toggleMap()">Show</button></div>
      <div id="mapWrap">
        <img id="mapImg" src="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/export?bbox=-87.5%2C36.5%2C-83.2%2C40.0&bboxSR=4326&size=900%2C560&format=png&f=image" alt="" onload="drawMapOverlay()">
        <canvas id="mapCvs"></canvas>
      </div>
    </div>
    <div class="lcard">
      <div class="ch">Comm Log
        <div class="la">
          <button onclick="cl()">Clear</button>
          <button onclick="exportLog()">Export</button>
        </div>
      </div>
      <div id="log"></div>
      <div class="lftr">
        <input id="ci" placeholder="GOTO 180 | MSPERDEG 163 | SETMIN 0 | HELP" onkeydown="ok(event)">
        <button class="sbtn" onclick="dc()">Send</button>
      </div>
    </div>
  </div>
</div>
<div id="pmask">
  <div id="pmod">
    <div class="pmt">Edit Preset</div>
    <label>Label</label>
    <input type="text" id="pmLbl" maxlength="8" autocomplete="off">
    <label>Heading (0&ndash;360&deg;)</label>
    <input type="number" id="pmDeg" min="0" max="360">
    <div class="pmb">
      <button id="pmOk" onclick="pmSave()">Save</button>
      <button id="pmCn" onclick="pmClose()">Cancel</button>
    </div>
  </div>
</div>
<script>
const W=186,CX=93,CY=93,R=84;
let heading=0,target=-1,hdgMin=0,hdgMax=360;
let displayHeading=0,animId=null;
let cmdHist=[],cmdIdx=-1,mapOpen=false,restoredDismissed=false;
const MAP_LAT=38.0344609,MAP_LON=-85.3316739;
const MAP_W=-87.5,MAP_E=-83.2,MAP_S=36.5,MAP_N=40.0;

function drawCompass(h,t){
  const cvs=document.getElementById('cmp'),c=cvs.getContext('2d');
  c.clearRect(0,0,W,W);
  c.beginPath();c.arc(CX,CY,R+2,0,Math.PI*2);c.strokeStyle='rgba(255,255,255,.04)';c.lineWidth=4;c.stroke();
  const g=c.createRadialGradient(CX,CY-20,10,CX,CY,R);
  g.addColorStop(0,'#1e2535');g.addColorStop(1,'#0d1117');
  c.beginPath();c.arc(CX,CY,R,0,Math.PI*2);c.fillStyle=g;c.fill();c.strokeStyle='#30363d';c.lineWidth=1.5;c.stroke();
  if(hdgMax-hdgMin<355){
    const sa=((hdgMax-90+720)%360)*Math.PI/180,ea=((hdgMin-90+720)%360)*Math.PI/180;
    c.beginPath();c.moveTo(CX,CY);c.arc(CX,CY,R-1,sa,ea);c.closePath();c.fillStyle='rgba(13,17,23,.7)';c.fill();
    [hdgMin,hdgMax].forEach(d=>{
      const a=(d-90)*Math.PI/180;
      c.beginPath();c.moveTo(CX+68*Math.cos(a),CY+68*Math.sin(a));c.lineTo(CX+83*Math.cos(a),CY+83*Math.sin(a));
      c.strokeStyle='#d29922';c.lineWidth=2.5;c.stroke();
    });
  }
  if(t>=0&&Math.abs(h-t)>0.5){
    const sA=(h-90)*Math.PI/180,eA=(t-90)*Math.PI/180;
    c.beginPath();c.arc(CX,CY,72,sA,eA,t<h);c.strokeStyle='rgba(47,129,247,.5)';c.lineWidth=5;c.stroke();
  }
  for(let i=0;i<360;i+=5){
    const a=(i-90)*Math.PI/180,card=i%90===0,maj=i%45===0,m10=i%10===0;
    const ir=card?66:maj?70:m10?72:74;
    c.beginPath();c.moveTo(CX+ir*Math.cos(a),CY+ir*Math.sin(a));c.lineTo(CX+83*Math.cos(a),CY+83*Math.sin(a));
    c.strokeStyle=card?'#484f58':maj?'#30363d':'#21262d';c.lineWidth=card?1.5:1;c.stroke();
  }
  [['N',0,'#e6edf3'],['E',90,'#7d8590'],['S',180,'#7d8590'],['W',270,'#7d8590']].forEach(([l,d,col])=>{
    const a=(d-90)*Math.PI/180;
    c.fillStyle=col;c.font=l==='N'?'bold 11px system-ui':'10px system-ui';
    c.textAlign='center';c.textBaseline='middle';
    c.fillText(l,CX+54*Math.cos(a),CY+54*Math.sin(a));
  });
  [['NE',45],['SE',135],['SW',225],['NW',315]].forEach(([l,d])=>{
    const a=(d-90)*Math.PI/180;
    c.fillStyle='#484f58';c.font='8px system-ui';c.textAlign='center';c.textBaseline='middle';
    c.fillText(l,CX+54*Math.cos(a),CY+54*Math.sin(a));
  });
  if(t>=0){
    const ta=(t-90)*Math.PI/180;
    c.beginPath();c.moveTo(CX,CY);c.lineTo(CX+R*.82*Math.cos(ta),CY+R*.82*Math.sin(ta));
    c.strokeStyle='rgba(47,129,247,.6)';c.lineWidth=1.5;c.setLineDash([4,3]);c.stroke();c.setLineDash([]);
    c.beginPath();c.arc(CX+R*.82*Math.cos(ta),CY+R*.82*Math.sin(ta),3.5,0,Math.PI*2);
    c.fillStyle='rgba(47,129,247,.85)';c.fill();
  }
  const na=(h-90)*Math.PI/180;
  c.beginPath();c.moveTo(CX-18*Math.cos(na),CY-18*Math.sin(na));c.lineTo(CX,CY);
  c.strokeStyle='#30363d';c.lineWidth=2.5;c.stroke();
  c.shadowColor='#f85149';c.shadowBlur=7;
  c.beginPath();c.moveTo(CX,CY);c.lineTo(CX+64*Math.cos(na),CY+64*Math.sin(na));
  c.strokeStyle='#f85149';c.lineWidth=2.5;c.stroke();c.shadowBlur=0;
  c.beginPath();c.arc(CX,CY,5,0,Math.PI*2);c.fillStyle='#2f81f7';c.fill();
  c.beginPath();c.arc(CX,CY,3,0,Math.PI*2);c.fillStyle='#e6edf3';c.fill();
}

function startAnim(){
  if(animId)return;
  function step(){
    const d=heading-displayHeading;
    if(Math.abs(d)>90){displayHeading=heading;drawCompass(displayHeading,target);drawMapOverlay();animId=null;return;}
    if(Math.abs(d)>0.2){displayHeading+=d*.14;drawCompass(displayHeading,target);animId=requestAnimationFrame(step);}
    else{displayHeading=heading;drawCompass(displayHeading,target);drawMapOverlay();animId=null;}
  }
  animId=requestAnimationFrame(step);
}

document.getElementById('cmp').addEventListener('click',e=>{
  const rect=e.target.getBoundingClientRect(),sc=rect.width/W;
  let deg=Math.atan2(e.clientY-rect.top-CY*sc,e.clientX-rect.left-CX*sc)*180/Math.PI+90;
  if(deg<0)deg+=360;
  deg=Math.min(hdgMax,Math.max(hdgMin,Math.round(deg)%360));
  document.getElementById('ti').value=deg;
  const ti=document.getElementById('ti');ti.style.borderColor='var(--ac)';setTimeout(()=>ti.style.borderColor='',600);
});

const NP=4;
function lps(){try{return JSON.parse(localStorage.getItem('rtp')||'[]');}catch{return[];}}
function sps(a){localStorage.setItem('rtp',JSON.stringify(a));}
function rndPre(){
  const ps=lps(),el=document.getElementById('prs');el.innerHTML='';
  for(let i=0;i<NP;i++){
    const p=ps[i],d=document.createElement('div');
    if(p){
      d.className='ps filled';d.title='GOTO '+p.d+'deg';
      d.innerHTML='<div class="psi" onclick="gtp('+i+')" style="cursor:pointer"><span class="psn">'+p.l+'</span><span class="psd">'+p.d+'&deg;</span></div><span class="pse" onclick="event.stopPropagation();stp('+i+')" title="Edit">&#x270E;</span>';
    }else{
      d.className='ps empty';d.title='Add preset';d.onclick=()=>stp(i);d.textContent='+';
    }
    el.appendChild(d);
  }
}
function gtp(i){const ps=lps();if(ps[i]){document.getElementById('ti').value=ps[i].d;sendGotoVal(ps[i].d,ps[i].l);}}
function stp(i){
  const ps=lps(),cur=ps[i];
  const dv=parseFloat(document.getElementById('ti').value);
  document.getElementById('pmLbl').value=cur?cur.l:'P'+(i+1);
  document.getElementById('pmDeg').value=isNaN(dv)?(cur?cur.d:0):Math.round(dv);
  document.getElementById('pmask')._slot=i;
  document.getElementById('pmask').classList.add('vis');
  setTimeout(()=>document.getElementById('pmLbl').select(),50);
}
function pmClose(){document.getElementById('pmask').classList.remove('vis');}
function pmSave(){
  const i=document.getElementById('pmask')._slot,ps=lps();
  const nl=(document.getElementById('pmLbl').value.trim().slice(0,8)||('P'+(i+1)));
  const nd=parseInt(document.getElementById('pmDeg').value)||0;
  ps[i]={l:nl,d:nd};sps(ps);rndPre();pmClose();
  lg('Preset '+(i+1)+' saved: '+nl+' / '+nd+'deg','lx');
}

async function api(url,method,body){
  const o={method:method||'GET'};
  if(body){o.headers={'Content-Type':'application/json'};o.body=JSON.stringify(body);}
  return(await fetch(url,o)).json();
}

async function pollStatus(){
  try{
    const d=await api('/api/status');
    const hv=d.headingKnown?parseFloat(d.heading):heading;
    if(!isNaN(hv))heading=hv;
    target=parseFloat(d.target);
    startAnim();
    document.getElementById('hdg').textContent=d.headingKnown?hv.toFixed(1):'---';
    document.getElementById('tgt').textContent=target>=0?target.toFixed(1)+'&deg;':'---&deg;';
    const st=d.status||'IDLE';
    document.getElementById('sts2').innerHTML='<span class="badge '+st+'">'+st.replace('_',' ')+'</span>';
    const cOn=d.pinCW===0,ccOn=d.pinCCW===0;
    document.getElementById('relayA').className='rl '+(cOn?'on':'off');
    document.getElementById('rAs').textContent=cOn?'ON':'OFF';
    document.getElementById('relayB').className='rl '+(ccOn?'on':'off');
    document.getElementById('rBs').textContent=ccOn?'ON':'OFF';
    document.getElementById('dRt').textContent=fMs(d.motorRunMs||0);
    document.getElementById('dUp').textContent=fUp(d.uptime||0);
    document.getElementById('dMd').textContent=parseFloat(d.msperdeg||0).toFixed(1);
    document.getElementById('dDb').textContent=parseFloat(d.deadband||0).toFixed(1)+'&deg;';
    const kEl=document.getElementById('dKn');kEl.textContent=d.headingKnown?'YES':'NO';kEl.className='dv '+(d.headingKnown?'ok':'er');
    const tEl=document.getElementById('dTm');tEl.textContent=d.timingMode?'ON':'OFF';tEl.className='dv '+(d.timingMode?'wn':'');
    if(d.hdgMin!==undefined){
      hdgMin=parseFloat(d.hdgMin);hdgMax=parseFloat(d.hdgMax);
      document.getElementById('ti').min=hdgMin;document.getElementById('ti').max=hdgMax;
      document.getElementById('dMn').textContent=hdgMin.toFixed(0)+'&deg;';
      document.getElementById('dMx').textContent=hdgMax.toFixed(0)+'&deg;';
    }
    const rb=document.getElementById('rb');
    if(d.hdgRestored&&!restoredDismissed)rb.style.display='flex';
    else if(!d.hdgRestored){restoredDismissed=false;rb.style.display='none';}
  }catch(e){}
}

function fMs(ms){if(!ms||ms<=0)return'0ms';if(ms<1000)return ms+'ms';if(ms<60000)return(ms/1000).toFixed(1)+'s';return Math.floor(ms/60000)+'m'+(Math.floor(ms/1000)%60)+'s';}
function fUp(ms){const s=Math.floor(ms/1000),h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sc=s%60;return(h?h+'h ':'')+( m?m+'m ':'')+sc+'s';}

async function sendGotoVal(deg,label){
  if(deg<0){deg=parseFloat(document.getElementById('ti').value);if(isNaN(deg))return;}
  deg=Math.round(Math.min(hdgMax,Math.max(hdgMin,deg)));
  const body={heading:deg};if(label)body.label=label;
  try{const d=await api('/api/goto','POST',body);lg('GOTO '+deg+'deg -> '+d.result,'lr');}
  catch(e){lg('GOTO error','le');}
}

async function doStop(){
  try{
    const d=await api('/api/stop','POST');
    let msg='STOP -> '+d.result;
    if(d.timedMs)msg+=' | '+d.timedMs+'ms -> MSPERDEG '+d.suggestedMsDeg;
    lg(msg,'lr');
  }catch(e){}
}

async function doPost(url,label){
  try{const d=await api(url,'POST');lg(label+' -> '+d.result,'lr');}catch(e){}
}

async function doCW(){
  try{await api('/api/stop','POST');}catch(e){}
  try{const d=await api('/api/cw','POST');lg('CW -> '+d.result,'lr');}catch(e){}
}

async function doCCW(){
  try{await api('/api/stop','POST');}catch(e){}
  try{const d=await api('/api/ccw','POST');lg('CCW -> '+d.result,'lr');}catch(e){}
}

async function doSethome(){
  const deg=parseInt(document.getElementById('ti').value,10);
  const val=isNaN(deg)?0:Math.min(hdgMax,Math.max(hdgMin,deg));
  try{const d=await api('/api/sethome','POST',{heading:val});lg('SETHOME '+val+'deg -> '+d.result,'lr');}catch(e){}
}

function qSet(deg){
  document.getElementById('ti').value=deg;
  const ti=document.getElementById('ti');ti.style.borderColor='var(--ac)';setTimeout(()=>ti.style.borderColor='',500);
}
function onTiKey(e){if(e.key==='Enter'){e.preventDefault();sendGotoVal(-1);}}

async function dc(){
  const ci=document.getElementById('ci'),cmd=ci.value.trim();if(!cmd)return;
  ci.value='';cmdHist.unshift(cmd);if(cmdHist.length>50)cmdHist.pop();cmdIdx=-1;
  try{const d=await api('/api/command','POST',{cmd:cmd});lg('> '+cmd,'lx');lg(d.result,'lr');}
  catch(e){lg('ERR: '+e,'le');}
}
function ok(e){
  if(e.key==='Enter'){dc();return;}
  if(e.key==='ArrowUp'){if(!cmdHist.length)return;cmdIdx=Math.min(cmdIdx+1,cmdHist.length-1);document.getElementById('ci').value=cmdHist[cmdIdx];e.preventDefault();}
  if(e.key==='ArrowDown'){cmdIdx=Math.max(cmdIdx-1,-1);document.getElementById('ci').value=cmdIdx<0?'':cmdHist[cmdIdx];e.preventDefault();}
}

document.addEventListener('keydown',e=>{
  const inInput=e.target.tagName==='INPUT';
  if(e.key==='Escape'){if(document.getElementById('pmask').classList.contains('vis')){pmClose();return;}doStop();return;}
  if(e.key==='Enter'&&!inInput){e.preventDefault();sendGotoVal(-1);}
});

function lg(msg,cls){
  const el=document.getElementById('log'),p=document.createElement('p');
  const t=new Date();
  const ts=String(t.getUTCHours()).padStart(2,'0')+String(t.getUTCMinutes()).padStart(2,'0')+'Z';
  p.innerHTML='<span class="lt">'+ts+'</span><span class="'+(cls||'li')+'"> '+msg+'</span>';
  el.appendChild(p);el.scrollTop=el.scrollHeight;
  if(el.children.length>100)el.removeChild(el.firstChild);
}
function cl(){document.getElementById('log').innerHTML='';}
function exportLog(){
  const lines=[...document.getElementById('log').querySelectorAll('p')].map(p=>p.textContent);
  const a=Object.assign(document.createElement('a'),{href:URL.createObjectURL(new Blob([lines.join('\n')],{type:'text/plain'})),download:'rotator-'+new Date().toISOString().slice(0,10)+'.txt'});
  a.click();
}
function dismissRb(){restoredDismissed=true;document.getElementById('rb').style.display='none';}

function toggleMap(){
  mapOpen=!mapOpen;
  document.getElementById('mapWrap').style.display=mapOpen?'block':'none';
  document.getElementById('btnMapTog').textContent=mapOpen?'Hide':'Show';
  if(mapOpen)setTimeout(drawMapOverlay,60);
}
function ll2px(lat,lon,W,H){return[(lon-MAP_W)/(MAP_E-MAP_W)*W,(MAP_N-lat)/(MAP_N-MAP_S)*H];}
function drawMapOverlay(){
  if(!mapOpen)return;
  const img=document.getElementById('mapImg'),cvs=document.getElementById('mapCvs');
  if(!img||!img.complete||!img.naturalWidth)return;
  const IW=img.offsetWidth,IH=img.offsetHeight;if(!IW||!IH)return;
  cvs.width=IW;cvs.height=IH;
  const c=cvs.getContext('2d');c.clearRect(0,0,IW,IH);
  const[hx,hy]=ll2px(MAP_LAT,MAP_LON,IW,IH);
  const pxMi=IH/(MAP_N-MAP_S)/69.0;
  [50,100,150,200].forEach(mi=>{
    const r=mi*pxMi;
    c.beginPath();c.arc(hx,hy,r,0,Math.PI*2);c.strokeStyle='rgba(47,129,247,.3)';c.lineWidth=1;c.setLineDash([5,4]);c.stroke();c.setLineDash([]);
    const rx=hx+r*.707,ry=hy-r*.707;
    c.font='9px monospace';c.textAlign='left';c.textBaseline='middle';c.fillStyle='rgba(47,129,247,.75)';c.fillText(mi+' mi',rx+1,ry);
  });
  const rad=displayHeading*Math.PI/180,blen=Math.hypot(IW,IH);
  c.beginPath();c.moveTo(hx,hy);c.lineTo(hx+blen*Math.sin(rad),hy-blen*Math.cos(rad));
  c.strokeStyle='rgba(248,81,73,.85)';c.lineWidth=2;c.stroke();
  c.beginPath();c.arc(hx,hy,6,0,Math.PI*2);c.fillStyle='#2f81f7';c.fill();c.strokeStyle='#e6edf3';c.lineWidth=2;c.stroke();
}

rndPre();
drawCompass(0,-1);
pollStatus();
setInterval(pollStatus,500);
setInterval(()=>{
  const t=new Date();
  document.getElementById('hdr-time').textContent=String(t.getUTCHours()).padStart(2,'0')+':'+String(t.getUTCMinutes()).padStart(2,'0')+':'+String(t.getUTCSeconds()).padStart(2,'0')+'Z';
},1000);
</script>
</body>
</html>)rawliteral";


// =============================================================================
//  Web server handlers
// =============================================================================
// Always add Connection:close so the ESP32 LwIP TCP stack doesn't accumulate
// sockets in TIME_WAIT — without this, 500 ms polling exhausts the PCB table
// and the web interface goes dead without a reboot.
static void jSend(int code, const String& body, const char* ct = "application/json") {
    server.sendHeader("Connection", "close");
    server.send(code, ct, body);
}

void handleRoot() {
    server.sendHeader("Connection", "close");
    server.send_P(200, "text/html", INDEX_HTML);
}

void handleStatus() {
    StaticJsonDocument<640> doc;
    // Apply mount offset so UI always shows compass bearings
    auto toDisplay = [](float phys) { return fmodf(phys + HDG_OFFSET + 360.0f, 360.0f); };
    doc["heading"]      = headingKnown ? serialized(String(toDisplay(currentHeading), 2)) : serialized(String("-1"));
    doc["target"]       = serialized(String(targetHeading >= 0.0f ? toDisplay(targetHeading) : targetHeading, 2));
    doc["hdgOffset"]    = serialized(String(HDG_OFFSET, 1));
    doc["status"]       = rotatorStatus;
    doc["headingKnown"] = headingKnown;
    doc["msperdeg"]     = serialized(String(MS_PER_DEG, 1));
    doc["deadband"]     = serialized(String(DEADBAND, 1));
    doc["pinCW"]        = (int)digitalRead(PIN_MOTOR_CW);
    doc["pinCCW"]       = (int)digitalRead(PIN_MOTOR_CCW);
    doc["motorRunMs"]   = (motorDirection != 0) ? (unsigned long)(millis() - motorStartMs) : 0UL;
    doc["uptime"]       = (unsigned long)millis();
    doc["timingMode"]   = timingMode;
    doc["hdgMin"]       = serialized(String(HDG_MIN, 1));
    doc["hdgMax"]       = serialized(String(HDG_MAX, 1));
    doc["hdgRestored"]  = hdgRestored;
    String json;
    serializeJson(doc, json);
    jSend(200, json);
}

void handleGoto() {
    if (!server.hasArg("plain")) { jSend(400, "{\"error\":\"no body\"}"); return; }
    if (!headingKnown) {
        jSend(200, "{\"result\":\"ERR HEADING_UNKNOWN — run TIMECCW to end stop then STOP to home first\"}");
        return;
    }
    StaticJsonDocument<128> doc;
    if (deserializeJson(doc, server.arg("plain"))) { jSend(400, "{\"error\":\"bad json\"}"); return; }
    float h = doc["heading"].as<float>();
    // Optional preset label — displayed on LCD line 1 during rotation
    if (doc.containsKey("label")) {
        String lbl = doc["label"].as<String>();
        lbl.trim(); lbl.toUpperCase();
        currentTargetLabel = lbl.substring(0, 6);
    } else {
        currentTargetLabel = "";
    }
    // Input is a compass bearing; convert to physical heading by subtracting offset
    float physH = fmodf(h - HDG_OFFSET + 360.0f, 360.0f);
    physH = fmaxf(HDG_MIN, fminf(HDG_MAX, physH));  // clamp to physical limits
    freeRunMode   = false;
    targetHeading = physH;
    jSend(200, "{\"result\":\"OK\",\"target\":" + String(h, 1) + "}");
}

void handleStop() {
    String resp = "{\"result\":\"STOPPED\"";
    int prevDir = motorDirection;  // capture before motorStop() clears it
    if (timingMode) {
        unsigned long elapsed = millis() - timingStartMs;
        timingMode = false;
        float sweep = HDG_MAX - HDG_MIN;
        resp += ",\"timedMs\":" + String(elapsed);
        if (sweep > 0.0f)
            resp += ",\"suggestedMsDeg\":" + String(elapsed / sweep, 1) +
                    ",\"sweepDeg\":" + String(sweep, 0);
        // Auto-home: CCW end stop = HDG_MIN (0°) — only set on CCW run
        if (prevDir == -1) {
            float homePos = HDG_MIN;
            currentHeading = homePos;
            motorStartHdg  = homePos;
            headingKnown   = true;
            hdgRestored    = false;
            prefs.putFloat("lastheading", homePos);
            prefs.putBool("hdgknown", true);
            resp += ",\"autoHome\":" + String(homePos, 1);
        }
    }
    resp += "}";
    freeRunMode        = false;
    targetHeading      = -1.0f;
    currentTargetLabel = "";
    motorStop();
    // Force an immediate NVS heading write on explicit STOP (bypass throttle)
    if (headingKnown) {
        prefs.putFloat("lastheading", currentHeading);
        lastNvsHdgWriteMs = millis();
    }
    jSend(200, resp);
}

void handleCommand() {
    if (!server.hasArg("plain")) { jSend(400, ""); return; }
    StaticJsonDocument<96> doc;
    if (deserializeJson(doc, server.arg("plain"))) { jSend(400, ""); return; }
    String cmd    = doc["cmd"].as<String>();
    String result = processCommand(cmd);
    StaticJsonDocument<384> resp;
    resp["result"] = result;
    String json;
    serializeJson(resp, json);
    jSend(200, json);
}

void handleNotFound() {
    jSend(404, "Not found", "text/plain");
}

void handleCW() {
    targetHeading      = -1.0f;
    freeRunMode        = true;
    currentTargetLabel = "";
    motorCW();  // motorCW() calls motorStop() + delay(200) interlock internally
    jSend(200, "{\"result\":\"ROTATING_CW\"}");
}

void handleCCW() {
    targetHeading      = -1.0f;
    freeRunMode        = true;
    currentTargetLabel = "";
    motorCCW();  // motorCCW() calls motorStop() + delay(200) interlock internally
    jSend(200, "{\"result\":\"ROTATING_CCW\"}");
}

void handleSethome() {
    float deg = 0.0f;
    if (server.hasArg("plain")) {
        StaticJsonDocument<64> doc;
        if (!deserializeJson(doc, server.arg("plain"))) {
            deg = doc["heading"].as<float>();
            deg = fmaxf(HDG_MIN, fminf(HDG_MAX, deg));  // clamp to limits
        }
    }
    motorStop();
    currentHeading = deg;
    motorStartHdg  = deg;
    headingKnown   = true;
    hdgRestored    = false;
    prefs.putFloat("lastheading", deg);
    prefs.putBool("hdgknown", true);
    jSend(200, "{\"result\":\"HOME_SET=" + String(deg, 1) + "\"}");
}

void handleTimeCW() {
    targetHeading = -1.0f;
    freeRunMode   = true;
    timingMode    = true;
    timingStartMs = millis();
    motorCW();
    jSend(200, "{\"result\":\"TIMING_CW\"}");
}

void handleTimeCCW() {
    targetHeading = -1.0f;
    freeRunMode   = true;
    timingMode    = true;
    timingStartMs = millis();
    motorCCW();
    jSend(200, "{\"result\":\"TIMING_CCW\"}");
}

// =============================================================================
//  Setup
// =============================================================================
void setup() {
    // SAFETY FIRST — de-energise relay outputs before anything else.
    // digitalWrite before pinMode pre-stages the output latch so the pin
    // comes up HIGH (relay OFF) the instant pinMode sets it to OUTPUT,
    // eliminating any brief LOW glitch that could pulse the relay on boot.
    digitalWrite(PIN_MOTOR_CW,  HIGH);
    digitalWrite(PIN_MOTOR_CCW, HIGH);

    // Relay inrush current can briefly dip the rail below the ESP32 brownout
    // threshold (~2.43 V), causing a spurious reset.  Safe to disable on USB power.
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    Serial.begin(115200);
    delay(100);
    Serial.println(F("\n\n=== nlOpenRotator ==="));

    // Log reset reason — helps diagnose power/brownout issues.
    // NOTE: brownout detector is disabled above, so ESP_RST_BROWNOUT means
    // the hardware threshold was hit before firmware ran.  ESP_RST_POWERON
    // after an unexpected reset means the rail fully collapsed (not a soft brownout).
    {
        esp_reset_reason_t reason = esp_reset_reason();
        const char* rrStr = "UNKNOWN";
        switch (reason) {
            case ESP_RST_POWERON:   rrStr = "POWERON";   break;
            case ESP_RST_EXT:      rrStr = "EXT_PIN";   break;
            case ESP_RST_SW:       rrStr = "SOFTWARE";  break;
            case ESP_RST_PANIC:    rrStr = "PANIC";     break;
            case ESP_RST_INT_WDT:  rrStr = "INT_WDT";   break;
            case ESP_RST_TASK_WDT: rrStr = "TASK_WDT";  break;
            case ESP_RST_WDT:      rrStr = "WDT";       break;
            case ESP_RST_BROWNOUT: rrStr = "BROWNOUT";  break;
            case ESP_RST_SDIO:     rrStr = "SDIO";      break;
            default: break;
        }
        Serial.printf("RESET REASON: %s (%d)\n", rrStr, (int)reason);
    }

    // Load calibration and last known heading from NVS (survives reboots)
    prefs.begin("rotator", false);
    prefsReady = true;
    MS_PER_DEG = prefs.getFloat("msperdeg",  163.0f);
    HDG_MIN    = prefs.getFloat("hdgmin",    0.0f);
    HDG_MAX    = prefs.getFloat("hdgmax",    360.0f);
    DEADBAND   = prefs.getFloat("deadband",  4.0f);
    HDG_OFFSET = prefs.getFloat("hdgoffset", 0.0f);
    if (prefs.getBool("hdgknown", false)) {
        if (prefs.getBool("motordirty", false)) {
            // Motor was running when power was lost — restored heading is unreliable.
            // Invalidate it and require user to re-home before GOTO is re-enabled.
            prefs.putBool("hdgknown",   false);
            prefs.putBool("motordirty", false);
            Serial.println(F("WARN: Rebooted mid-rotation — heading invalidated. Run TIMECCW+STOP to re-home."));
        } else {
            currentHeading = prefs.getFloat("lastheading", 0.0f);
            motorStartHdg  = currentHeading;
            headingKnown   = true;
            hdgRestored    = true;
            Serial.printf("NVS: heading restored to %.1f deg\n", currentHeading);
        }
    } else {
        prefs.putBool("motordirty", false);  // clear any stale dirty flag
    }
    Serial.printf("NVS: MSPERDEG=%.1f  MIN=%.1f  MAX=%.1f  DB=%.1f\n",
                  MS_PER_DEG, HDG_MIN, HDG_MAX, DEADBAND);
    staSSID = prefs.getString("wifissid", "");
    staPass = prefs.getString("wifipass", "");

    pinMode(PIN_LED, OUTPUT);

    // Relay outputs — start with both relays off (HIGH = off on active-LOW module)
    pinMode(PIN_MOTOR_CW,  OUTPUT);
    pinMode(PIN_MOTOR_CCW, OUTPUT);
    motorStop();

    // LCD — auto-detect I2C address: tries 0x27 (PCF8574) then 0x3F (PCF8574A)
    Wire.begin();
    {
        const uint8_t lcdAddrs[] = {0x27, 0x3F};
        for (uint8_t a : lcdAddrs) {
            Wire.beginTransmission(a);
            if (Wire.endTransmission() == 0) {
                lcd = new LiquidCrystal_I2C(a, 16, 2);
                lcd->init();
                lcd->backlight();
                lcd->setCursor(0, 0); lcd->print("OpenRotator");
                lcd->setCursor(0, 1); lcd->print("Initializing...");
                lcdPresent = true;
                Serial.printf("LCD: found at 0x%02X\n", a);
                break;
            }
        }
        if (!lcdPresent) Serial.println(F("LCD: not found (0x27/0x3F) — no display"));
    }

    // WiFi — start AP first (always), then optionally join home network as STA
    WiFi.mode(staSSID.length() > 0 ? WIFI_AP_STA : WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS, 6);  // pin to channel 6 so AP stays stable in AP+STA mode
    Serial.print(F("AP started  SSID="));
    Serial.print(AP_SSID);
    Serial.print(F("  IP="));
    Serial.println(WiFi.softAPIP());

    if (staSSID.length() > 0) {
        WiFi.begin(staSSID.c_str(), staPass.c_str());
        Serial.print(F("WiFi STA connecting to "));
        Serial.print(staSSID);
        Serial.print(F(" "));
        unsigned long t0 = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
            delay(250); Serial.print('.');
        }
        if (WiFi.status() == WL_CONNECTED) {
            staConnected = true;
            staIP = WiFi.localIP();
            Serial.print(F(" OK  STA-IP="));
            Serial.println(staIP);
        } else {
            Serial.println(F(" FAILED (AP only)"));
        }
    }
    if (staConnected) {
        Serial.print(F("Web UI with satellite map: http://"));
        Serial.println(staIP);
    }

    // HTTP routes
    server.on("/",             HTTP_GET,  handleRoot);
    server.on("/api/status",   HTTP_GET,  handleStatus);
    server.on("/api/goto",     HTTP_POST, handleGoto);
    server.on("/api/stop",     HTTP_POST, handleStop);
    server.on("/api/cw",       HTTP_POST, handleCW);
    server.on("/api/ccw",      HTTP_POST, handleCCW);
    server.on("/api/sethome",  HTTP_POST, handleSethome);
    server.on("/api/timecw",   HTTP_POST, handleTimeCW);
    server.on("/api/timeccw",  HTTP_POST, handleTimeCCW);
    server.on("/api/command",  HTTP_POST, handleCommand);
    server.onNotFound(handleNotFound);
    server.begin();

    Serial.println(F("HTTP server started."));
    Serial.println(F("Open http://192.168.4.1 in your browser."));
    Serial.println(F("Type HELP for serial commands."));
    Serial.println(F("*** Run SETHOME to set a reference heading before using GOTO ***\n"));
}

// =============================================================================
//  Loop
// =============================================================================
void loop() {
    server.handleClient();

    unsigned long now = millis();

    // Control loop at 100 ms intervals
    if (now - lastControlMs >= 100) {
        lastControlMs = now;
        controlLoop();
    }

    // LCD refresh at LCD_UPDATE_MS intervals
    if (now - lastLcdUpdateMs >= LCD_UPDATE_MS) {
        lastLcdUpdateMs = now;
        updateLCD();
    }

    // Serial command input (newline-terminated)
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        if (cmd.length() > 0) {
            String result = processCommand(cmd);
            Serial.println(result);
        }
    }
}
