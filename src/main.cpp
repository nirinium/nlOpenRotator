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
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>OpenRotator</title>
<style>
:root{
  --p0:#000800;--p1:#001200;--p2:#002200;--p3:#003a00;--p4:#00660a;--p5:#00aa14;
  --p6:#00dd1c;--p7:#44ff55;--am:#88ff99;--rd:#ff3300;--yw:#ccee00;--dim:#003300;
  --scan:repeating-linear-gradient(0deg,transparent,transparent 2px,rgba(0,0,0,0.18) 2px,rgba(0,0,0,0.18) 4px);
}
*{box-sizing:border-box;margin:0;padding:0}
html,body{height:100%;overflow:hidden}
body{background:var(--p0);color:var(--p6);font-family:'Courier New',monospace;font-size:12px;
  display:flex;flex-direction:column;background-image:var(--scan);}
#hdr{flex:none;background:var(--p1);padding:4px 10px;display:flex;align-items:center;
  justify-content:space-between;border-bottom:2px solid var(--p3);
  box-shadow:0 0 12px rgba(0,180,20,0.15),inset 0 -1px 0 var(--p4);}
#hdr h1{color:var(--p6);font-size:12px;font-weight:normal;letter-spacing:3px;
  text-shadow:0 0 8px var(--p5),0 0 20px rgba(0,200,20,0.3);text-transform:uppercase;}
#hdr-right{display:flex;align-items:center;gap:10px}
#hdr-time{color:var(--p7);font-size:12px;letter-spacing:2px;text-shadow:0 0 8px var(--p5);}
#grid-ref{color:var(--p5);font-size:11px;letter-spacing:1px;border-left:1px solid var(--p3);padding-left:10px;}
#rb{flex:none;background:#1a0000;border-bottom:2px solid var(--rd);color:var(--rd);
  padding:3px 10px;font-size:10px;display:none;justify-content:space-between;align-items:center;
  text-shadow:0 0 6px rgba(255,60,0,0.5);animation:rblink 1s step-end infinite;}
@keyframes rblink{0%,100%{opacity:1}50%{opacity:.7}}
#rb button{background:none;border:none;color:var(--rd);cursor:pointer;font-size:13px}
#main{flex:1;display:flex;gap:4px;padding:4px;overflow:hidden;min-height:0}
.pn{background:var(--p1);border:1px solid var(--p3);position:relative;
  box-shadow:0 0 10px rgba(0,160,20,0.07),inset 0 0 30px rgba(0,0,0,0.4);
  padding:6px;overflow:hidden;display:flex;flex-direction:column;}
.pn::before,.pn::after{content:'';position:absolute;width:8px;height:8px;border-color:var(--p5);border-style:solid;pointer-events:none;}
.pn::before{top:2px;left:2px;border-width:2px 0 0 2px}
.pn::after{bottom:2px;right:2px;border-width:0 2px 2px 0}
#pL{flex:none;width:192px}#pR{flex:1;min-width:0;overflow-y:auto}
canvas{display:block;margin:0 auto;cursor:crosshair}
.rl-row{display:flex;gap:4px;margin-top:6px}
.rl{flex:1;padding:3px 2px;text-align:center;font-size:11px;line-height:1.5;
  background:var(--p0);border:1px solid var(--p3);color:var(--p5);text-transform:uppercase;letter-spacing:1px;}
.rl.on{border-color:var(--rd);color:var(--rd);background:#0d0000;
  box-shadow:0 0 6px rgba(255,50,0,0.3),inset 0 0 4px rgba(255,0,0,0.1);
  text-shadow:0 0 6px var(--rd);animation:pulse-red .6s ease-in-out infinite alternate;}
@keyframes pulse-red{from{opacity:.8}to{opacity:1}}
.rl b{display:block;font-size:12px}
.stat{display:flex;justify-content:space-between;align-items:center;padding:2px 0;border-bottom:1px solid var(--p2);font-size:12px}
.stat:last-child{border:none}
.stat span:first-child{color:var(--p6);letter-spacing:1px;text-transform:uppercase}
.sv{color:var(--p7);font-size:16px;letter-spacing:2px;
  text-shadow:0 0 8px var(--p6),0 0 20px rgba(0,220,20,0.4);
  background:var(--p0);padding:1px 5px;border:1px solid var(--p3);
  min-width:58px;text-align:right;display:inline-block;
  border-top-color:var(--p2);border-left-color:var(--p2);}
.badge{padding:1px 8px;font-size:11px;font-weight:normal;letter-spacing:2px;text-transform:uppercase;border:1px solid;}
.IDLE,.AT_TARGET{background:var(--p0);color:var(--p5);border-color:var(--p4);text-shadow:0 0 5px var(--p4);}
.ROTATING_CW,.ROTATING_CCW{background:#0d0500;color:var(--yw);border-color:#667700;
  text-shadow:0 0 5px var(--yw);animation:pulse-yw .5s ease-in-out infinite alternate;}
@keyframes pulse-yw{from{opacity:.75}to{opacity:1}}
.LIMIT_HIT{background:#0d0000;color:var(--rd);border-color:#660000;text-shadow:0 0 6px var(--rd);}
.sec{font-size:11px;color:var(--p6);letter-spacing:3px;text-transform:uppercase;
  margin:5px 0 3px;padding:2px 0;border-bottom:1px solid var(--p4);
  display:flex;align-items:center;gap:6px;text-shadow:0 0 5px rgba(0,180,20,0.4);}
.sec::before{content:'//';color:var(--p4)}
input[type=number]{width:100%;padding:3px 5px;border:1px solid var(--p3);
  border-top-color:var(--p2);border-left-color:var(--p2);
  background:var(--p0);color:var(--p7);font-size:15px;font-family:'Courier New',monospace;
  text-align:center;text-shadow:0 0 6px var(--p6);letter-spacing:2px;}
input[type=number]:focus{outline:none;border-color:var(--p5);box-shadow:0 0 6px rgba(0,180,20,0.3)}
.br{display:flex;gap:2px;margin-top:3px}
button{flex:1;padding:5px 2px;border:1px solid var(--p4);background:var(--p1);color:var(--p6);
  font-size:11px;letter-spacing:1px;text-transform:uppercase;cursor:pointer;
  font-family:'Courier New',monospace;white-space:nowrap;transition:all .1s;}
button:hover{background:var(--p2);color:var(--p7);border-color:var(--p6);
  box-shadow:0 0 6px rgba(0,200,20,0.25);text-shadow:0 0 5px var(--p5);}
button:active{background:var(--p3);color:var(--p7);box-shadow:inset 0 0 6px rgba(0,0,0,0.5);}
#bGo{border-color:var(--p5);color:var(--p7);text-shadow:0 0 5px var(--p5)}
#bGo:hover{background:var(--p3);box-shadow:0 0 8px rgba(0,220,20,0.4)}
#bSt{border-color:var(--rd);color:var(--rd);text-shadow:0 0 5px var(--rd)}
#bSt:hover{background:#180000;box-shadow:0 0 8px rgba(255,50,0,0.3)}
#bHm{border-color:#4488cc;color:#66aaee;text-shadow:0 0 5px #4488cc}
#bHm:hover{background:#001020;box-shadow:0 0 8px rgba(60,120,200,0.3)}
#bCW,#bCC{border-color:var(--yw);color:var(--yw);text-shadow:0 0 4px var(--yw)}
#bCW:hover,#bCC:hover{background:#0d0c00;box-shadow:0 0 8px rgba(180,200,0,0.3)}
#bTC,#bTCC{border-color:#667700;color:#aacc00}
#bTC:hover,#bTCC:hover{background:#080a00}
.pre-row{display:grid;grid-template-columns:repeat(4,1fr);gap:2px;margin-top:3px}
.pre{border:1px solid var(--p3);padding:2px;cursor:pointer;font-size:11px;
  background:var(--p0);color:var(--p4);
  min-height:28px;display:flex;flex-direction:column;align-items:center;justify-content:center;
  text-transform:uppercase;letter-spacing:1px;}
.ps{color:var(--p6);border-color:var(--p4);background:var(--p1);
  flex-direction:row;justify-content:space-between;padding:2px 4px;}
.ps:hover{border-color:var(--p6);box-shadow:0 0 4px rgba(0,180,20,0.2)}
.ps .pd{text-align:left;line-height:1.3}
.ps .pd b{display:block;color:var(--p7);font-size:10px;text-shadow:0 0 4px var(--p5)}
.ps .pe{font-size:11px;color:var(--p3);cursor:pointer;padding:0 2px;flex:none}
.ps .pe:hover{color:var(--yw)}
#mapWrap{flex:none;overflow:hidden;position:relative;margin-bottom:3px;display:none;
  border:1px solid var(--p3);box-shadow:0 0 8px rgba(0,160,20,0.1);}
#mapWrap img{width:100%;height:auto;display:block;filter:sepia(0.2) saturate(0.7) brightness(0.75)}
#mapCvs{position:absolute;top:0;left:0;width:100%;height:100%;pointer-events:none}
.dg{display:grid;grid-template-columns:1fr 1fr 1fr;gap:1px 6px}
.di{padding:2px 0;border-bottom:1px solid var(--p2);font-size:11px;display:flex;justify-content:space-between;letter-spacing:.5px;}
.di span:first-child{color:var(--p6);text-transform:uppercase}
.dv{color:var(--p7);text-shadow:0 0 5px var(--p5)}
.ok{color:var(--p7)!important;text-shadow:0 0 5px var(--p5)!important}
.wn{color:var(--yw)!important;text-shadow:0 0 4px #888800!important}
.er{color:var(--rd)!important;text-shadow:0 0 5px rgba(255,50,0,0.5)!important}
#log{flex:none;height:150px;background:var(--p0);border:1px solid var(--p3);
  border-top-color:var(--p2);border-left-color:var(--p2);
  padding:4px 5px;overflow-y:auto;font-size:11px;color:var(--p6);min-height:40px;
  text-shadow:0 0 4px rgba(0,180,20,0.4);
  background-image:var(--scan),linear-gradient(var(--p0),var(--p0));}
#log p{margin:0;line-height:1.5}
.cl{color:var(--am)}.er2{color:var(--rd);text-shadow:0 0 5px rgba(255,50,0,0.4)}
#log p:last-child::after{content:'\25AE';animation:cur .8s step-end infinite;color:var(--p6)}
@keyframes cur{0%,100%{opacity:1}50%{opacity:0}}
.cr{display:flex;gap:2px;margin-top:3px;flex:none}
.cr input{flex:1;padding:3px 5px;border:1px solid var(--p3);
  border-top-color:var(--p2);border-left-color:var(--p2);
  background:var(--p0);color:var(--am);font-size:11px;font-family:'Courier New',monospace;
  text-shadow:0 0 4px rgba(100,255,120,0.3);letter-spacing:1px;}
.cr input:focus{outline:none;border-color:var(--p5);box-shadow:0 0 6px rgba(0,180,20,0.3)}
.cr input::placeholder{color:var(--p4)}
.cr button{flex:none;padding:4px 10px;background:var(--p2);color:var(--p7);border:1px solid var(--p5);
  cursor:pointer;font-size:11px;letter-spacing:2px;text-transform:uppercase;
  font-family:'Courier New',monospace;text-shadow:0 0 5px var(--p5);}
.cr button:hover{background:var(--p3);box-shadow:0 0 8px rgba(0,200,20,0.3)}
.lbar{display:flex;justify-content:space-between;align-items:center;flex:none}
.lb{padding:1px 7px;font-size:11px;background:var(--p1);color:var(--p6);border:1px solid var(--p4);
  cursor:pointer;font-family:'Courier New',monospace;letter-spacing:1px;text-transform:uppercase;}
.lb:hover{color:var(--p7);border-color:var(--p6);box-shadow:0 0 4px rgba(0,180,20,0.2)}
#pmask{display:none;position:fixed;inset:0;background:rgba(0,8,0,0.82);z-index:999;
  align-items:center;justify-content:center;}
#pmask.vis{display:flex}
#pmod{background:var(--p1);border:1px solid var(--p5);
  box-shadow:0 0 24px rgba(0,200,20,0.35),inset 0 0 40px rgba(0,0,0,0.5);
  padding:14px 16px;min-width:220px;position:relative;}
#pmod .sec{margin-top:0;margin-bottom:8px;font-size:12px}
#pmod label{display:block;color:var(--p6);font-size:11px;letter-spacing:1px;text-transform:uppercase;margin-bottom:2px;margin-top:8px}
#pmod input[type=text],#pmod input[type=number]{width:100%;padding:4px 6px;
  border:1px solid var(--p3);border-top-color:var(--p2);border-left-color:var(--p2);
  background:var(--p0);color:var(--p7);font-size:14px;font-family:'Courier New',monospace;letter-spacing:2px;}
#pmod input:focus{outline:none;border-color:var(--p5);box-shadow:0 0 6px rgba(0,180,20,0.3)}
.pmbr{display:flex;gap:4px;margin-top:12px}
#pmOk{flex:1;background:var(--p2);color:var(--p7);border:1px solid var(--p5);font-size:11px;
  padding:6px;letter-spacing:2px;text-transform:uppercase;cursor:pointer;
  font-family:'Courier New',monospace;text-shadow:0 0 5px var(--p5);}
#pmOk:hover{background:var(--p3);box-shadow:0 0 8px rgba(0,200,20,0.4)}
#pmCn{flex:none;background:transparent;color:var(--rd);border:1px solid var(--rd);
  font-size:11px;padding:6px 10px;letter-spacing:2px;text-transform:uppercase;
  cursor:pointer;font-family:'Courier New',monospace;}
#pmCn:hover{background:#180000}
::-webkit-scrollbar{width:6px;background:var(--p0)}
::-webkit-scrollbar-track{background:var(--p0);border-left:1px solid var(--p2)}
::-webkit-scrollbar-thumb{background:var(--p3)}
::-webkit-scrollbar-thumb:hover{background:var(--p4)}
</style>
</head>
<body>
<div id="hdr">
  <h1>&#x2316;&nbsp; ANT-ROTATOR CTL &mdash; NL-01</h1>
  <div id="hdr-right">
    <span id="grid-ref">QTH: EM77EB &nbsp;|&nbsp; 38.032&deg;N 85.347&deg;W</span>
    <span id="hdr-time">--:--:--Z</span>
    <span id="sts2"><span class="badge IDLE">IDLE</span></span>
  </div>
</div>
<div id="rb">
  &#9888;&nbsp; [WARN] HEADING RESTORED FROM NVS &mdash; VERIFY ANTENNA POSITION BEFORE GOTO
  <button onclick="dismissRb()">&#x2715;</button>
</div>
<div id="main">
  <div class="pn" id="pL">
    <canvas id="cmp" width="166" height="166" title="Click to set target"></canvas>
    <div class="rl-row">
      <div class="rl off" id="rA">CW<b id="rAs">OFF</b></div>
      <div class="rl off" id="rB">CCW<b id="rBs">OFF</b></div>
    </div>
    <div style="margin-top:6px">
      <div class="stat"><span>Heading</span><span class="sv" id="hdg">---&deg;</span></div>
      <div class="stat"><span>Target</span><span class="sv" id="tgt">---&deg;</span></div>
    </div>
  </div>
  <div class="pn" id="pR">
    <div class="sec">Navigate</div>
    <input type="number" id="ti" value="180" min="0" max="360">
    <div class="br">
      <button id="bGo" onclick="sendGotoVal(-1)">GOTO</button>
      <button id="bSt" onclick="doStop()">STOP</button>
      <button id="bHm" onclick="doSethome()">HOME</button>
    </div>
    <div class="br" style="margin-top:4px">
      <button id="bCW"  onclick="doPost('/api/cw','CW')">&#9664;&nbsp;CW</button>
      <button id="bCC"  onclick="doPost('/api/ccw','CCW')">CCW&nbsp;&#9654;</button>
      <button id="bTC"  onclick="doPost('/api/timecw','TIMECW')">&#9201;&nbsp;T-CW</button>
      <button id="bTCC" onclick="doPost('/api/timeccw','TIMECCW')">&#9201;&nbsp;T-CCW</button>
    </div>
    <div class="sec">Presets <span style="font-size:9px;letter-spacing:0;text-transform:none;color:var(--p3)">&nbsp;click=goto &nbsp;&#x270E;=edit</span></div>
    <div class="pre-row" id="prs"></div>
    <div class="sec" style="display:flex;align-items:center;justify-content:space-between;flex:none">
      <span>Area Map</span>
      <button class="lb" id="btnMapTog" onclick="toggleMap()">Show</button>
    </div>
    <div id="mapWrap">
      <img id="mapImg" src="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/export?bbox=-87.5%2C36.5%2C-83.2%2C40.0&bboxSR=4326&size=900%2C560&format=png&f=image"
           alt="Taylorsville KY area" onload="drawMapOverlay()">
      <canvas id="mapCvs"></canvas>
    </div>
    <div class="sec">Diagnostics</div>
    <div class="dg">
      <div class="di"><span>Runtime</span><span class="dv" id="dRt">0ms</span></div>
      <div class="di"><span>Uptime</span><span class="dv" id="dUp">---</span></div>
      <div class="di"><span>MS/Deg</span><span class="dv" id="dMd">---</span></div>
      <div class="di"><span>Deadband</span><span class="dv" id="dDb">---</span></div>
      <div class="di"><span>Known</span><span class="dv er" id="dKn">NO</span></div>
      <div class="di"><span>Timing</span><span class="dv" id="dTm">OFF</span></div>
      <div class="di"><span>Min</span><span class="dv" id="dMn">0&deg;</span></div>
      <div class="di"><span>Max</span><span class="dv" id="dMx">360&deg;</span></div>
    </div>
    <div class="lbar" style="margin-top:5px">
      <span class="sec" style="margin:0">// Console</span>
      <button class="lb" onclick="cl()">CLR</button>
    </div>
    <div id="log"></div>
    <div class="cr">
      <input id="ci" placeholder="GOTO 180 | SETWIFI myssid mypass | FACTORY | HELP" onkeydown="ok(event)">
      <button onclick="dc()">TX</button>
    </div>
  </div>
</div>
<!-- Preset editor modal -->
<div id="pmask">
  <div id="pmod">
    <div class="sec">// Edit Preset</div>
    <label>Label (6 chars)</label>
    <input type="text" id="pmLbl" maxlength="6" autocomplete="off">
    <label>Heading (0&ndash;360&deg;)</label>
    <input type="number" id="pmDeg" min="0" max="360">
    <div class="pmbr">
      <button id="pmOk" onclick="pmSave()">SAVE</button>
      <button id="pmCn" onclick="pmClose()">CANCEL</button>
    </div>
  </div>
</div>
<script>
const W=166,CX=83,CY=83,R=76;
let heading=0,target=-1,hdgMin=0,hdgMax=360;
let displayHeading=0,animId=null;
let cmdHist=[],cmdIdx=-1,restoredDismissed=false;
let mapOpen=false;
// QTH: 847 Townhill Rd, Taylorsville KY 40071
const MAP_LAT=38.0344609,MAP_LON=-85.3316739;
const MAP_W=-87.5,MAP_E=-83.2,MAP_S=36.5,MAP_N=40.0;
const MAP_CITIES=[
  ['Louisville',38.2527,-85.7585],['Lexington',38.0406,-84.5037],
  ['Frankfort',38.2009,-84.8733],['Cincinnati',39.1031,-84.5120],
  ["E'town",37.6937,-85.8591],['Bardstown',37.8137,-85.4669],
  ['Shelbyville',38.2115,-85.2238],['Lawrenceburg',38.037,-84.899],
  ['Bowling Green',36.9903,-86.4436],['Indianapolis',39.7684,-86.1581],
];

function drawCompass(h,t){
  const c=document.getElementById('cmp').getContext('2d');
  c.clearRect(0,0,W,W);
  c.beginPath();c.arc(CX,CY,R,0,Math.PI*2);
  c.fillStyle='#000d00';c.fill();c.strokeStyle='#004000';c.lineWidth=2;c.stroke();
  if(hdgMax-hdgMin<355){
    const sa=((hdgMax-90+720)%360)*Math.PI/180,ea=((hdgMin-90+720)%360)*Math.PI/180;
    c.beginPath();c.moveTo(CX,CY);c.arc(CX,CY,R-1,sa,ea);c.closePath();
    c.fillStyle='rgba(0,10,0,.85)';c.fill();
    [hdgMin,hdgMax].forEach(d=>{
      const a=(d-90)*Math.PI/180;
      c.beginPath();c.moveTo(CX+62*Math.cos(a),CY+62*Math.sin(a));
      c.lineTo(CX+75*Math.cos(a),CY+75*Math.sin(a));
      c.strokeStyle='#ccee00';c.lineWidth=3;c.stroke();
    });
  }
  if(t>=0&&Math.abs(h-t)>0.5){
    const sA=(h-90)*Math.PI/180,eA=(t-90)*Math.PI/180;
    c.beginPath();c.arc(CX,CY,70,sA,eA,t<h);
    c.strokeStyle=t<h?'rgba(0,200,20,.45)':'rgba(0,160,20,.45)';
    c.lineWidth=4;c.stroke();
  }
  for(let i=0;i<72;i++){
    const a=(i*5-90)*Math.PI/180,big=i%18===0,med=i%6===0;
    c.beginPath();
    c.moveTo(CX+(big?57:med?61:64)*Math.cos(a),CY+(big?57:med?61:64)*Math.sin(a));
    c.lineTo(CX+74*Math.cos(a),CY+74*Math.sin(a));
    c.strokeStyle=big?'#00dd1c':med?'#005500':'#003300';c.lineWidth=big?2:1;c.stroke();
  }
  [['N',0,'#ff3300'],['E',90,'#005500'],['S',180,'#005500'],['W',270,'#005500']].forEach(([l,d,col])=>{
    const a=(d-90)*Math.PI/180;c.fillStyle=col;c.font='bold 10px monospace';
    c.textAlign='center';c.textBaseline='middle';
    if(l==='N'){c.shadowColor='rgba(255,50,0,0.6)';c.shadowBlur=6;}
    c.fillText(l,CX+48*Math.cos(a),CY+48*Math.sin(a));
    c.shadowBlur=0;
  });
  if(t>=0){
    const ta=(t-90)*Math.PI/180;
    c.beginPath();c.moveTo(CX,CY);
    c.lineTo(CX+R*.78*Math.cos(ta),CY+R*.78*Math.sin(ta));
    c.strokeStyle='rgba(180,220,0,.5)';c.lineWidth=2;c.setLineDash([3,3]);c.stroke();c.setLineDash([]);
  }
  const na=(h-90)*Math.PI/180;
  c.beginPath();c.moveTo(CX-15*Math.cos(na),CY-15*Math.sin(na));c.lineTo(CX,CY);
  c.strokeStyle='#003a00';c.lineWidth=3;c.stroke();
  c.beginPath();c.moveTo(CX,CY);c.lineTo(CX+58*Math.cos(na),CY+58*Math.sin(na));
  c.strokeStyle='#00ff1e';c.lineWidth=3;
  c.shadowColor='rgba(0,255,30,0.6)';c.shadowBlur=8;c.stroke();c.shadowBlur=0;
  c.beginPath();c.arc(CX,CY,5,0,Math.PI*2);c.fillStyle='#00dd1c';c.fill();
  c.fillStyle='#44ff55';c.font='bold 12px monospace';
  c.textAlign='center';c.textBaseline='middle';
  c.shadowColor='rgba(0,220,20,0.5)';c.shadowBlur=6;
  c.fillText(Math.round(h)+'\u00b0',CX,CY+20);
  c.shadowBlur=0;
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
  document.getElementById('ti').value=deg;sendGotoVal(deg);
});

// Presets
const NP=4;
function lps(){try{return JSON.parse(localStorage.getItem('rtp')||'[]');}catch{return [];}}
function sps(a){localStorage.setItem('rtp',JSON.stringify(a));}
function rndPre(){
  const ps=lps(),el=document.getElementById('prs');el.innerHTML='';
  for(let i=0;i<NP;i++){
    const p=ps[i],d=document.createElement('div');
    if(p){
      d.className='pre ps';
      d.innerHTML='<span class="pd" onclick="gtp('+i+')">'+p.l+'<b>'+p.d+'\u00b0</b></span>'
                 +'<span class="pe" onclick="stp('+i+')" title="Edit">&#x270E;</span>';
    }else{
      d.className='pre';
      d.innerHTML='<span style="color:var(--p4);text-align:center" onclick="stp('+i+')">'
                 +'P'+(i+1)+'<b style="display:block;color:#1e2d40;font-size:.7rem">+</b></span>';
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
document.addEventListener('keydown',e=>{
  if(e.key==='Escape')pmClose();
  if(e.key==='Enter'&&document.getElementById('pmask').classList.contains('vis')){e.preventDefault();pmSave();}
});
function pmSave(){
  const i=document.getElementById('pmask')._slot;
  const ps=lps();
  const nl=document.getElementById('pmLbl').value.trim().slice(0,6)||('P'+(i+1));
  const nd=parseInt(document.getElementById('pmDeg').value)||0;
  ps[i]={l:nl,d:nd};
  sps(ps);rndPre();pmClose();
  lg('PRESET '+(i+1)+' SAVED \u2192 '+nl+' / '+nd+'\u00b0','cl');
  const slots=document.querySelectorAll('#prs .pre');
  if(slots[i]){
    slots[i].style.boxShadow='0 0 8px 2px rgba(0,220,20,0.7)';
    slots[i].style.borderColor='var(--p7)';
    setTimeout(()=>{slots[i].style.boxShadow='';slots[i].style.borderColor='';},1200);
  }
}

// Map
function ll2px(lat,lon,W,H){
  return[(lon-MAP_W)/(MAP_E-MAP_W)*W,(MAP_N-lat)/(MAP_N-MAP_S)*H];}
function distMi(a,b,c,d){
  const R=3958.8,dL=(c-a)*Math.PI/180,dG=(d-b)*Math.PI/180;
  const x=Math.sin(dL/2)**2+Math.cos(a*Math.PI/180)*Math.cos(c*Math.PI/180)*Math.sin(dG/2)**2;
  return R*2*Math.atan2(Math.sqrt(x),Math.sqrt(1-x));}
function bearingDeg(la1,lo1,la2,lo2){
  const r=Math.PI/180,dL=(lo2-lo1)*r;
  const x=Math.sin(dL)*Math.cos(la2*r);
  const y=Math.cos(la1*r)*Math.sin(la2*r)-Math.sin(la1*r)*Math.cos(la2*r)*Math.cos(dL);
  return(Math.atan2(x,y)*180/Math.PI+360)%360;}
function drawMapOverlay(){
  if(!mapOpen)return;
  const img=document.getElementById('mapImg'),cvs=document.getElementById('mapCvs');
  if(!img||!cvs||!img.complete||!img.naturalWidth)return;
  const W=img.offsetWidth,H=img.offsetHeight;if(!W||!H)return;
  cvs.width=W;cvs.height=H;
  const c=cvs.getContext('2d');
  c.clearRect(0,0,W,H);
  const[hx,hy]=ll2px(MAP_LAT,MAP_LON,W,H);
  const pxMi=H/(MAP_N-MAP_S)/69.0;
  [50,100,150,200].forEach(mi=>{
    const r=mi*pxMi;
    c.beginPath();c.arc(hx,hy,r,0,Math.PI*2);
    c.strokeStyle='rgba(0,200,20,0.35)';c.lineWidth=1;c.setLineDash([5,4]);c.stroke();c.setLineDash([]);
    const rx=hx+r*0.707,ry=hy-r*0.707;
    c.lineWidth=2;c.strokeStyle='rgba(0,0,0,0.7)';
    c.font='bold 9px monospace';c.textAlign='left';c.textBaseline='middle';
    c.strokeText(mi+'mi',rx+1,ry+1);
    c.fillStyle='rgba(0,220,20,0.9)';c.shadowColor='rgba(0,200,20,0.5)';c.shadowBlur=3;
    c.fillText(mi+'mi',rx,ry);c.shadowBlur=0;
  });
  MAP_CITIES.forEach(([name,lat,lon])=>{
    const[cx,cy]=ll2px(lat,lon,W,H);
    const d=distMi(MAP_LAT,MAP_LON,lat,lon).toFixed(0)+'mi';
    const brg=bearingDeg(MAP_LAT,MAP_LON,lat,lon).toFixed(0)+'\u00b0';
    c.beginPath();c.arc(cx,cy,4,0,Math.PI*2);
    c.fillStyle='#00dd1c';c.fill();c.strokeStyle='#000d00';c.lineWidth=1.5;c.stroke();
    c.font='bold 10px monospace';c.textAlign='left';c.textBaseline='bottom';
    c.lineWidth=3;c.strokeStyle='rgba(0,0,0,0.85)';
    c.strokeText(name,cx+6,cy);c.fillStyle='#44ff55';c.fillText(name,cx+6,cy);
    c.font='9px monospace';c.lineWidth=2;
    c.strokeText(brg+' / '+d,cx+6,cy+11);c.fillStyle='#ccee00';c.fillText(brg+' / '+d,cx+6,cy+11);
  });
  const rad=displayHeading*Math.PI/180,blen=Math.hypot(W,H);
  c.beginPath();c.moveTo(hx,hy);c.lineTo(hx+blen*Math.sin(rad),hy-blen*Math.cos(rad));
  c.strokeStyle='rgba(0,255,30,0.85)';c.lineWidth=2;c.setLineDash([]);c.stroke();
  const lx=hx+45*Math.sin(rad),ly=hy-45*Math.cos(rad);
  c.font='bold 12px monospace';c.textAlign='center';c.textBaseline='middle';
  c.lineWidth=3;c.strokeStyle='rgba(0,0,0,0.9)';
  const ht=Math.round(displayHeading)+'\u00b0';
  c.strokeText(ht,lx,ly);
  c.fillStyle='#00ff1e';c.shadowColor='rgba(0,255,30,0.5)';c.shadowBlur=5;
  c.fillText(ht,lx,ly);c.shadowBlur=0;
  c.beginPath();c.arc(hx,hy,7,0,Math.PI*2);
  c.fillStyle='#00dd1c';c.fill();c.strokeStyle='#44ff55';c.lineWidth=2;c.stroke();
  c.beginPath();c.moveTo(hx-13,hy);c.lineTo(hx+13,hy);c.moveTo(hx,hy-13);c.lineTo(hx,hy+13);
  c.strokeStyle='rgba(0,200,20,0.6)';c.lineWidth=1;c.stroke();
  c.font='bold 10px monospace';c.textAlign='center';c.textBaseline='bottom';
  c.lineWidth=2;c.strokeStyle='rgba(0,0,0,0.7)';
  c.strokeText('N',hx,hy-16);c.fillStyle='#ff3300';c.fillText('N',hx,hy-16);
}
function toggleMap(){
  mapOpen=!mapOpen;
  document.getElementById('mapWrap').style.display=mapOpen?'block':'none';
  document.getElementById('btnMapTog').textContent=mapOpen?'Hide':'Show';
  if(mapOpen)setTimeout(drawMapOverlay,60);
}
function dismissRb(){restoredDismissed=true;document.getElementById('rb').style.display='none';}
function lg(msg,cls){
  const el=document.getElementById('log'),p=document.createElement('p');
  if(cls)p.className=cls;
  const t=new Date();
  const ts=String(t.getUTCHours()).padStart(2,'0')+String(t.getUTCMinutes()).padStart(2,'0')+'Z';
  p.textContent=ts+' > '+msg;
  el.appendChild(p);el.scrollTop=el.scrollHeight;
  if(el.children.length>80)el.removeChild(el.firstChild);
}
function cl(){document.getElementById('log').innerHTML='';}
function fMs(ms){if(!ms||ms<=0)return '0ms';if(ms<1000)return ms+'ms';if(ms<60000)return(ms/1000).toFixed(1)+'s';return Math.floor(ms/60000)+'m'+(Math.floor(ms/1000)%60)+'s';}
function fUp(ms){const s=Math.floor(ms/1000),h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sc=s%60;return(h?h+'h ':'')+( m?m+'m ':'')+sc+'s';}
function updDbg(d){
  const cOn=d.pinCW===0,ccOn=d.pinCCW===0;
  document.getElementById('rA').className='rl '+(cOn?'on':'off');
  document.getElementById('rAs').textContent=cOn?'ON':'OFF';
  document.getElementById('rB').className='rl '+(ccOn?'on':'off');
  document.getElementById('rBs').textContent=ccOn?'ON':'OFF';
  document.getElementById('dRt').textContent=fMs(d.motorRunMs||0);
  document.getElementById('dUp').textContent=fUp(d.uptime||0);
  document.getElementById('dMd').textContent=(d.msperdeg||0).toFixed(1);
  document.getElementById('dDb').textContent=(d.deadband||0).toFixed(1)+'\u00b0';
  const kEl=document.getElementById('dKn');kEl.textContent=d.headingKnown?'YES':'NO';kEl.className='dv '+(d.headingKnown?'ok':'er');
  const tEl=document.getElementById('dTm');tEl.textContent=d.timingMode?'ON':'OFF';tEl.className='dv '+(d.timingMode?'wn':'');
  if(d.hdgMin!==undefined){
    hdgMin=parseFloat(d.hdgMin);hdgMax=parseFloat(d.hdgMax);
    document.getElementById('dMn').textContent=hdgMin.toFixed(0)+'\u00b0';
    document.getElementById('dMx').textContent=hdgMax.toFixed(0)+'\u00b0';
    document.getElementById('ti').min=hdgMin;document.getElementById('ti').max=hdgMax;
  }
  const rb=document.getElementById('rb');
  if(d.hdgRestored&&!restoredDismissed)rb.style.display='flex';
  else if(!d.hdgRestored){restoredDismissed=false;rb.style.display='none';}
}
function ok(e){
  if(e.key==='Enter'){dc();return;}
  if(e.key==='ArrowUp'){if(!cmdHist.length)return;cmdIdx=Math.min(cmdIdx+1,cmdHist.length-1);document.getElementById('ci').value=cmdHist[cmdIdx];e.preventDefault();}
  if(e.key==='ArrowDown'){cmdIdx=Math.max(cmdIdx-1,-1);document.getElementById('ci').value=cmdIdx<0?'':cmdHist[cmdIdx];e.preventDefault();}
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
    target=parseFloat(d.target);startAnim();
    document.getElementById('hdg').textContent=d.headingKnown?hv.toFixed(1)+'\u00b0':'---\u00b0';
    document.getElementById('tgt').textContent=target>=0?target.toFixed(1)+'\u00b0':'---\u00b0';
    const st=d.status||'IDLE';
    document.getElementById('sts2').innerHTML='<span class="badge '+st+'">'+st+'</span>';
    updDbg(d);
  }catch(e){}
}
async function sendGotoVal(deg,label){
  if(deg<0){deg=parseFloat(document.getElementById('ti').value);if(isNaN(deg))return;}
  deg=Math.round(Math.min(hdgMax,Math.max(hdgMin,deg)));
  const body={heading:deg};if(label)body.label=label;
  try{const d=await api('/api/goto','POST',body);lg('GOTO '+deg+'\u00b0 \u2192 '+d.result,'cl');}
  catch(e){lg('GOTO error','er2');}
}
async function doStop(){
  try{
    const d=await api('/api/stop','POST');
    let msg='STOP \u2192 '+d.result;
    if(d.timedMs)msg+=' | '+d.timedMs+'ms for '+d.sweepDeg+'\u00b0 \u2192 MSPERDEG '+d.suggestedMsDeg;
    lg(msg,'cl');
  }catch(e){}
}
async function doPost(url,label){try{const d=await api(url,'POST');lg(label+' \u2192 '+d.result,'cl');}catch(e){}}
async function doSethome(){
  const deg=parseInt(document.getElementById('ti').value,10);
  const val=isNaN(deg)?0:Math.min(hdgMax,Math.max(hdgMin,deg));
  try{const d=await api('/api/sethome','POST',{heading:val});lg('SETHOME '+val+'\u00b0 \u2192 '+d.result,'cl');}catch(e){}
}
async function dc(){
  const ci=document.getElementById('ci'),cmd=ci.value.trim();if(!cmd)return;
  ci.value='';cmdHist.unshift(cmd);if(cmdHist.length>50)cmdHist.pop();cmdIdx=-1;
  try{const d=await api('/api/command','POST',{cmd:cmd});lg('> '+cmd,'cl');lg(d.result);}
  catch(e){lg('ERR: '+e,'er2');}
}
rndPre();drawCompass(0,-1);pollStatus();setInterval(pollStatus,500);
setInterval(()=>{
  const t=new Date();
  document.getElementById('hdr-time').textContent=
    String(t.getUTCHours()).padStart(2,'0')+':'+
    String(t.getUTCMinutes()).padStart(2,'0')+':'+
    String(t.getUTCSeconds()).padStart(2,'0')+'Z';
},1000);
</script>
</body>
</html>)rawliteral";

// =============================================================================
//  Web server handlers
// =============================================================================
void handleRoot() {
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
    server.send(200, "application/json", json);
}

void handleGoto() {
    if (!server.hasArg("plain")) { server.send(400, "application/json", "{\"error\":\"no body\"}"); return; }
    if (!headingKnown) {
        server.send(200, "application/json", "{\"result\":\"ERR HEADING_UNKNOWN — run TIMECCW to end stop then STOP to home first\"}");
        return;
    }
    StaticJsonDocument<128> doc;
    if (deserializeJson(doc, server.arg("plain"))) { server.send(400, "application/json", "{\"error\":\"bad json\"}"); return; }
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
    server.send(200, "application/json",
        "{\"result\":\"OK\",\"target\":" + String(h, 1) + "}");
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
    server.send(200, "application/json", resp);
}

void handleCommand() {
    if (!server.hasArg("plain")) { server.send(400); return; }
    StaticJsonDocument<96> doc;
    if (deserializeJson(doc, server.arg("plain"))) { server.send(400); return; }
    String cmd    = doc["cmd"].as<String>();
    String result = processCommand(cmd);
    StaticJsonDocument<384> resp;
    resp["result"] = result;
    String json;
    serializeJson(resp, json);
    server.send(200, "application/json", json);
}

void handleNotFound() {
    server.send(404, "text/plain", "Not found");
}

void handleCW() {
    if (millis() - lastRelayChangeMs < RELAY_COOLDOWN_MS) {
        server.send(429, "application/json", "{\"result\":\"COOLDOWN\"}");
        return;
    }
    targetHeading      = -1.0f;
    freeRunMode        = true;
    currentTargetLabel = "";
    motorCW();
    server.send(200, "application/json", "{\"result\":\"ROTATING_CW\"}");
}

void handleCCW() {
    if (millis() - lastRelayChangeMs < RELAY_COOLDOWN_MS) {
        server.send(429, "application/json", "{\"result\":\"COOLDOWN\"}");
        return;
    }
    targetHeading      = -1.0f;
    freeRunMode        = true;
    currentTargetLabel = "";
    motorCCW();
    server.send(200, "application/json", "{\"result\":\"ROTATING_CCW\"}");
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
    server.send(200, "application/json",
        "{\"result\":\"HOME_SET=" + String(deg, 1) + "\"}");
}

void handleTimeCW() {
    targetHeading = -1.0f;
    freeRunMode   = true;
    timingMode    = true;
    timingStartMs = millis();
    motorCW();
    server.send(200, "application/json", "{\"result\":\"TIMING_CW\"}" );
}

void handleTimeCCW() {
    targetHeading = -1.0f;
    freeRunMode   = true;
    timingMode    = true;
    timingStartMs = millis();
    motorCCW();
    server.send(200, "application/json", "{\"result\":\"TIMING_CCW\"}");
}

// =============================================================================
//  Setup
// =============================================================================
void setup() {
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
    MS_PER_DEG = prefs.getFloat("msperdeg",  163.0f);
    HDG_MIN    = prefs.getFloat("hdgmin",    0.0f);
    HDG_MAX    = prefs.getFloat("hdgmax",    360.0f);
    DEADBAND   = prefs.getFloat("deadband",  4.0f);
    HDG_OFFSET = prefs.getFloat("hdgoffset", 0.0f);
    if (prefs.getBool("hdgknown", false)) {
        currentHeading = prefs.getFloat("lastheading", 0.0f);
        motorStartHdg  = currentHeading;
        headingKnown   = true;
        hdgRestored    = true;
        Serial.printf("NVS: heading restored to %.1f deg\n", currentHeading);
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
