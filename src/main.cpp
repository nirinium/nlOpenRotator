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

// ─── Pin definitions ──────────────────────────────────────────────────────────
//  Relay module wiring (2× 1-channel, 5 V, active-LOW, optocoupler-isolated):
//    PIN_MOTOR_CW  → relay module IN1  (LOW = relay on = 24 V AC to CW winding)
//    PIN_MOTOR_CCW → relay module IN2  (LOW = relay on = 24 V AC to CCW winding)
//  Relay module VCC → ESP32 5 V (VIN).  Never energise both relays simultaneously.
#define PIN_MOTOR_CW   25   // digital output → relay IN1 (active LOW)
#define PIN_MOTOR_CCW  26   // digital output → relay IN2 (active LOW)
//  Optional status LED (built-in on most ESP32 devkits)
#define PIN_LED         2

// ─── WiFi access-point credentials ───────────────────────────────────────────
const char* AP_SSID = "OpenRotator";
const char* AP_PASS = "rotator123";   // change to something stronger!

// ─── Calibration ─────────────────────────────────────────────────────────────
//  Time how long (in ms) the rotator takes to turn a known number of degrees,
//  then set MS_PER_DEG = elapsed_ms / degrees_turned.
//  Example: rotator does a full 360° in 45 seconds → MS_PER_DEG = 45000/360 = 125
//  Use the TIMECW / TIMECCW serial commands to measure this at runtime.
float MS_PER_DEG   = 125.0f;  // milliseconds per degree of rotation (adjust to suit)

// ─── Tuning knobs ─────────────────────────────────────────────────────────────
float DEADBAND     = 4.0f;    // Stop within ±N degrees of target (increase if overshooting)

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

// =============================================================================
//  Position tracking — timed dead-reckoning
//  Call updateHeading() at any time to refresh currentHeading from elapsed run time.
// =============================================================================
void updateHeading() {
    if (motorDirection == 0) return;
    unsigned long elapsed = millis() - motorStartMs;
    float delta = (float)elapsed / MS_PER_DEG;
    currentHeading = fmodf(motorStartHdg + (motorDirection * delta) + 360.0f, 360.0f);
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
}

void motorCW() {
    motorStop();                        // de-energise + capture heading first
    motorStartHdg = currentHeading;
    motorStartMs  = millis();
    motorDirection = 1;
    digitalWrite(PIN_MOTOR_CW,  LOW);   // LOW = relay on → 24 V AC to CW winding
    rotatorStatus = "ROTATING_CW";
    digitalWrite(PIN_LED, HIGH);
}

void motorCCW() {
    motorStop();                        // de-energise + capture heading first
    motorStartHdg = currentHeading;
    motorStartMs  = millis();
    motorDirection = -1;
    digitalWrite(PIN_MOTOR_CCW, LOW);   // LOW = relay on → 24 V AC to CCW winding
    rotatorStatus = "ROTATING_CCW";
    digitalWrite(PIN_LED, HIGH);
}

// =============================================================================
//  Control loop  (called every 100 ms)
// =============================================================================
void controlLoop() {
    updateHeading();  // refresh currentHeading from elapsed motor run time

    if (!headingKnown || targetHeading < 0.0f) {
        // No known reference or no active target — ensure motor is stopped
        if (motorDirection != 0) motorStop();
        return;
    }

    // Shortest-path angular error, normalised to -180 … +180
    float error = targetHeading - currentHeading;
    while (error >  180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    if (fabsf(error) <= DEADBAND) {
        motorStop();
        rotatorStatus = "AT_TARGET";
        targetHeading = -1.0f;          // arrived — clear target
    } else if (error > 0.0f) {
        if (motorDirection != 1) motorCW();
    } else {
        if (motorDirection != -1) motorCCW();
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
    cmd.toUpperCase();

    if (cmd.startsWith("GOTO ")) {
        if (!headingKnown) return "ERR HEADING_UNKNOWN — run SETHOME first";
        float deg = cmd.substring(5).toFloat();
        deg = fmodf(deg + 360.0f, 360.0f);
        targetHeading = deg;
        return "OK GOING_TO=" + String(deg, 1);
    }
    if (cmd == "STOP") {
        targetHeading = -1.0f;
        motorStop();
        return "OK STOPPED";
    }
    if (cmd == "STATUS") {
        String tgt = (targetHeading >= 0.0f) ? String(targetHeading, 1) : "NONE";
        return "HDG=" + (headingKnown ? String(currentHeading, 1) : String("?")) +
               " TGT=" + tgt +
               " STS=" + rotatorStatus +
               " MS/DEG=" + String(MS_PER_DEG, 1) +
               " DB=" + String(DEADBAND, 1);
    }
    if (cmd.startsWith("SETHOME")) {
        float deg = 0.0f;
        if (cmd.length() > 8) deg = fmodf(cmd.substring(8).toFloat() + 360.0f, 360.0f);
        motorStop();
        currentHeading = deg;
        motorStartHdg  = deg;
        headingKnown   = true;
        return "OK HOME_SET=" + String(deg, 1);
    }
    if (cmd.startsWith("MSPERDEG ")) {
        float v = cmd.substring(9).toFloat();
        if (v <= 0.0f) return "ERR VALUE_MUST_BE_POSITIVE";
        MS_PER_DEG = v;
        return "OK MS_PER_DEG=" + String(MS_PER_DEG, 1);
    }
    if (cmd.startsWith("DEADBAND ")) {
        DEADBAND = max(0.5f, cmd.substring(9).toFloat());
        return "OK DEADBAND=" + String(DEADBAND, 1);
    }
    if (cmd == "HELP") {
        return "Commands: GOTO <deg> | STOP | STATUS | SETHOME [deg] | "
               "MSPERDEG <val> | DEADBAND <deg> | HELP";
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
:root{--accent:#4fc3f7;--bg:#0f0f1a;--card:#16213e;--text:#e0e0e0;--red:#ef5350}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);font-family:monospace;display:flex;
  flex-direction:column;align-items:center;min-height:100vh;padding:16px 12px}
h1{color:var(--accent);margin-bottom:16px;font-size:1.5rem;letter-spacing:3px;text-align:center}
.card{background:var(--card);border-radius:12px;padding:18px;margin:8px 0;
  width:100%;max-width:420px;box-shadow:0 4px 24px rgba(0,0,0,.5)}
.compass-wrap{display:flex;justify-content:center;margin:4px 0 12px}
canvas{cursor:crosshair}
.stat{display:flex;justify-content:space-between;padding:5px 0;border-bottom:1px solid #1e2d50}
.stat:last-child{border:none}
.stat span:last-child{color:var(--accent);font-weight:bold}
.badge{display:inline-block;padding:3px 10px;border-radius:20px;font-size:.75rem;font-weight:bold}
.IDLE,.AT_TARGET{background:#1b5e20;color:#a5d6a7}
.ROTATING_CW,.ROTATING_CCW{background:#bf360c;color:#ffcc80}
label{font-size:.8rem;color:#8899aa;margin-bottom:4px;display:block}
input[type=number]{width:100%;padding:10px;border-radius:8px;border:1px solid #2a3a5e;
  background:#0a1628;color:#fff;font-size:1.1rem;text-align:center;margin:6px 0 10px}
.btn-row{display:flex;gap:10px;margin-top:4px}
button{flex:1;padding:12px;border:none;border-radius:8px;font-size:.95rem;
  cursor:pointer;font-family:monospace;font-weight:bold;transition:opacity .15s}
button:hover{opacity:.8}
#btnGoto{background:var(--accent);color:#000}
#btnStop{background:var(--red);color:#fff}
.log{background:#070710;border-radius:8px;padding:10px;height:130px;overflow-y:auto;
  font-size:.75rem;color:#7a8fa8;margin-top:8px}
.log p{margin:1px 0}
.cmd-row{display:flex;gap:8px;margin-top:8px}
.cmd-row input{flex:1;padding:9px 12px;border-radius:6px;border:1px solid #2a3a5e;
  background:#0a1628;color:#fff;font-size:.9rem}
.cmd-row button{flex:none;width:auto;padding:9px 18px;background:#4fc3f7;color:#000}
</style>
</head>
<body>
<h1>&#x2316; OPEN ROTATOR</h1>

<div class="card">
  <div class="compass-wrap">
    <canvas id="cmp" width="220" height="220" title="Click to set target heading"></canvas>
  </div>
  <div class="stat"><span>Heading</span><span id="hdg">---&deg;</span></div>
  <div class="stat"><span>Target</span><span id="tgt">---&deg;</span></div>
  <div class="stat"><span>Status</span><span id="sts"><span class="badge IDLE">IDLE</span></span></div>
</div>

<div class="card">
  <label>Go to heading &mdash; type or click compass (0&ndash;359&deg;):</label>
  <input type="number" id="ti" min="0" max="359" value="0">
  <div class="btn-row">
    <button id="btnGoto" onclick="sendGoto()">GOTO</button>
    <button id="btnStop" onclick="sendStop()">STOP</button>
  </div>
</div>

<div class="card">
  <label>Serial console</label>
  <div class="log" id="log"></div>
  <div class="cmd-row">
    <input id="ci" placeholder="e.g. GOTO 180 | STATUS | HELP"
           onkeydown="if(event.key==='Enter')sendCmd()">
    <button onclick="sendCmd()">SEND</button>
  </div>
</div>

<script>
const cvs=document.getElementById('cmp'),ctx=cvs.getContext('2d');
const W=220,CX=110,CY=110,R=100;
let heading=0,target=-1;

function drawCompass(h,t){
  ctx.clearRect(0,0,W,W);
  // Outer ring
  ctx.beginPath();ctx.arc(CX,CY,R,0,Math.PI*2);
  ctx.fillStyle='#070710';ctx.fill();
  ctx.strokeStyle='#223355';ctx.lineWidth=2;ctx.stroke();
  // Tick marks
  for(let i=0;i<72;i++){
    const a=(i*5-90)*Math.PI/180;
    const big=i%18===0,med=i%6===0;
    const inner=big?78:med?82:87;
    ctx.beginPath();
    ctx.moveTo(CX+inner*Math.cos(a),CY+inner*Math.sin(a));
    ctx.lineTo(CX+97*Math.cos(a),CY+97*Math.sin(a));
    ctx.strokeStyle=big?'#4fc3f7':med?'#334466':'#1e2d50';
    ctx.lineWidth=big?2:1;ctx.stroke();
  }
  // Cardinal labels
  [['N',0,'#ef5350'],['E',90,'#aaa'],['S',180,'#aaa'],['W',270,'#aaa']].forEach(([l,d,c])=>{
    const a=(d-90)*Math.PI/180;
    ctx.fillStyle=c;ctx.font='bold 13px monospace';
    ctx.textAlign='center';ctx.textBaseline='middle';
    ctx.fillText(l,CX+68*Math.cos(a),CY+68*Math.sin(a));
  });
  // Target line (if set)
  if(t>=0){
    const ta=(t-90)*Math.PI/180;
    ctx.beginPath();
    ctx.moveTo(CX,CY);
    ctx.lineTo(CX+R*0.85*Math.cos(ta),CY+R*0.85*Math.sin(ta));
    ctx.strokeStyle='rgba(255,200,0,0.5)';ctx.lineWidth=2;
    ctx.setLineDash([4,4]);ctx.stroke();ctx.setLineDash([]);
  }
  // Heading needle
  const na=(h-90)*Math.PI/180;
  // Back (south) — white stub
  ctx.beginPath();
  ctx.moveTo(CX-22*Math.cos(na),CY-22*Math.sin(na));
  ctx.lineTo(CX,CY);
  ctx.strokeStyle='#888';ctx.lineWidth=3;ctx.stroke();
  // Forward (red)
  ctx.beginPath();
  ctx.moveTo(CX,CY);
  ctx.lineTo(CX+78*Math.cos(na),CY+78*Math.sin(na));
  ctx.strokeStyle='#ef5350';ctx.lineWidth=3;ctx.stroke();
  // Hub
  ctx.beginPath();ctx.arc(CX,CY,6,0,Math.PI*2);
  ctx.fillStyle='#4fc3f7';ctx.fill();
  // Degree text
  ctx.fillStyle='#fff';ctx.font='bold 15px monospace';
  ctx.textAlign='center';ctx.textBaseline='middle';
  ctx.fillText(Math.round(h)+'°',CX,CY+28);
}

// Click on canvas → set target
cvs.addEventListener('click',e=>{
  const rect=cvs.getBoundingClientRect();
  const dx=e.clientX-rect.left-CX*(rect.width/W);
  const dy=e.clientY-rect.top -CY*(rect.height/W);
  let deg=Math.atan2(dy,dx)*180/Math.PI+90;
  if(deg<0)deg+=360;
  deg=Math.round(deg)%360;
  document.getElementById('ti').value=deg;
  sendGotoVal(deg);
});

function log(msg){
  const el=document.getElementById('log');
  const p=document.createElement('p');
  p.textContent=new Date().toLocaleTimeString()+' > '+msg;
  el.appendChild(p);el.scrollTop=el.scrollHeight;
  if(el.children.length>60)el.removeChild(el.firstChild);
}

async function pollStatus(){
  try{
    const r=await fetch('/api/status');
    const d=await r.json();
    heading=d.heading;target=d.target;
    drawCompass(d.heading,d.target);
    document.getElementById('hdg').textContent=d.heading.toFixed(1)+'°';
    document.getElementById('tgt').textContent=d.target>=0?d.target.toFixed(1)+'°':'---°';
    const sb=document.getElementById('sts');
    sb.innerHTML='<span class="badge '+d.status+'">'+d.status+'</span>';
  }catch(e){}
}

async function sendGotoVal(deg){
  const r=await fetch('/api/goto',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({heading:deg})});
  const d=await r.json();
  log('GOTO '+deg+'° → '+d.result);
}

async function sendGoto(){
  const deg=parseFloat(document.getElementById('ti').value);
  if(isNaN(deg)||deg<0||deg>=360){alert('Enter 0–359');return;}
  sendGotoVal(Math.round(deg));
}

async function sendStop(){
  const r=await fetch('/api/stop',{method:'POST'});
  const d=await r.json();
  log('STOP → '+d.result);
}

async function sendCmd(){
  const cmd=document.getElementById('ci').value.trim().toUpperCase();
  if(!cmd)return;
  document.getElementById('ci').value='';
  const r=await fetch('/api/command',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({cmd})});
  const d=await r.json();
  log(cmd+' → '+d.result);
}

drawCompass(0,-1);
pollStatus();
setInterval(pollStatus,1000);
</script>
</body>
</html>
)rawliteral";

// =============================================================================
//  Web server handlers
// =============================================================================
void handleRoot() {
    server.send_P(200, "text/html", INDEX_HTML);
}

void handleStatus() {
    StaticJsonDocument<160> doc;
    doc["heading"]      = headingKnown ? serialized(String(currentHeading, 2)) : serialized(String("-1"));
    doc["target"]       = serialized(String(targetHeading,  2));
    doc["status"]       = rotatorStatus;
    doc["headingKnown"] = headingKnown;
    String json;
    serializeJson(doc, json);
    server.send(200, "application/json", json);
}

void handleGoto() {
    if (!server.hasArg("plain")) { server.send(400, "application/json", "{\"error\":\"no body\"}"); return; }
    StaticJsonDocument<64> doc;
    if (deserializeJson(doc, server.arg("plain"))) { server.send(400, "application/json", "{\"error\":\"bad json\"}"); return; }
    float h = doc["heading"].as<float>();
    h = fmodf(h + 360.0f, 360.0f);
    targetHeading = h;
    server.send(200, "application/json",
        "{\"result\":\"OK\",\"target\":" + String(h, 1) + "}");
}

void handleStop() {
    targetHeading = -1.0f;
    motorStop();
    server.send(200, "application/json", "{\"result\":\"STOPPED\"}");
}

void handleCommand() {
    if (!server.hasArg("plain")) { server.send(400); return; }
    StaticJsonDocument<96> doc;
    if (deserializeJson(doc, server.arg("plain"))) { server.send(400); return; }
    String cmd    = doc["cmd"].as<String>();
    String result = processCommand(cmd);
    StaticJsonDocument<128> resp;
    resp["result"] = result;
    String json;
    serializeJson(resp, json);
    server.send(200, "application/json", json);
}

void handleNotFound() {
    server.send(404, "text/plain", "Not found");
}

// =============================================================================
//  Setup
// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println(F("\n\n=== nlOpenRotator ==="));

    pinMode(PIN_LED, OUTPUT);

    // Relay outputs — start with both relays off (HIGH = off on active-LOW module)
    pinMode(PIN_MOTOR_CW,  OUTPUT);
    pinMode(PIN_MOTOR_CCW, OUTPUT);
    motorStop();

    // WiFi access point
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.print(F("AP started  SSID="));
    Serial.print(AP_SSID);
    Serial.print(F("  IP="));
    Serial.println(WiFi.softAPIP());

    // HTTP routes
    server.on("/",            HTTP_GET,  handleRoot);
    server.on("/api/status",  HTTP_GET,  handleStatus);
    server.on("/api/goto",    HTTP_POST, handleGoto);
    server.on("/api/stop",    HTTP_POST, handleStop);
    server.on("/api/command", HTTP_POST, handleCommand);
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

    // Control loop at 100 ms intervals
    unsigned long now = millis();
    if (now - lastControlMs >= 100) {
        lastControlMs = now;
        controlLoop();
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
