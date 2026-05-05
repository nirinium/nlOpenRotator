# nlOpenRotator

ESP32-based controller for small TV / RCA antenna rotators.  
Provides a web UI (compass rose, click-to-target) and a serial command interface.

---

## Hardware

### Components needed

| Part | Example |
|---|---|
| Microcontroller | ESP32 dev board (any 38-pin or 30-pin) |
| Motor driver | 2× 1-channel 5 V relay modules (optocoupler-isolated, active-LOW) |
| AC transformer | 24 V AC / 1 A (wall-wart or chassis-mount) |
| Rotator | RCA VH-226 / VH-100A or similar AC antenna rotator |

### Wiring overview

See [wiring.md](wiring.md) for full schematics, pot voltage divider diagrams, and calibration steps.

```
RCA Rotator                   ESP32
────────────────────────────────────────────────────────
Motor CW winding  ──→  Relay 1 NO contact
Motor CCW winding ──→  Relay 2 NO contact
Motor common      ──→  24 V AC transformer secondary neutral

Relay 1 COM       ←──  24 V AC transformer secondary hot
Relay 2 COM       ←──  24 V AC transformer secondary hot
Relay module IN1  ←──  GPIO 25  (active LOW)
Relay module IN2  ←──  GPIO 26  (active LOW)
Relay module VCC  ←──  ESP32 5 V (VIN pin)
Relay module GND  ←──  GND
```

> The ESP32 never connects to the 24 V AC circuit.
> Relay contacts are the only bridge between the transformer and motor.

---

## Software

### Build with PlatformIO

```
pio run -t upload
```

Or open the folder in VS Code with the PlatformIO extension and click **Upload**.

---

## Configuration (top of `src/main.cpp`)

```cpp
const char* AP_SSID = "OpenRotator";   // WiFi network name
const char* AP_PASS = "rotator123";    // Change this!

int   CAL_ADC_MIN  = 150;   // raw ADC at 0°
int   CAL_ADC_MAX  = 3900;  // raw ADC at 360°
float CAL_DEG_MIN  = 0.0f;
float CAL_DEG_MAX  = 360.0f;

float DEADBAND     = 3.0f;  // stop within ±N degrees of target
int   MOTOR_SPEED  = 230;   // PWM 0-255
```

### Calibration procedure

1. Flash the firmware and open the serial monitor at **115200 baud**.
2. Manually point the rotator to **0° (North)**:  
   `STOP`  then type `RAWPOS` — note the value.  Set `CAL_ADC_MIN` to that value.
3. Manually point the rotator to **360° (full CW stop)**:  
   `RAWPOS` again — set `CAL_ADC_MAX` to that value.
4. Update the constants and re-flash, **or** send `CALMIN <value>` / `CALMAX <value>`
   over serial (takes effect immediately but resets on reboot).

---

## Web interface

1. Connect your phone/PC to the **OpenRotator** WiFi network (password: `rotator123`).
2. Open **http://192.168.4.1** in a browser.

| Feature | How to use |
|---|---|
| Compass rose | Click anywhere on the compass to instantly send a GOTO command |
| Heading input | Type a bearing (0–359) and press **GOTO** |
| STOP button | Halts the motor and clears the target |
| Serial console | Send any serial command from the browser |

The status badge updates every second via polling.

---

## Serial commands

Connect at **115200 baud** (e.g. `pio device monitor`).

| Command | Description |
|---|---|
| `GOTO <deg>` | Rotate to heading 0–359 |
| `STOP` | Halt immediately |
| `STATUS` | Print heading, target, state, speed, deadband |
| `SPEED <0-255>` | Set PWM motor speed (takes effect on next move) |
| `DEADBAND <deg>` | Arrival tolerance in degrees (min 0.5) |
| `CALMIN <adc>` | Set ADC value for 0° (temporary) |
| `CALMAX <adc>` | Set ADC value for 360° (temporary) |
| `RAWPOS` | Read raw ADC count — useful for calibration |
| `HELP` | List all commands |

---

## REST API

The web server runs on port 80.  All POST bodies are `application/json`.

| Method | Endpoint | Body / Response |
|---|---|---|
| GET | `/api/status` | `{"heading":123.4,"target":180.0,"status":"ROTATING_CW"}` |
| POST | `/api/goto` | `{"heading":270}` → `{"result":"OK","target":270.0}` |
| POST | `/api/stop` | (no body) → `{"result":"STOPPED"}` |
| POST | `/api/command` | `{"cmd":"STATUS"}` → `{"result":"HDG=..."}` |

---

## Pin summary

| GPIO | Function |
|---|---|
| 25 | Relay IN1 — CW (active LOW) |
| 26 | Relay IN2 — CCW (active LOW) |
| 2 | Status LED |
