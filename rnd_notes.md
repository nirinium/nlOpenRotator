# nlOpenRotator — R&D Notes

---

## End Stop Detection & Reboot Behavior
*2026-05-20*

### Hardware Reality
The RCA VH-226 (and similar 3-wire AC antenna rotators) have **no sensors of any kind**:
- No limit switches at the end stops
- No potentiometer or encoder for position feedback
- Motor simply stalls against the mechanical hard stop at both ends of travel

The ESP32 receives **no electrical signal** when an end stop is hit.

### End Stop Handling
Protection is purely software-based via dead-reckoning estimates in `controlLoop()`:

- **Limit guard** — if estimated heading reaches `HDG_MIN` (CCW stop) or `HDG_MAX` (CW stop), the motor is force-stopped (`LIMIT_HIT` status)
- **Runaway guard** — if the motor has run longer than `(HDG_MAX - HDG_MIN) * MS_PER_DEG * 1.5`, it is force-stopped (`RUNAWAY` status)

If dead-reckoning drifts (miscalibrated `MS_PER_DEG`, brownout mid-rotation, etc.), the firmware can overshoot the limit and the motor will grind against the mechanical stop until the runaway timer fires.

### ESP32 Reboot While Rotating
1. Relay pins float briefly during boot before `setup()` sets them `HIGH` — potential for a brief glitch pulse to the relay coil (hardware risk, low probability)
2. Motor de-energizes once `setup()` runs (both relay pins driven `HIGH`)
3. Last heading is restored from NVS — but NVS heading is only written every **10 seconds** (`NVS_HDG_INTERVAL_MS = 10000`) or on explicit `STOP`, so restored position can lag up to 10 seconds of actual rotation
4. `hdgRestored = true` flag is set, triggering the yellow **"HEADING RESTORED FROM NVS — VERIFY ANTENNA POSITION BEFORE GOTO"** banner in the web UI

### Risk
If the rotator was moving when power was lost, the restored heading will be stale/wrong. A subsequent `GOTO` could drive the antenna past the physical end stop because the firmware believes the antenna is in a different position than it actually is.

### Safe Recovery Procedure After Unexpected Reboot
1. Do **not** issue `GOTO` commands until position is re-established
2. Run `TIMECCW` — drives antenna to the CCW hard stop
3. Run `STOP` — auto-homes to physical 0° and re-enables `GOTO`
4. Issue `HOMEDIR <bearing>` if compass orientation needs to be re-applied

---

## Reboot-Safety Mitigations Implemented
*2026-05-20*

### 1. Relay Glitch Prevention (Boot-time)
**Problem:** ESP32 GPIO pins float LOW briefly during boot before `setup()` sets them HIGH. Since relay modules are active-LOW, this can pulse a relay coil.

**Fix:** Added `digitalWrite(PIN_MOTOR_CW, HIGH)` and `digitalWrite(PIN_MOTOR_CCW, HIGH)` as the very first lines of `setup()`, before the brownout disable and `Serial.begin()`. On ESP32/Arduino, `digitalWrite` before `pinMode` pre-stages the output latch — when `pinMode` sets the pin to OUTPUT a moment later, it comes up HIGH from the start with no LOW glitch.

### 2. NVS "Motor Running" Dirty Flag
**Problem:** On unexpected reboot, the restored NVS heading could be stale (up to 10 s of untracked rotation), causing a subsequent GOTO to overshoot the physical end stop.

**Fix:** Added a `motordirty` boolean key in NVS:
- **Set `true`** in `motorCW()` / `motorCCW()` immediately after relay energizes
- **Set `false`** in `motorStop()` only when a direction was active (clean stop)
- **Checked on boot:** if `motordirty == true`, heading is invalidated (`headingKnown = false`), NVS flags are cleared, serial warning printed. GOTO stays disabled until re-homed via `TIMECCW` + `STOP`.
- **Pre-move snapshot:** current heading is force-written to NVS at the start of each rotation so the pre-move position is always preserved regardless of the dirty flag.

### 3. More Frequent NVS Heading Writes While Rotating
**Problem:** `NVS_HDG_INTERVAL_MS = 10000` meant up to 10 s of motor travel (~60°) could be unrecorded.

**Fix:** `controlLoop()` now writes NVS heading every **3 seconds** while `motorDirection != 0`, reducing worst-case untracked travel to ~18°. The 10 s throttle still applies when idle to minimize flash wear.

---
