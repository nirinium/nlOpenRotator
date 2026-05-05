# nlOpenRotator — Full Wiring Guide (AC Rotator, 3-wire)

Small RCA-style antenna rotators (VH-226, VH-100A, etc.) are driven by a
**24 V AC single-phase motor with two separate windings** — one for CW, one for CCW.
The original controller simply applies 24 V AC to one winding at a time.
This guide replaces that controller with an ESP32 + two relays + a small transformer.

These rotators have **3 motor terminals only** (COM, CW, CCW).
There is no position potentiometer.  Position is tracked by **timed dead-reckoning**:
calibrate how long (ms) the motor takes per degree, and the firmware times each move.

> **Safety first.**  
> The transformer output is low-voltage AC (24 V) — not directly dangerous but
> it can still arc, damage components, or start a fire if short-circuited.  
> Keep the mains side of the transformer fully enclosed and fused.  
> Never connect the transformer output directly to the ESP32.

---

## Bill of materials

| Qty | Part | Notes |
|---|---|---|
| 1 | ESP32 dev board | Any 30- or 38-pin board |
| 1 | Dual-secondary transformer | 120 V primary; 24 V AC secondary (motor) + 9 V AC secondary (ESP32 power); e.g. original RCA controller transformer |
| 2 | 1-channel 5 V relay module | Optocoupler-isolated, active-LOW trigger — one module per winding (very common on Amazon/AliExpress) |
| 4 | 1N4007 diode | Full-wave bridge rectifier for 9 V secondary |
| 1 | 1000 µF / 16 V electrolytic capacitor | Filter cap for rectified 9 V rail |
| 1 | MP1584 buck module | Adjustable DC-DC step-down; set output to 5.0 V |
| 1 | SPST toggle switch (6 A 250 VAC) | Mains on/off — wired in series with primary Hot wire |
| — | Hookup wire, connectors, enclosure | — |

---

## Transformer wiring

Your transformer has three sets of wires:

| Wires | Voltage | Use |
|---|---|---|
| 2-wire (Red + Black, from wall plug) | 120 V AC | Primary (mains input) |
| 2-wire | 24 V AC | Motor relay supply |
| 2-wire | 9 V AC | ESP32 power supply (via MP1584) |

### Primary — mains switch wiring

Wire a SPST toggle switch in series with one primary wire:

```
Wall plug
  Red   ─────── Switch Prong 1 ── Switch Prong 2 ─────── Transformer Primary terminal A
  Black ────────────────────────────────────────────────── Transformer Primary terminal B
```

To identify Hot vs Neutral: check the plug — the **narrow blade = Hot**, **wide blade = Neutral**.
Trace which wire connects to each blade. If blades are the same width, use a multimeter on AC Volts,
probe one wire to ground — ~120 V = Hot, ~0 V = Neutral. Put the switch on **Hot**.

---

## Block diagram

```
  WALL PLUG (120 V AC)
  Hot   (Red)   ──[SPST Switch]──┐
  Neutral (Black) ───────────────┤
                        ┌────────┴──────────────────────┐
                        │         TRANSFORMER            │
                        │  Primary:     120 V            │
                        │  Secondary A:  24 V AC ────────┼──► Motor relay supply
                        │  Secondary B:   9 V AC ────────┼──► Bridge rect → MP1584 → 5 V → ESP32 VIN
                        └───────────────────────────────-┘

  24 V AC secondary:
    Hot ──► Relay A COM ── NO ──► Motor CW terminal
         └► Relay B COM ── NO ──► Motor CCW terminal
    Neutral ─────────────────────────────────────────► Motor COM

  9 V AC secondary:
    ──► 4× 1N4007 bridge rectifier ──► 1000 µF cap ──► MP1584 input
                                                        MP1584 out (5.0 V) ──► ESP32 VIN
```

---

## Schematic — relay module wiring

```
 ┌───────────────────────────────────────────────────────┐
 │     RELAY MODULE A — CW  (5 V, active LOW)          │
 │                                                       │
 │  VCC ◄──── 5 V (from ESP32 5V/VIN pin)               │
 │  GND ◄──── GND                                        │
 │  IN  ◄──── GPIO 25  (LOW = relay energised)           │
 │                                                       │
 │  Relay: COM ── 24 V AC hot                           │
 │         NO  ── Motor CW terminal                     │
 │         NC  ── (leave unconnected)                   │
 └───────────────────────────────────────────────────────┘

 ┌───────────────────────────────────────────────────────┐
 │     RELAY MODULE B — CCW  (5 V, active LOW)           │
 │                                                       │
 │  VCC ◄──── 5 V (from ESP32 5V/VIN pin)               │
 │  GND ◄──── GND                                        │
 │  IN  ◄──── GPIO 26  (LOW = relay energised)           │
 │                                                       │
 │  Relay: COM ── 24 V AC hot                           │
 │         NO  ── Motor CCW terminal                    │
 │         NC  ── (leave unconnected)                   │
 └───────────────────────────────────────────────────────┘

 Motor COM ─────────────────────────────── 24 V AC neutral (transformer secondary A)
```

> Most relay modules have **active-LOW** inputs — the relay energises when
> the ESP32 pin is pulled LOW.  The firmware handles this; see the note in
> `main.cpp`.

---

## How the AC motor is controlled

The rotator's motor has **three terminals** coming out on the cable:

| Terminal | Function |
|---|---|
| **COM** | Common (neutral) — always connected to 24 V secondary neutral |
| **CW** | CW winding — apply 24 V AC here to rotate clockwise |
| **CCW** | CCW winding — apply 24 V AC here to rotate counter-clockwise |

Energise CW or CCW (never both at once); leave the other de-energised.  
The two relays do exactly that, each one switching 24 V AC to one winding.

---

## Rotator cable pinout (probe with a multimeter — do not assume)

Typical 3-wire RCA rotator cable:

| Wire colour (common) | Function |
|---|---|
| Red or White | Motor CW winding |
| Black or Blue | Motor CCW winding |
| Green or Yellow | Motor common (neutral) |

**Always verify with a meter before wiring.**  With the rotator unplugged:
- CW and CCW windings will read as a winding resistance (~50–200 Ω) to COM.
- COM to either winding reads the same resistance.  Measuring across CW→CCW reads double (series).

---

## Firmware — timed dead-reckoning

With no pot, position is inferred from motor run time.  Key constants and commands:

```cpp
// In src/main.cpp — adjust to match your rotator
float MS_PER_DEG = 125.0f;  // ms per degree  (125 ≈ 45 s for a full 360°)
float DEADBAND   = 4.0f;    // stop within ±4° of target
```

### Calibration procedure

1. Flash the firmware.  Open the serial monitor at **115200 baud**.
2. Point the antenna to a known bearing (e.g. North = 0°) by hand.
3. Send `SETHOME 0` — this sets the reference and enables GOTO.
4. Send `GOTO 90` and time how long the motor runs until it stops (note seconds).
5. Measure the actual angle turned with a compass or by observing a known landmark.
6. `MS_PER_DEG = elapsed_ms / actual_degrees`  
   Example: motor ran 11.25 s (11250 ms) for 90° → MS_PER_DEG = 11250 / 90 = **125**
7. Send `MSPERDEG 125` (or update the constant in `main.cpp` and reflash).
8. Send `SETHOME 0` again to reset after the test run.

> Dead-reckoning accumulates error over multiple moves.  
> Re-run `SETHOME` whenever you can visually confirm the antenna direction.

### Motor control functions (already in main.cpp — for reference)

```cpp
void motorStop()  { /* captures heading from elapsed time, de-energises relays */ }
void motorCW()    { /* calls motorStop() first, then snapshots heading+time, energises CW relay */ }
void motorCCW()   { /* calls motorStop() first, then snapshots heading+time, energises CCW relay */ }
```

---

## Complete wiring checklist

- [ ] Transformer primary fused (original controller already has this; if using a bare transformer, add a 0.5 A fuse)
- [ ] **Unplug the controller from the wall before opening it**
- [ ] Transformer secondary connected to relay COM terminals only (not ESP32)
- [ ] Motor COM wire connected to transformer secondary neutral
- [ ] Relay module VCC connected to ESP32 5 V (VIN) pin, **not** 3.3 V
- [ ] Relay module GND shared with ESP32 GND
- [ ] IN1 → GPIO 25, IN2 → GPIO 26
- [ ] ESP32 powered from MP1584 buck module (9 V AC secondary → bridge rectifier → MP1584 → 5 V DC → ESP32 VIN), **or** USB
- [ ] MP1584 output trimmer adjusted to **5.0 V** before connecting ESP32
- [ ] `MS_PER_DEG` calibrated for your rotator's speed (see Calibration above)
- [ ] `SETHOME` sent after pointing antenna to a known bearing on first use

---

## Using the original RCA controller box

If you have the original controller, you can reuse its built-in transformer
rather than buying a separate one.  **Unplug it from the wall before opening.**

### Option A — tap the transformer secondary (recommended)

This is the cleanest approach.  The original switch is bypassed entirely.

1. Open the controller box (usually 2–4 Phillips screws on the base).
2. Locate the transformer — a small laminated-core block, usually bolted to the
   chassis.  It has two sets of wires:
   - **Primary** (mains side) — thicker wires going to the mains cord and fuse.
     **Do not touch these.**
   - **Secondary** (low-voltage AC side) — thinner wires, typically two, going
     to the rotary switch or terminal strip.  These carry ~24 V AC.
3. Trace the secondary wires from the transformer to the switch/terminal strip.
   Cut or unsolder them at the switch end, leaving as much wire length as
   possible on the transformer side.
4. Run those two secondary wires out of the controller box (use a grommet or
   strain relief through an existing hole or a small new hole in the side).
5. These become your **24 V AC hot** and **24 V AC neutral** supply for the
   relay COM terminals.

```
Inside original controller box:
  Mains cord ──► [Fuse] ──► Transformer primary
                             Transformer secondary ──► (wires exit box)
                                                           │
                                          ┌────────────────┴─────────────────┐
                                          ▼                                   ▼
                                   Relay A COM                          Relay B COM
                                   Relay A NO ──► Motor CW terminal
                                   Relay B NO ──► Motor CCW terminal

  Motor COM ──────────────────────────────────────────► Transformer secondary neutral
```

6. The original switch, indicator lamp, and meter inside the controller can be
   left disconnected — they are no longer in the circuit.
7. Drill or punch a second hole for the motor cable (or reuse the existing
   cable entry if you are keeping the cable attached).
8. Power the ESP32 from USB; plug the original controller's mains cord back in
   to power the transformer.

### Option B — intercept at the rotator cable connector (no disassembly)

If you would rather not open the controller:

1. Unplug the rotator cable from the controller's output socket.
2. Identify the CW, CCW, and COM pins using a multimeter (see *Rotator cable
   pinout* above).
3. Wire your relay NO contacts and neutral to a mating connector (or bare wire
   directly) for those three pins.
4. Leave the original controller plugged into the wall — its transformer stays
   live and the secondary feeds through the internal wiring to the output socket.
5. Set the original rotary switch to the centre/off position (if it has one)
   so it doesn't apply voltage to the windings independently of the relays.

> **Risk:** if the original switch is accidentally moved to CW or CCW while
> the ESP32 relay is also energised in the opposite direction, both windings
> see voltage simultaneously.  Option A avoids this entirely.

---

## Powering the ESP32 from the 9 V AC secondary (MP1584 buck module)

The original controller has a 9 V AC secondary tap (used for the indicator meter/lamp).
You can rectify this to power the ESP32, eliminating the need for a separate USB supply.

**MP1584 module specs:**  Input 4.5–28 V DC · Output adjustable · Up to 3 A  
The ESP32 draws ~250 mA average, well within rating.

### Circuit

```
9 V AC (transformer secondary tap)
         │           │
        [+]         [–]   ← polarity doesn't matter for AC input to bridge
         │           │
   ┌─────┴───────────┴─────┐
   │    Bridge rectifier   │   4× 1N4007 diodes  (or a W005G / DB107 package)
   │    (full-wave)        │
   └─────┬───────────┬─────┘
        DC+          DC–
         │            │
      [1000 µF]       │    ← electrolytic cap, 16 V or higher, + toward DC+
      [16 V cap]      │      smooths rectified DC to ~11–12 V
         │            │
    IN+ ──┤            ├── IN–
    │    MP1584 module     │
    OUT+ ─┤            ├── OUT–
         │            │
       5 V DC        GND
         │            │
    ESP32 VIN (5V)  GND
```

### Step-by-step

1. **Rectify:** Wire the four 1N4007 diodes as a full-wave bridge, or use a
   single W005G/DB107 package.  The two AC inputs go to the AC~ pins;
   DC+ and DC− come out the other two pins.
2. **Filter:** Solder a 1000 µF / 16 V electrolytic capacitor across DC+ and DC−,
   observing polarity.  This smooths the pulsed DC to a steady ~11–12 V.
3. **Adjust the MP1584:** Before wiring to the ESP32, power it from a bench
   supply (or temporarily from the rectifier) and adjust the trimmer pot until
   the output reads **5.0 V** on a multimeter.
4. **Connect:** Wire MP1584 OUT+ → ESP32 **VIN** pin (5 V),
   MP1584 OUT− → ESP32 **GND**.  Do **not** use the 3.3 V pin — always feed
   regulated 5 V to VIN.
5. **Isolate:** Keep all DC wiring clear of the 24 V AC relay circuit.
   The two transformer secondaries share a common core but their outputs are
   electrically independent — treat them as separate circuits.

### Bridge rectifier diode wiring (1N4007)

```
        AC~              AC~
  9V AC ─┤►├─ DC+ ──┬──────────── (+) to MP1584 IN+
         │          │
         │       [1000µF]
         │          │
  9V AC ─┤►├─ DC− ──┴──────────── (–) to MP1584 IN−  / GND
```

A pre-made W005G or DB107 bridge rectifier package has four labelled legs
(AC~, AC~, +, −) and is simpler to solder.

> **Update the wiring checklist:** replace the "5 V USB wall-wart" item with
> "MP1584 output adjusted to 5.0 V and wired to ESP32 VIN + GND".

---

## Optional: station-controller integration

If you want the ESP32 to sit inside the original controller enclosure:

```
Transformer 24 V AC secondary ─── relay contacts ─── motor cable connector
Transformer  9 V AC secondary ─── bridge rectifier ─── MP1584 ─── ESP32 VIN

All AC wiring kept on one side of the enclosure.
ESP32 + relay signal wires on the other side.
Use barrier strips (screw terminals) for all connections.
```

Keep the 24 V AC wiring well-separated from the ESP32 and relay signal wires.
