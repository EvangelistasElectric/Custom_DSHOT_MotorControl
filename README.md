# Custom_DSHOT_MotorControl
dshot_MotorControl.h

Header-only, multi-ESC DShot600 + Telemetry + PI control for Teensy 4.x.
Includes a flexible DemoSequence timeline (ramps/holds), feed-forward + PI with anti-windup, slew limiting, EMA filtering with spike gate, and a built-in step response profiler (rise90/settle/overshoot).

Ideal for ESC/motor characterization with inertia wheels, bench testing, and embedded control prototyping.

Table of Contents

Features

Hardware & Protocols

Getting Started

1) Drop-in

2) Wire it

3) Quick start sketch

Default Demo Sequence

API Reference

DemoSequence

MotorController<ESC_PIN>

Control Loop Details

Mapping (RPM→µs)

Filtering & Spike Gate

PI + Anti-windup + Slew

DShot600 Transmission

Telemetry Frames

Tuning Guide

Timing & Performance

Safety Notes

Troubleshooting

FAQ

Examples

Project Structure

License

Features

Multi-ESC: Instantiate MotorController<Pin> as many times as you need; each instance owns its own PI, telemetry, timeline, and profiler.

DShot600: 1 kHz packet cadence (configurable), precise bit-banged timing using digitalWriteFast() with compile-time pin.

Telemetry: APD/KISS-style 10-byte frames (Temp/V/A/mAh/eRPM + CRC-8). Averaged readouts every 15 ms by default.

Control: Linear feed-forward (RPM→µs map) + PI with anti-windup back-calculation and slew limiter.

Filtering: EMA (α=0.2) with dynamic spike gate to suppress outlier RPM readings.

Profiler: Step response logging (rise90, settle ±2% with 50 ms dwell, overshoot) whenever target Δ≥200 RPM.

DemoSequence: Build timelines from {duration_ms, target_rpm} points; supports looping and hot-swapping sequences.

Header-only: No .cpp. Include and go.

Hardware & Protocols

MCU: Teensy 4.0/4.1 (600 MHz ARM Cortex-M7). Other high-speed MCUs may work but are untested.

ESC: APD 40F3 (or DShot600-capable ESC). Signal pin must be 3.3V logic compatible (Teensy is 3.3V).

DShot: DShot600 timings (bit period ~1.67 µs). Implementation sends 16 bits (11-bit throttle, 1 telemetry bit, 4-bit checksum).

Telemetry: UART (8N1). Default start at 115200, auto-scan option across {115200, 230400, 57600, 250000}.

Pinning (example)

ESC signal: Teensy digital pin 4 (or any you template into the controller).

APD telemetry: ESC’s T pin → Teensy RX of the chosen UART (e.g., Serial1 RX1).

Common ground between ESC power source, ESC, and Teensy.

Getting Started
1) Drop-in

Place the header in your repo:

lib/
 └── dshot_MotorControl.h
src/
 └── main.ino

2) Wire it

ESC signal → Teensy pin (compile-time template parameter).

ESC telemetry T → Teensy Serial RX pin (e.g., Serial1 RX1).

Share ground. Power ESC/motor per your ESC’s spec.

3) Quick start sketch
#include <Arduino.h>
#include "lib/dshot_MotorControl.h"

// One ESC on pin 4, telemetry on Serial1
MotorController<4> esc1(Serial1, "Serial1 (RX1)");

void setup() {
  esc1.begin();
  esc1.setSlewLimit(12000);  // ~12 µs/ms for snappy “rapid” steps
  // esc1.setLoop(false);    // one-shot run if you want
}

void loop() {
  esc1.tick();               // run control + telemetry + DShot @ 1kHz
}

Default Demo Sequence

The built-in default sequence matches your test plan (with an inertia wheel):

Ramp to 3000 RPM (5 s)

Hold 3000 RPM (5 s)

Drop to 1500 RPM (30 ms)

Hold 1500 RPM (5 s)

Ramp to 3000 RPM (5 s)

Rise to 4500 RPM (30 ms)

Hold 4500 RPM (5 s)

Rise to 6000 RPM (30 ms)

Hold 6000 RPM (5 s)

Rise to 7500 RPM (30 ms)

Hold 7500 RPM (5 s)

Ramp down to 6000 RPM (5 s)

Drop to 4500 RPM (30 ms)

Hold 4500 RPM (5 s)

Ramp down to 0 RPM (5 s)

Looping is enabled by default; disable via setLoop(false).

API Reference
DemoSequence

Lightweight builder for a time-ordered set of linear ramps built from “points”:

struct DemoSequence {
  struct Point { uint32_t duration_ms; float rpm; };
  DemoSequence(std::initializer_list<Point> pts, bool loop=true);
  void build(std::initializer_list<Point> pts, bool loop=true);
  bool addPoint(uint32_t duration_ms, float rpm);
  void setLoop(bool loop);
  bool loop() const;
  uint32_t cycleMs() const;
  float targetAt(uint32_t msInto) const; // linear interpolation inside segment
};


Usage:

DemoSequence seq({
  {5000, 3000},  // 0->3000 in 5s
  {5000, 3000},  // hold 5s
  {30,   1500},  // rapid down 30ms
  {5000, 0}      // back to zero 5s
}, true);

MotorController<ESC_PIN>

Encapsulates a full controller for a single ESC on a specific compile-time pin:

template<uint8_t ESC_PIN>
class MotorController {
public:
  explicit MotorController(HardwareSerial& tlm, const char* tlm_name="SerialX");

  void begin();                    // init pin/serial, arm ESC, print anchors
  void tick();                     // run control, telemetry, print

  // Sequences
  void setSequence(const DemoSequence& seq);
  void setLoop(bool loop);
  uint32_t cycleMs() const;

  // Mapping anchors (RPM↔µs linear map)
  void setAnchors(float rpmA, unsigned usA, float rpmB, unsigned usB);

  // Tuning / IO
  void setSlewLimit(unsigned us_per_s);
  void enableTelemetryScan(bool enable);
  void setTelemetryBaud(unsigned baud);
};


Construction:

MotorController<4> esc(Serial1, "Serial1 (RX1)");


Using a template parameter for ESC_PIN enables digitalWriteFast(ESC_PIN, …), which is critical for low-jitter DShot timing.

Control Loop Details

The loop is executed at the packet cadence (default 1 kHz):

Target RPM: from the active DemoSequence at msInto = millis() - cycleStart.

Telemetry: UART ISR-style polling reads 10-byte frames, decoded into volts/amps/temp/eRPM/mAh. Mechanical RPM = eRPM / pole pairs.

Filtering:

Bootstrap: EMA starts at first valid reading to avoid large initial error.

Spike gate: Ignore changes larger than max(800 RPM, 30% of max(target, current)).

EMA: rpm_filt = α * rpm + (1 - α) * rpm_filt with α = 0.2.

Feed-forward:

Linear map: anchor points (RPM_A, US_A) and (RPM_B, US_B) define slope in µs per RPM.

Optional bias FF_OFFSET_US (default +25 µs) nudges response to compensate static losses.

PI with anti-windup:

err = target - rpm_filt

unsat = ff_us + FF_OFFSET_US + KP*err + I

sat = clamp(unsat, MIN_US, MAX_US)

Update I only when not pushing further into saturation. Apply back-calculation: I += KAW * (sat - unsat).

Clamp I to ±I_CLAMP_US.

Zero-cross boost: when error flips sign, halve I to unwind quickly.

Slew limiter:

Maximum change per packet based on MAX_SLEW_US_PER_S.

Prevents unrealistic jumps and helps avoid current spikes.

DShot600 send:

Convert µs → throttle (48..2047).

Build packet with 4-bit checksum.

Bit-bang 16 bits with strict high/low nanosecond windows.

Mapping (RPM→µs)

Default anchors (tuned for your inertia wheel):
RPM_A=2000 @ US_A=1310, RPM_B=6400 @ US_B=1795
Slope ≈ 0.111 µs/RPM.

Update at runtime:

esc.setAnchors(1800, 1300, 7000, 1850);

Filtering & Spike Gate

Spike gate protects EMA from transient bad telemetry:

gate = max(800 RPM, 0.30 * max(targetRPM, currentRPM))


If |rpm_raw - rpm_filt| > gate, reuse rpm_filt for this tick.

PI + Anti-windup + Slew

KP_US_PER_RPM = 0.04

KI_US_PER_RPM_PER_S = 0.030

I_CLAMP_US = 120

KAW = 0.20 (back-calculation)

MAX_SLEW_US_PER_S = 4000 by default; set higher for 30 ms “rapid” segments (e.g., 12k–40k).

DShot600 Transmission

Bit period 1670 ns

1 → high 1250 ns, low 420 ns

0 → high 830 ns, low 840 ns

Packet: 16 bits (MSB first): throttle[10:0] | telemetry_bit | checksum[3:0]

Telemetry Frames

Assumes APD/KISS-style 10-byte frame:

[0]: Temp (int8_t °C)
[1-2]: Voltage *100 (uint16_t)
[3-4]: Current *100 (uint16_t)
[5-6]: mAh (uint16_t)
[7-8]: eRPM/100 (uint16_t)  ← stored as rpm16; value*100 = eRPM
[9]: CRC-8 (poly 0x07, init 0x00) over bytes 0..8


Derived:

mechRPM = eRPM / pole_pairs (default 14)

Averages printed every 15 ms:

t=3s targetRPM=3000 us=1450->thr=520 | V=16.80 A=5.23 Temp=42.0C eRPM=45000 mechRPM=3214 mAh=50 n=15

Tuning Guide

Map anchors (setAnchors)

Run a few steady points, note µs that yields desired RPM, update anchors so the linear fit matches your motor/ESC.

Slew limit (setSlewLimit)

For 30 ms “rapid” transitions, raise to 12000–40000 µs/s depending on aggressiveness and current limits.

KP/KI

Start with defaults. Increase KP until small overshoot appears, then back off ~20%.

Increase KI until steady-state error is near zero without causing “breathing”.

FF offset

Adjust FF_OFFSET_US ±5..±15 to pre-bias load. Positive offsets generally reduce lag on spin-up.

Spike gate & EMA

If telemetry is noisy: lower α (e.g., 0.15) and/or increase spike gate fraction.

Profiler output helps confirm targets:

rise90 (µs) — time to 90% of step

settle (µs) — time to remain within ±2% for ≥50 ms

overshoot (%) — peak deviation beyond target

Timing & Performance

Teensy 4.x @ 600 MHz comfortably meets DShot600 timing with digitalWriteFast().

The bit loop disables interrupts for ~16 * 1.67 µs ≈ 26.7 µs per packet.

Packet cadence = 1 kHz (configurable in the class).

Avoid long blocking code in loop(); call tick() quickly and consistently.

Multiple controllers: call each tick() sequentially — timing is tight but acceptable for 1–2 ESCs at 1 kHz on T4.x.

Safety Notes

Bench-test with prop removed or with a safe inertia wheel.

Confirm signal polarity and ground before powering.

Do not raise slew limits or gains without current limiting in place.

Ensure telemetry UART isn’t shared between controllers.

Consider adding thermal/current cutbacks using A and TempC from telemetry before running high power tests.

Troubleshooting

No spin / no response

Verify ESC is armed (library sends 0 throttle then idle throttle).

Check signal pin wiring and that template pin matches hardware.

Ensure ESC supports DShot600.

Garbage telemetry

Confirm baud (default 115200). Try setTelemetryBaud(230400) or enable auto scan with enableTelemetryScan(true).

Verify the T wire → Teensy RX pin of your chosen UART.

Choppy response

Slew limit too low; increase via setSlewLimit(12000) or higher.

Improve map anchors and FF offset.

Overshoot / ringing

Decrease KP, decrease KI, or increase KAW slightly (with caution).

Consider slightly higher EMA smoothing (lower α).

FAQ

Q: Can I run two ESCs?
A: Yes. Create two controllers with different pins and UARTs:

MotorController<4> A(Serial1, "Serial1");
MotorController<5> B(Serial2, "Serial2");


Q: Can I use a runtime pin instead of a template?
A: Possible but not recommended; you’d lose digitalWriteFast() timing guarantees. The template keeps jitter low.

Q: How do I make the rapid transitions truly step-like?
A: Set those durations to 1–2 ms in the DemoSequence and raise the slew limit (e.g., 30000–40000 µs/s). Watch current limits.

Q: Can I log to SD?
A: Not built-in, but trivial to add — hook into the 15 ms print block or create a callback.

Examples

Custom 7-point sequence, looped:

DemoSequence custom({
  {5000, 3000},
  {3000, 3000},
  {4000, 1500},
  {3000, 3000},
  {5000, 4500},
  {5000, 6000},
  {5000,    0}
}, true);

MotorController<4> esc(Serial1, "Serial1");
void setup() {
  esc.begin();
  esc.setSequence(custom);
}
void loop() { esc.tick(); }


Two controllers with different personalities:

MotorController<4> escA(Serial1, "Serial1");
MotorController<5> escB(Serial2, "Serial2");

void setup(){
  escA.begin();
  escB.begin();

  escA.setSlewLimit(15000);  // aggressive
  escB.setSlewLimit(8000);   // gentle

  // Short loop for B
  DemoSequence mini({
    {2000, 2500}, {1000, 2500}, {30, 4000}, {2000, 4000}, {2000, 0}
  }, true);
  escB.setSequence(mini);
}

void loop(){ escA.tick(); escB.tick(); }

Project Structure
lib/
 └── dshot_MotorControl.h      # the entire library (header-only)
src/
 └── main.ino                  # your sketch using MotorController
README.md                      # this file

License

MIT (or your preferred OSS license). Add a LICENSE file in the repo root.
