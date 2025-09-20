# ⚡ Custom_DSHOT_MotorControl

Header-only, multi-ESC DShot600 + Telemetry + PI control for **Teensy 4.x**.  
Includes a flexible `DemoSequence` timeline (ramps/holds), feed-forward + PI with anti-windup, slew limiting, EMA filtering with spike gate, and a built-in step response profiler (rise90/settle/overshoot).  

Ideal for **ESC/motor characterization** with inertia wheels, bench testing, and embedded control prototyping.  

---

## 📜 License
This project is licensed under the **Evangelista's Electric Non-Commercial Open Source License (EENOSL), Version 1.0**.  

- ✅ Free for **personal, educational, and research** use  

See the [LICENSE](./LICENSE) file for full details.  

---

## 📑 Table of Contents
- [✨ Features](#-features)  
- [🔌 Hardware & Protocols](#-hardware--protocols)  
- [🚀 Getting Started](#-getting-started)  
- [🎯 Default Demo Sequence](#-default-demo-sequence)  
- [📚 API Reference](#-api-reference)  
- [⚙️ Control Loop Details](#️-control-loop-details)  
- [📊 Tuning Guide](#-tuning-guide)  
- [⏱️ Timing & Performance](#️-timing--performance)  
- [⚠️ Safety Notes](#️-safety-notes)  
- [🛠️ Troubleshooting](#️-troubleshooting)  
- [❓ FAQ](#-faq)  
- [💡 Examples](#-examples)  
- [📂 Project Structure](#-project-structure)  
- [📜 License](#-license)  

---

## ✨ Features
- 🌀 **Multi-ESC:** Instantiate `MotorController<Pin>` as many times as you need; each instance owns its own PI, telemetry, timeline, and profiler.  
- ⚡ **DShot600:** 1 kHz packet cadence (configurable), precise bit-banged timing using `digitalWriteFast()`.  
- 📡 **Telemetry:** APD/KISS-style 10-byte frames (Temp/V/A/mAh/eRPM + CRC-8). Averaged readouts every 15 ms.  
- 🎛️ **Control:** Linear feed-forward + PI with anti-windup and slew limiter.  
- 🧮 **Filtering:** EMA (α=0.2) with dynamic spike gate to suppress outlier RPM readings.  
- 📈 **Profiler:** Step response logging (rise90, settle ±2%, overshoot) whenever Δ≥200 RPM.  
- 🗓️ **DemoSequence:** Build timelines from `{duration_ms, target_rpm}` points; supports looping and hot-swapping sequences.  
- 🧩 **Header-only:** No `.cpp`. Just include and go.  

---

## 🔌 Hardware & Protocols
- **MCU:** Teensy 4.0 / 4.1 (600 MHz ARM Cortex-M7).  
- **ESC:** APD 40F3 (or DShot600-capable ESC). 3.3V logic required.  
- **DShot:** DShot600 timings (bit period ~1.67 µs). Sends 16-bit frames.  
- **Telemetry:** UART (8N1). Default 115200 baud, optional auto-scan {115200, 230400, 57600, 250000}.  

**Pinning Example:**  
- ESC signal → Teensy pin 4 (or any templated pin).  
- ESC telemetry T → Teensy Serial RX (e.g., `Serial1 RX1`).  
- Common ground between ESC, Teensy, and power source.  

---

## 🚀 Getting Started
1. **Drop-in**  
lib/
└── dshot_MotorControl.h
src/
└── main.ino

arduino
Copy code

2. **Wire it**  
- ESC signal → Teensy pin  
- ESC telemetry T → Teensy Serial RX pin  
- Share ground  

3. **Quick start sketch**  
```cpp
#include <Arduino.h>
#include "lib/dshot_MotorControl.h"

// One ESC on pin 4, telemetry on Serial1
MotorController<4> esc1(Serial1, "Serial1 (RX1)");

void setup() {
  esc1.begin();
  esc1.setSlewLimit(12000);  // snappy rapid steps
}

void loop() {
  esc1.tick();               // run control + telemetry + DShot @ 1kHz
}
🎯 Default Demo Sequence
The default sequence matches the inertia-wheel test plan:

1️⃣ Ramp to 3000 RPM (5 s)
2️⃣ Hold 3000 (5 s)
3️⃣ Drop to 1500 RPM (30 ms)
4️⃣ Hold 1500 (5 s)
5️⃣ Ramp to 3000 (5 s)
6️⃣ Rise to 4500 RPM (30 ms)
7️⃣ Hold 4500 (5 s)
8️⃣ Rise to 6000 RPM (30 ms)
9️⃣ Hold 6000 (5 s)
🔟 Rise to 7500 RPM (30 ms)
1️⃣1️⃣ Hold 7500 (5 s)
1️⃣2️⃣ Ramp down to 6000 (5 s)
1️⃣3️⃣ Drop to 4500 RPM (30 ms)
1️⃣4️⃣ Hold 4500 (5 s)
1️⃣5️⃣ Ramp down to 0 RPM (5 s)

Looping is enabled by default (setLoop(true)), disable for one-shot runs.

📚 API Reference
DemoSequence
Lightweight builder for linear RPM ramps.

cpp
Copy code
struct DemoSequence {
  struct Point { uint32_t duration_ms; float rpm; };
  DemoSequence(std::initializer_list<Point> pts, bool loop=true);
  void build(std::initializer_list<Point> pts, bool loop=true);
  bool addPoint(uint32_t duration_ms, float rpm);
  void setLoop(bool loop);
  bool loop() const;
  uint32_t cycleMs() const;
  float targetAt(uint32_t msInto) const;
};
MotorController<ESC_PIN>
Encapsulates full control for a single ESC.

cpp
Copy code
template<uint8_t ESC_PIN>
class MotorController {
public:
  explicit MotorController(HardwareSerial& tlm, const char* tlm_name="SerialX");
  void begin();
  void tick();
  void setSequence(const DemoSequence& seq);
  void setLoop(bool loop);
  uint32_t cycleMs() const;
  void setAnchors(float rpmA, unsigned usA, float rpmB, unsigned usB);
  void setSlewLimit(unsigned us_per_s);
  void enableTelemetryScan(bool enable);
  void setTelemetryBaud(unsigned baud);
};
📊 Tuning Guide
Anchors: Run steady RPMs, measure µs, update via setAnchors().

Slew limit: Raise (e.g., 12k–40k µs/s) for rapid 30 ms transitions.

KP/KI: Increase KP until mild overshoot, back off. Raise KI to kill steady-state error.

FF offset: Adjust ±5–15 µs for load compensation.

EMA & spike gate: Lower α or increase gate fraction if telemetry is noisy.

Profiler output: Check rise90, settle, overshoot values to tune.

⚠️ Safety Notes
❌ Never spin with props; use safe inertia wheels.

🔌 Confirm wiring & ground before powering.

🔥 Watch currents when raising slew/gains.

⚡ Ensure telemetry UART is dedicated.

🌡️ Add current/thermal cutbacks in real runs.

🛠️ Troubleshooting
No spin: Check arm, pin wiring, ESC supports DShot600.

Garbage telemetry: Try other baud, enable auto scan, check T wire → RX pin.

Choppy response: Slew limit too low.

Overshoot: Reduce KP/KI or tweak EMA.

📂 Project Structure
bash
Copy code
lib/
 └── dshot_MotorControl.h   # the entire library (header-only)
src/
 └── main.ino               # example sketch
README.md                   # this file
LICENSE                     # custom EENOSL v1.0 license
