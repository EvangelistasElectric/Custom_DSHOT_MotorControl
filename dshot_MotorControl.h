/**
 * @file    dshot_MotorControl.h
 * @author  Matthew Evangelista
 * @date    2025-09-22
 * @version 1.0
 *
 * @brief
 * Teensy 4.0 + APD 40F3 ESC control using DShot600 at 1 kHz with UART telemetry.
 * Provides a modular, header-only motor controller with feed-forward + PI control,
 * slew limiting, telemetry averaging, and a built-in response-time profiler.
 *
 * @section Overview
 * - Protocol: DShot600 (digital throttle) at 1 kHz packet cadence.
 * - Target MCU: Teensy 4.0 (600 MHz) for precise nanosecond timing.
 * - ESC: APD 40F3 (tested) with 10-byte UART telemetry frames (V, A, Temp, RPM, mAh).
 * - Control: feed-forward mapping (us vs. RPM) + PI loop with anti-windup and slew limit.
 * - Telemetry: EMA-filtered RPM with spike gating; averaged V/A/T/eRPM/mRPM between prints.
 * - Timeline: parametric DemoSequence to define ramp/hold steps in ms and target RPM.
 * - Profiling: rise90, settle band, overshoot estimation, step detector, per-instance.
 *
 * @section Features
 * 1) DemoSequence:
 *    - Build a composite test sequence from {duration_ms, target_rpm} points.
 *    - Linear interpolation between points; looped or single-shot operation.
 *    - Default sequence includes 0->3000 ramp and 4500/6000/7500 step checks.
 *
 * 2) MotorController<ESC_PIN>:
 *    - DShot600 bit-banging with exact high/low intervals (ns resolution).
 *    - 1 kHz packet stream; per-packet optional telemetry request bit.
 *    - Anchored us-to-RPM mapping (two-point affine) with clamping.
 *    - PI controller with anti-windup (KAW), integral clamp, and sign-change damp.
 *    - Slew limiting in microseconds per second to enforce smooth throttle motion.
 *    - Telemetry decode (CRC-8 0x07) and accumulation for print-period averages.
 *    - EMA filtered mechRPM with dynamic spike gate vs target/filt RPM.
 *    - Built-in response profiler for step changes >= STEP_DETECT_THRESH_RPM.
 *
 * @section Wiring
 * - ESC DShot signal: connect to Teensy pin ESC_PIN (template parameter).
 * - ESC Telemetry (T pin): connect to Teensy HW serial RX (e.g., Serial1 RX1).
 * - Common ground between battery/ESC and Teensy.
 * - Ensure voltage-level compatibility (APD signal input tolerant at 3.3 V logic).
 *
 * @section Timing
 * - PACKET_PERIOD_US       = 1000 us (1 kHz command cadence).
 * - DSHOT600_BIT_TIME_NS   = 1670 ns total per bit.
 * - DSHOT600_ONE_HIGH_NS   = 1250 ns high for logical 1.
 * - DSHOT600_ZERO_HIGH_NS  = 830 ns high for logical 0.
 * - PRINT_PERIOD_MS        = 15 ms averaged telemetry print cadence.
 * - SETTLE_HOLD_US         = 50,000 us dwell inside band to mark settle.
 *
 * @section Telemetry_Format
 * APD UART 10-byte frame (validated by CRC-8 poly 0x07 over first 9 bytes):
 *   [0]  int8  : Temp_C
 *   [1]  uint8 : V_H
 *   [2]  uint8 : V_L         -> Volts = ((V_H<<8)|V_L) / 100.0
 *   [3]  uint8 : I_H
 *   [4]  uint8 : I_L         -> Amps  = ((I_H<<8)|I_L) / 100.0
 *   [5]  uint8 : mAh_H
 *   [6]  uint8 : mAh_L       -> mAh   = ((mAh_H<<8)|mAh_L)
 *   [7]  uint8 : RPM_H
 *   [8]  uint8 : RPM_L       -> eRPM  = ((RPM_H<<8)|RPM_L) * 100.0
 *   [9]  uint8 : CRC-8(0x07)
 * mechRPM = eRPM / MOTOR_POLE_PAIRS.
 *
 * @section Control_Loop
 * - Feed-forward: rpmToUs_() via two anchors (RPM_A, US_A) and (RPM_B, US_B).
 * - Proportional: KP_US_PER_RPM * error.
 * - Integral: KI_US_PER_RPM_PER_S * error integrated with anti-windup KAW and clamp.
 * - Saturation: output clamped to [MIN_US, MAX_US]; slew limit applied per packet.
 * - Filtering: EMA on mechRPM; large spikes suppressed with dynamic gate vs target/filt.
 *
 * @section Response_Profiler
 * Triggered when |target change| >= STEP_DETECT_THRESH_RPM.
 * Tracks:
 *   - Rise90 time: first crossing of 90% of step amplitude.
 *   - Settle time: first entry into +/-BAND_PCT band around target that remains for SETTLE_HOLD_US.
 *   - Overshoot: peak excursion relative to step amplitude (up or down).
 * Emits a single "RESP:" line to Serial when settle is marked.
 *
 * @section API_Summary
 * DemoSequence
 *   - build({{ms, rpm}, ...}, loop)     : initialize sequence
 *   - addPoint(ms, rpm)                 : append a segment end-point
 *   - setLoop(bool) / loop()            : enable/disable repeating cycle
 *   - cycleMs()                         : total cycle duration (ms)
 *   - targetAt(ms_into_cycle)           : interpolated target RPM
 *
 * MotorController<ESC_PIN>
 *   - ctor(HardwareSerial& tlm, const char* name="SerialX")
 *   - begin()                           : init serial, arm ESC, start cycle clock
 *   - tick()                            : call from loop() at ~1 kHz cadence
 *   - setSequence(DemoSequence)         : change the test timeline
 *   - setLoop(bool)                     : toggle looping
 *   - cycleMs()                         : expose sequence cycle length
 *   - setAnchors(rpmA, usA, rpmB, usB)  : calibrate mapping
 *   - setSlewLimit(us_per_s)            : throttle slew constraint
 *   - enableTelemetryScan(bool)         : optional baud scan (rotates a small set)
 *   - setTelemetryBaud(unsigned)        : fixed baud if known
 *
 * @section Example
 * // Teensy 4.0, ESC signal on pin 4, telemetry on Serial1
 * #include "dshot_MotorControl.h"
 *
 * // ESC on pin 4
 * MotorController<4> motor(Serial1, "Serial1");
 *
 * void setup() {
 *   motor.begin();
 *   // Optional: anchors for your ESC/motor pair (RPM vs microseconds mapping)
 *   motor.setAnchors(2000.0f, 1310, 6400.0f, 1795);
 *   // Optional: custom sequence
 *   auto seq = makeDefaultSequence(true);
 *   motor.setSequence(seq);
 * }
 *
 * void loop() {
 *   motor.tick(); // run control + send DShot + process telemetry + print avg
 * }
 *
 * @section Configuration_Tuning
 * - Anchors: setAnchors(rpmA, usA, rpmB, usB) should be measured for your hardware.
 * - Gains: KP_US_PER_RPM, KI_US_PER_RPM_PER_S, KAW, I_CLAMP_US tune tracking vs stability.
 * - Feed-forward offset: FF_OFFSET_US can remove static bias under load.
 * - Slew: setSlewLimit(us_per_s) to constrain acceleration for large steps.
 * - Filtering: EMA_ALPHA, spike gate params balance noise rejection vs responsiveness.
 * - Profiler: STEP_DETECT_THRESH_RPM, BAND_PCT, SETTLE_HOLD_US define response metrics.
 *
 * @section Logging
 * Periodic print (every PRINT_PERIOD_MS) shows time within cycle and averaged telemetry
 * since last print, e.g.:
 *   t=12s  targetRPM=6000  us=1732->thr=1024  |  V=11.10  A=8.25  Temp=33.0C
 *   eRPM=84000  mechRPM=6000  mAh=40  n=18
 * Where n is the number of telemetry frames averaged since the last print.
 *
 * @section Known_Limits
 * - The print/log interval may mask sub-interval dynamics. Faster logging can expose
 *   overshoot and packet-timing variability; telemetry may appear noisier even when
 *   control remains stable. Control performance is governed by the 1 kHz command loop.
 * - DShot timings assume Teensy 4.x clocking; porting to other MCUs requires retiming.
 * - ESC telemetry format and scaling are specific to APD implementation tested here.
 *
 * @section Safety
 * - Spin-up is hazardous. Secure the motor and any inertia loads. Keep clear of props.
 * - Verify common ground and correct polarity. Use proper supply and current limits.
 * - Do not exceed ESC/motor ratings. Monitor temperature and current draw.
 *
 * @section License
 * This source is released under the Evangelista's Electric Non-Commercial Open Source
 * License (EENOSL) v1.0. See LICENSE for terms. Commercial use requires prior consent.
 *
 * @section Change_Log
 * - 2025-09-22: Initial public header-only release with DemoSequence, PI loop, telemetry,
 *               response profiler, and default 15-step sequence.
 */


#pragma once

#include <Arduino.h>
#include <math.h>
#include <initializer_list>

// ================== DEMO SEQUENCE ==================
struct Seg { uint32_t t0_ms, t1_ms; float rpm0, rpm1; };

class DemoSequence {
public:
  struct Point { uint32_t duration_ms; float rpm; };
  static constexpr size_t MAX_SEG = 32;

  DemoSequence(bool loop=true) : _loop(loop), _nseg(0), _cycle_ms(0) {}

  DemoSequence(std::initializer_list<Point> pts, bool loop=true) : DemoSequence(loop) {
    build(pts, loop);
  }

  void build(std::initializer_list<Point> pts, bool loop=true) {
    _nseg = 0; _cycle_ms = 0; _loop = loop;
    uint32_t t = 0;
    float prev = 0.0f; // start from rest
    for (auto &p : pts) {
      if (_nseg >= MAX_SEG) break;
      uint32_t t0 = t, t1 = t + p.duration_ms;
      _segs[_nseg++] = Seg{t0, t1, prev, p.rpm};
      t = t1; prev = p.rpm;
    }
    _cycle_ms = t;
  }

  bool addPoint(uint32_t duration_ms, float target_rpm) {
    if (_nseg >= MAX_SEG) return false;
    float prev = (_nseg == 0) ? 0.0f : _segs[_nseg-1].rpm1;
    uint32_t t0 = (_nseg == 0) ? 0 : _segs[_nseg-1].t1_ms;
    uint32_t t1 = t0 + duration_ms;
    _segs[_nseg++] = Seg{t0, t1, prev, target_rpm};
    _cycle_ms = t1; return true;
  }

  void setLoop(bool loop) { _loop = loop; }
  bool loop() const { return _loop; }
  uint32_t cycleMs() const { return _cycle_ms; }

  float targetAt(uint32_t msInto) const {
    if (_nseg == 0) return 0.0f;
    if (_loop && _cycle_ms > 0) msInto %= _cycle_ms;
    if (!_loop && msInto >= _cycle_ms) return 0.0f;
    for (size_t i = 0; i < _nseg; ++i) {
      const auto &s = _segs[i];
      if (msInto >= s.t0_ms && msInto < s.t1_ms) {
        float a = float(msInto - s.t0_ms) / float(s.t1_ms - s.t0_ms);
        return s.rpm0 + a * (s.rpm1 - s.rpm0);
      }
    }
    return 0.0f;
  }

private:
  bool _loop;
  Seg  _segs[MAX_SEG];
  size_t _nseg;
  uint32_t _cycle_ms;
};

// A handy default sequence (your 15-step plan).
static inline DemoSequence makeDefaultSequence(bool loop=true){
  return DemoSequence({
    { 5000, 3000 },  // 1) 0->3000 over 5s
    { 5000, 3000 },  // 2) hold 5s
    {   30, 1500 },  // 3) rapid to 1500 (30ms)
    { 5000, 1500 },  // 4) hold 5s
    { 5000, 3000 },  // 5) up 5s
    {   30, 4500 },  // 6) rapid to 4500
    { 5000, 4500 },  // 7) hold
    {   30, 6000 },  // 8) rapid to 6000
    { 5000, 6000 },  // 9) hold
    {   30, 7500 },  // 10) rapid to 7500
    { 5000, 7500 },  // 11) hold
    { 5000, 6000 },  // 12) down 5s
    {   30, 4500 },  // 13) rapid to 4500
    { 5000, 4500 },  // 14) hold
    { 5000,    0 }   // 15) down to 0 over 5s
  }, loop);
}

// =============== MOTOR CONTROLLER (templated on pin) ===============
template<uint8_t ESC_PIN>
class MotorController {
public:
  // ---- Tunables / constants ----
  static constexpr unsigned PACKET_PERIOD_US = 1000;  // 1 kHz
  static constexpr unsigned MIN_US = 1000;
  static constexpr unsigned MAX_US = 2000;
  static constexpr unsigned ARM_ZERO_MS  = 1000;
  static constexpr unsigned ARM_IDLE_MS  = 1500;
  static constexpr uint16_t ARM_IDLE_THR = 120;

  static constexpr unsigned DSHOT600_BIT_TIME_NS  = 1670;
  static constexpr unsigned DSHOT600_ONE_HIGH_NS  = 1250;
  static constexpr unsigned DSHOT600_ZERO_HIGH_NS = 830;

  static constexpr uint32_t PRINT_PERIOD_MS = 15;

  static constexpr float KP_US_PER_RPM        = 0.04f;
  static constexpr float KI_US_PER_RPM_PER_S  = 0.030f;
  static constexpr float I_CLAMP_US           = 120.0f;
  static constexpr float KAW                  = 0.20f;
  static constexpr int   FF_OFFSET_US         = -110;

  static constexpr float EMA_ALPHA            = 0.20f;
  static constexpr float MIN_SPIKE_GATE_RPM   = 800.0f;
  static constexpr float SPIKE_GATE_FRAC      = 0.30f;

  static constexpr float STEP_DETECT_THRESH_RPM = 200.0f;
  static constexpr float BAND_PCT               = 0.02f;
  static constexpr uint32_t SETTLE_HOLD_US      = 50000;
  static constexpr float RISE_HIGH_FRAC         = 0.90f;

  // ---- ctor ----
  explicit MotorController(HardwareSerial& tlm, const char* tlm_name="SerialX")
  : _tlm(&tlm), _tlmName(tlm_name) {
    setAnchors(2000.0f, 1310, 6400.0f, 1795); // default A/B anchors
    _seq = makeDefaultSequence(true);
  }

  // ---- public controls ----
  void setSequence(const DemoSequence& seq) { _seq = seq; }
  void setLoop(bool loop) { _seq.setLoop(loop); }
  uint32_t cycleMs() const { return _seq.cycleMs(); }

  void setAnchors(float rpmA, unsigned usA, float rpmB, unsigned usB) {
    _RPM_A = rpmA; _US_A = usA; _RPM_B = rpmB; _US_B = usB;
    _mapSlope = (float(int(_US_B)-int(_US_A))) / (_RPM_B - _RPM_A);
  }

  void setSlewLimit(unsigned us_per_s){ _maxSlew_us_per_s = us_per_s; }
  void enableTelemetryScan(bool en){ _scanBauds = en; }
  void setTelemetryBaud(unsigned baud){ _tlm->end(); _tlm->begin(baud); }

  void begin() {
    pinMode(ESC_PIN, OUTPUT);
    digitalWriteFast(ESC_PIN, LOW);

    Serial.begin(115200);
    uint32_t _t0 = millis();
    while (!Serial && (millis() - _t0) < 1000) { /* wait up to 1s for USB */ }

    Serial.printf("\nAPD DShot600 @ %u Hz; Telemetry on %s\n",
                  1000000u/PACKET_PERIOD_US, _tlmName);

    _tlm->begin(115200);  // default start
    armESC_();
    _cycleStartMs = millis();

    Serial.printf("Anchors: US_A=%u @ RPM_A=%.0f, US_B=%u @ RPM_B=%.0f (slope=%.3f us/RPM)\n",
                  _US_A, _RPM_A, _US_B, _RPM_B, _mapSlope);
  }

  void tick(){
    telemetryScanTick_();

    uint32_t now_ms = millis();
    uint32_t msInto = now_ms - _cycleStartMs;
    float target_mechRPM = _seq.targetAt(msInto);

    // response profiler step detection
    uint32_t now_us = micros();
    if (fabsf(target_mechRPM - _lastTargetRPM) >= STEP_DETECT_THRESH_RPM) {
      _respStart(target_mechRPM, now_us);
    }
    _lastTargetRPM = target_mechRPM;

    telemetryStreamTick_();

    // control loop
    static uint32_t prev_us = 0;
    if (!prev_us) prev_us = now_us;
    float dt = (now_us - prev_us) * 1e-6f;
    prev_us = now_us;

    float rpm_raw = _rpmInst;

    // bootstrap EMA
    if (_rpmFilt == 0.0f && rpm_raw > 0.0f) _rpmFilt = rpm_raw;

    float dynGate = fmaxf(MIN_SPIKE_GATE_RPM,
                          SPIKE_GATE_FRAC * fmaxf(target_mechRPM, _rpmFilt > 1.0f ? _rpmFilt : target_mechRPM));
    float rpm = (fabsf(rpm_raw - _rpmFilt) > dynGate) ? _rpmFilt : rpm_raw;

    _rpmFilt = EMA_ALPHA * rpm + (1.0f - EMA_ALPHA) * _rpmFilt;

    unsigned ff_us = rpmToUs_(target_mechRPM);

    float err = target_mechRPM - _rpmFilt;
    float unsat_us = (float)ff_us + FF_OFFSET_US + KP_US_PER_RPM * err + _I_us;
    float sat_us   = constrain(unsat_us, (float)MIN_US, (float)MAX_US);

    bool pushing_up   = (unsat_us > sat_us) && (err > 0.0f);
    bool pushing_down = (unsat_us < sat_us) && (err < 0.0f);
    if (!(pushing_up || pushing_down)) {
      _I_us += (KI_US_PER_RPM_PER_S * err) * dt;
    }
    _I_us += KAW * (sat_us - unsat_us);

    static float prev_err = 0.0f;
    if ((err > 0.0f) != (prev_err > 0.0f)) _I_us *= 0.5f;
    prev_err = err;

    _I_us = constrain(_I_us, -I_CLAMP_US, I_CLAMP_US);

    int desiredUs = (int)sat_us;
    unsigned maxStep = stepPerPacket_(_maxSlew_us_per_s);
    if (_currentUs + maxStep < (unsigned)desiredUs)      _currentUs += maxStep;
    else if (_currentUs > (unsigned)desiredUs + maxStep) _currentUs -= maxStep;
    else                                                 _currentUs  = desiredUs;

    // send DShot (request telemetry)
    uint16_t throttle = usToDShotThrottle_(_currentUs);
    uint16_t packet   = createDShotPacket_(throttle, true);
    sendDShotPacket_(packet);

    // status print
    if (now_ms - _lastPrintMs >= PRINT_PERIOD_MS){
      _lastPrintMs = now_ms;
      double n = (double)_accum.n;
      double vAvg   = (n>0) ? (_accum.sumV   / n) : NAN;
      double aAvg   = (n>0) ? (_accum.sumA   / n) : NAN;
      double tAvg   = (n>0) ? (_accum.sumT   / n) : NAN;
      double eAvg   = (n>0) ? (_accum.sumERPM/ n) : NAN;
      double mrpmAvg= (n>0) ? (_accum.sumMRPM/ n) : NAN;
      uint32_t mAhW = _accum.summAh;

      Serial.printf(
        "t=%lus  targetRPM=%.0f  us=%u->thr=%u  |  V=%.2f  A=%.2f  Temp=%.1fC  eRPM=%.0f  mechRPM=%.0f  mAh=%lu  n=%lu\n",
        (unsigned)((msInto % (_seq.cycleMs() ? _seq.cycleMs() : 1))/1000),
        _lastTargetRPM, _currentUs, throttle,
        isnan(vAvg)?-1.0:vAvg,
        isnan(aAvg)?-1.0:aAvg,
        isnan(tAvg)?-1.0:tAvg,
        isnan(eAvg)?-1.0:eAvg,
        isnan(mrpmAvg)?-1.0:mrpmAvg,
        (unsigned long)mAhW,
        (unsigned long)_accum.n
      );

      _accum.reset();
    }

    delayMicroseconds(PACKET_PERIOD_US);

    // loop pause between repeats
    if (_seq.loop() && _seq.cycleMs() > 0 && (millis() - _cycleStartMs) >= _seq.cycleMs()){
      _cycleStartMs = millis();
      sendForMs_(0, 3000);
    }
  }

private:
  // ===== support structs =====
  struct TelemetrySample {
    float volts = NAN, amps = NAN, tempC = NAN, eRPM = NAN, mechRPM = NAN;
    uint16_t mAh = 0;
  };
  struct TelemetryAccum {
    double sumV=0, sumA=0, sumT=0, sumERPM=0, sumMRPM=0;
    uint32_t summAh=0; uint32_t n=0;
    void add(const TelemetrySample& s){
      if (!isnan(s.volts))   sumV   += s.volts;
      if (!isnan(s.amps))    sumA   += s.amps;
      if (!isnan(s.tempC))   sumT   += s.tempC;
      if (!isnan(s.eRPM))    sumERPM+= s.eRPM;
      if (!isnan(s.mechRPM)) sumMRPM+= s.mechRPM;
      summAh += s.mAh; n++;
    }
    void reset(){ sumV=sumA=sumT=sumERPM=sumMRPM=0; summAh=0; n=0; }
  };

  // ===== response-time profiler (per instance) =====
  void _respStart(float toRPM, uint32_t now_us){
    _resp.active=true; _resp.riseMarked=false; _resp.settleMarked=false;
    _resp.fromRPM=_lastTargetRPM; _resp.toRPM=toRPM; _resp.stepAmp=toRPM - _lastTargetRPM;
    _resp.t0_us=now_us; _resp.t_rise_90_us=0; _resp.t_settle_us=0;
    _resp.peakMax=-1e9f; _resp.peakMin=+1e9f; _resp.inBandStart_us=0;
  }
  void _respUpdate(float rpm, uint32_t now_us){
    if (!_resp.active) return;
    if (rpm > _resp.peakMax) _resp.peakMax = rpm;
    if (rpm < _resp.peakMin) _resp.peakMin = rpm;

    bool up = _resp.stepAmp > 0.0f;
    float hi = _resp.fromRPM + RISE_HIGH_FRAC * _resp.stepAmp;
    if (!_resp.riseMarked){
      if ((up && rpm >= hi) || (!up && rpm <= hi)){
        _resp.t_rise_90_us = now_us - _resp.t0_us;
        _resp.riseMarked = true;
      }
    }
    float bandLo = _resp.toRPM * (1.0f - BAND_PCT);
    float bandHi = _resp.toRPM * (1.0f + BAND_PCT);
    bool inBand = (rpm >= bandLo && rpm <= bandHi);
    if (inBand){
      if (!_resp.inBandStart_us) _resp.inBandStart_us = now_us;
      uint32_t dwell = now_us - _resp.inBandStart_us;
      if (!_resp.settleMarked && dwell >= SETTLE_HOLD_US){
        _resp.t_settle_us = now_us - _resp.t0_us;
        _resp.settleMarked = true;
        float amp = fabsf(_resp.stepAmp) > 1.0f ? fabsf(_resp.stepAmp) : 1.0f;
        float overshoot_pct = 0.0f;
        if (up) { float over = _resp.peakMax - _resp.toRPM; if (over > 0) overshoot_pct = 100.0f * over / amp; }
        else   { float under= _resp.toRPM - _resp.peakMin; if (under> 0) overshoot_pct = 100.0f * under/ amp; }
        Serial.printf("RESP: step %.0f->%.0f | rise90=%lu us | settle±%.1f%%=%lu us | overshoot=%.2f%%\n",
                      _resp.fromRPM, _resp.toRPM,
                      (unsigned long)_resp.t_rise_90_us,
                      BAND_PCT*100.0f, (unsigned long)_resp.t_settle_us,
                      overshoot_pct);
        _resp.active=false;
      }
    } else {
      _resp.inBandStart_us = 0;
    }
  }
  struct Resp {
    bool active=false, riseMarked=false, settleMarked=false;
    float fromRPM=0, toRPM=0, stepAmp=0;
    uint32_t t0_us=0, t_rise_90_us=0, t_settle_us=0;
    float peakMax=-1e9f, peakMin=+1e9f;
    uint32_t inBandStart_us=0;
  } _resp;

  // ===== helpers =====
  static uint8_t crc8_0x07_(const uint8_t* d, int n) {
    uint8_t c = 0;
    for (int i = 0; i < n; i++) {
      c ^= d[i];
      for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
    }
    return c;
  }

  unsigned stepPerPacket_(unsigned slew_us_per_s){
    unsigned s = (unsigned)(( (uint64_t)slew_us_per_s * PACKET_PERIOD_US ) / 1000000ULL);
    return s ? s : 1;
  }

  unsigned rpmToUs_(float rpm){
    if (!(_RPM_B > _RPM_A)) return MIN_US;
    float usf = _US_A + _mapSlope * (rpm - _RPM_A);
    const float RPM_SPIN_MARGIN = 100.0f;
    if (rpm >= (_RPM_A - RPM_SPIN_MARGIN) && usf < (float)_US_A) usf = (float)_US_A;
    if (usf < (float)MIN_US) usf = (float)MIN_US;
    if (usf > (float)MAX_US) usf = (float)MAX_US;
    return (unsigned)(usf + 0.5f);
  }

  uint16_t usToDShotThrottle_(unsigned us){
    if (us < MIN_US) us = MIN_US;
    if (us > MAX_US) us = MAX_US;
    float norm = (float)(us - MIN_US) / (MAX_US - MIN_US);
    uint16_t thr = 48 + (uint16_t)(norm * (2047 - 48));
    if (thr > 2047) thr = 2047;
    return thr;
  }

  uint16_t createDShotPacket_(uint16_t throttle, bool telemetryBit){
    throttle &= 0x07FF;
    uint16_t packet = (throttle << 1) | (telemetryBit ? 1 : 0);
    uint8_t  crc = 0; uint16_t t = packet;
    for (int i = 0; i < 3; i++) { crc ^= t & 0xF; t >>= 4; }
    crc &= 0xF;
    return (packet << 4) | crc;
  }

  void sendDShotPacket_(uint16_t packet){
    noInterrupts();
    for (int i = 15; i >= 0; --i) {
      bool bit = (packet >> i) & 0x01;
      digitalWriteFast(ESC_PIN, HIGH);
      if (bit) delayNanoseconds(DSHOT600_ONE_HIGH_NS);
      else     delayNanoseconds(DSHOT600_ZERO_HIGH_NS);
      digitalWriteFast(ESC_PIN, LOW);
      delayNanoseconds(DSHOT600_BIT_TIME_NS - (bit ? DSHOT600_ONE_HIGH_NS : DSHOT600_ZERO_HIGH_NS));
    }
    interrupts();
  }

  void sendForMs_(uint16_t throttle, uint32_t ms){
    uint32_t t0 = millis();
    uint16_t pkt = createDShotPacket_(throttle, true);
    while (millis() - t0 < ms){
      sendDShotPacket_(pkt);
      delayMicroseconds(PACKET_PERIOD_US);
    }
  }

  void armESC_(){
    sendForMs_(0, ARM_ZERO_MS);
    sendForMs_(ARM_IDLE_THR, ARM_IDLE_MS);
  }

  void telemetryStreamTick_() {
    while (_tlm->available()) {
      _tlmBuf[_tlmIdx++] = (uint8_t)_tlm->read();
      if (_tlmIdx < 10) continue;
      decodeTelemetryPacket_(_tlmBuf);
      memmove(_tlmBuf, _tlmBuf+1, 9);
      _tlmIdx = 9;
    }
  }

  void decodeTelemetryPacket_(const uint8_t f[10]) {
    if (crc8_0x07_(f, 9) != f[9]) return;

    int8_t   tempC = (int8_t)f[0];
    uint16_t vraw  = ((uint16_t)f[1]<<8) | f[2];
    uint16_t iraw  = ((uint16_t)f[3]<<8) | f[4];
    uint16_t mAh   = ((uint16_t)f[5]<<8) | f[6];
    uint16_t rpm16 = ((uint16_t)f[7]<<8) | f[8];

    TelemetrySample s;
    s.volts   = vraw / 100.0f;
    s.amps    = iraw / 100.0f;
    s.tempC   = (float)tempC;
    s.eRPM    = (float)rpm16 * 100.0f;
    s.mechRPM = (MOTOR_POLE_PAIRS>0) ? (s.eRPM / (float)MOTOR_POLE_PAIRS) : NAN;
    s.mAh     = mAh;

    _accum.add(s);
    _respUpdate(s.mechRPM, micros());
    _rpmInst = s.mechRPM;
  }

  void telemetryScanTick_() {
    if (!_scanBauds) return;
    uint32_t now = millis();
    if (now - _lastBaudSwitchMs >= 2000) {
      _lastBaudSwitchMs = now;
      _baudIdx = (_baudIdx + 1) % (_NBAUDS);
      _tlm->end(); _tlm->begin(_BAUDS[_baudIdx]);
      Serial.printf("TLM: trying %u baud on %s\n", _BAUDS[_baudIdx], _tlmName);
    }
  }

  // ===== per-instance state =====
  HardwareSerial* _tlm;
  const char* _tlmName;

  // anchors & map
  float _RPM_A, _RPM_B, _mapSlope;
  unsigned _US_A, _US_B;

  // timeline
  DemoSequence _seq;

  // telemetry
  static constexpr int _NBAUDS = 4;
  const unsigned _BAUDS[_NBAUDS] = {115200, 230400, 57600, 250000};
  bool _scanBauds = false;
  int _baudIdx = -1;
  uint8_t _tlmBuf[10] = {0};
  uint8_t _tlmIdx = 0;

  // accumulators / control
  TelemetryAccum _accum;
  volatile float _rpmInst = 0.0f;
  float _rpmFilt = 0.0f;
  float _I_us    = 0.0f;
  unsigned _maxSlew_us_per_s = 4000;
  unsigned _currentUs = MIN_US;

  // timing
  uint32_t _cycleStartMs = 0;
  uint32_t _lastPrintMs = 0;
  uint32_t _lastBaudSwitchMs = 0;

  // profiler
  float _lastTargetRPM = 0.0f;

  // motor config
  static constexpr int MOTOR_POLE_PAIRS = 14; // BrotherHobby Avenger 2806.5
};
