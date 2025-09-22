/**
 * File:    dshot_MotorControl.h
 * Author:  Matthew Evangelista
 * Date:    2025-09-22
 *
 * Teensy 4.0 control of APD 40F3 ESC using DShot600 @1 kHz with UART telemetry.
 * Provides a header-only MotorController with feed-forward + PI loop, slew limit,
 * EMA filtering, and step-response profiling. Includes DemoSequence for building
 * ramp/hold test timelines with a ready-to-use 15-step default sequence.
 *
 * License: Evangelista’s Electric Non-Commercial Open Source License (EENOSL v1.0).
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
