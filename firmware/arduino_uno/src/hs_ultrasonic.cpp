#include "hs_ultrasonic.h"

#include "hs_config.h"

namespace hs {

void UltrasonicSequencer::reset(uint32_t now_ms) {
  for (uint8_t i = 0; i < kSensors; ++i) {
    readings_[i].kind = kRangeNone;
    readings_[i].cm = 0;
    readings_[i].t_ms = 0;
    cause_[i] = kCauseNone;
  }
  state_ = kStartSlot;
  sensor_ = 0;
  slot_start_ms_ = now_ms;
  trig_us_ = 0;
  rise_us_ = 0;
  cycles_ = 0;
  started_ = false;
}

uint8_t UltrasonicSequencer::classify(uint32_t echo_us, uint16_t& cm_out) {
  cm_out = 0;
  if (echo_us > HS_US_MAX_ECHO_US) return kRangeNoEcho;
  uint32_t cm = (echo_us + HS_US_US_PER_CM / 2) / HS_US_US_PER_CM;
  if (cm < HS_US_MIN_CM) return kRangeError;  // implausible
  if (cm > HS_US_MAX_CM) return kRangeNoEcho;
  cm_out = static_cast<uint16_t>(cm);
  return kRangeValid;
}

void UltrasonicSequencer::finish(uint8_t kind, uint16_t cm, uint8_t cause, uint32_t now_ms) {
  hal_.disarm();
  readings_[sensor_].kind = kind;
  readings_[sensor_].cm = cm;
  readings_[sensor_].t_ms = now_ms;
  cause_[sensor_] = cause;
  state_ = kSlotDone;
}

bool UltrasonicSequencer::update(uint32_t now_ms, uint32_t now_us) {
  bool cycle_done = false;
  if (state_ == kSlotDone && now_ms - slot_start_ms_ >= HS_US_SLOT_MS) {
    sensor_ = static_cast<uint8_t>((sensor_ + 1) & 3);
    if (sensor_ == 0) {
      ++cycles_;
      cycle_done = true;
    }
    state_ = kStartSlot;
  }
  if (state_ == kStartSlot) {
    if (started_ && now_ms - slot_start_ms_ < HS_US_SLOT_MS) return cycle_done;
    started_ = true;
    slot_start_ms_ = now_ms;
    if (hal_.echoHigh(sensor_)) {
      // Echo line already high before triggering: stuck output or (with the
      // pull-up) a disconnected sensor. Never treat this as "clear".
      finish(kRangeError, 0, kCauseStuckHigh, now_ms);
      return cycle_done;
    }
    trig_us_ = now_us;
    hal_.fire(sensor_);
    state_ = kWaitRise;
  }
  uint32_t rise = 0, fall = 0;
  uint8_t flags = 0;
  if (state_ == kWaitRise || state_ == kWaitFall) flags = hal_.edges(rise, fall);
  if (state_ == kWaitRise) {
    if (flags & 1) {
      rise_us_ = rise;
      state_ = kWaitFall;
    } else if (now_us - trig_us_ > HS_US_RISE_TIMEOUT_US) {
      finish(kRangeError, 0, kCauseNoRise, now_ms);  // sensor missing / unpowered
    }
  }
  if (state_ == kWaitFall) {
    if (flags & 2) {
      uint16_t cm;
      uint8_t kind = classify(fall - rise_us_, cm);
      finish(kind, cm, kind == kRangeError ? kCauseTooClose : kCauseNone, now_ms);
    } else if (now_us - rise_us_ > HS_US_MAX_ECHO_US) {
      finish(kRangeNoEcho, 0, kCauseNone, now_ms);  // nothing within range
    }
  }
  return cycle_done;
}

}  // namespace hs
