// Non-blocking, staggered HC-SR04 acquisition.
//
// Sensors are fired one at a time in fixed HS_US_SLOT_MS slots (S0, S1, S2,
// S3, repeat). Echo edges are captured with microsecond timestamps by a
// pin-change interrupt in the hardware layer (see UltrasonicHal); this class
// only runs a bounded state machine from loop(), so a missing echo never
// blocks serial handling or motor timing.
#ifndef HS_ULTRASONIC_H
#define HS_ULTRASONIC_H

#include <stdint.h>

#include "hs_protocol.h"

namespace hs {

// Hardware abstraction (implemented with pins + PCINT on the Uno, faked in tests).
class UltrasonicHal {
 public:
  virtual bool echoHigh(uint8_t sensor) = 0;
  // Arm edge capture for `sensor`, then emit the 10 us trigger pulse.
  virtual void fire(uint8_t sensor) = 0;
  // Atomically read captured edges: bit0 = rise seen, bit1 = fall seen.
  virtual uint8_t edges(uint32_t& rise_us, uint32_t& fall_us) = 0;
  virtual void disarm() = 0;

 protected:
  ~UltrasonicHal() {}
};

struct Reading {
  uint8_t kind;   // RangeKind
  uint16_t cm;    // valid only for kRangeValid
  uint32_t t_ms;  // millis() when the measurement completed
};

// Why a measurement was reported as an error (diagnostics only).
enum ErrorCause { kCauseNone = 0, kCauseStuckHigh = 1, kCauseNoRise = 2, kCauseTooClose = 3 };

class UltrasonicSequencer {
 public:
  explicit UltrasonicSequencer(UltrasonicHal& hal) : hal_(hal) { reset(0); }
  void reset(uint32_t now_ms);
  // Run the state machine. Returns true when a full S0..S3 cycle completed.
  bool update(uint32_t now_ms, uint32_t now_us);
  const Reading& reading(uint8_t s) const { return readings_[s & 3]; }
  uint8_t lastErrorCause(uint8_t s) const { return cause_[s & 3]; }
  uint32_t cycles() const { return cycles_; }
  uint8_t activeSensor() const { return sensor_; }

  static uint8_t classify(uint32_t echo_us, uint16_t& cm_out);

 private:
  enum State { kStartSlot = 0, kWaitRise = 1, kWaitFall = 2, kSlotDone = 3 };
  void finish(uint8_t kind, uint16_t cm, uint8_t cause, uint32_t now_ms);

  UltrasonicHal& hal_;
  Reading readings_[kSensors];
  uint8_t cause_[kSensors];
  uint8_t state_;
  uint8_t sensor_;
  uint32_t slot_start_ms_;
  uint32_t trig_us_;
  uint32_t rise_us_;
  uint32_t cycles_;
  bool started_;
};

}  // namespace hs

#endif  // HS_ULTRASONIC_H
