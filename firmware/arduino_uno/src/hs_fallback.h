// Ultrasonic-only local warnings used when host commands expire (FALLBACK mode).
#ifndef HS_FALLBACK_H
#define HS_FALLBACK_H

#include <stdint.h>

#include "hs_ultrasonic.h"

namespace hs {

// S0 -> M0, S1 -> M2, S2 -> M4, S3 -> M6. Diagonal motors stay off: there is
// no sensor coverage there, and the wearer has been told the system is degraded.
class FallbackPolicy {
 public:
  FallbackPolicy() { reset(); }
  void reset();
  // Fills slots[8]. Valid, fresh range -> CAUTION/WARNING/CRITICAL by band;
  // fresh no-echo -> OFF; error, missing, or stale reading -> DEGRADED.
  void compute(const Reading readings[4], uint32_t now_ms, uint8_t slots[8]);
  static uint8_t rankFor(const Reading& r, uint32_t now_ms);

 private:
  // ranks: 0 OFF, 1 DEGRADED, 2 CAUTION, 3 WARNING, 4 CRITICAL
  uint32_t last_[4][5];
  uint8_t seen_[4];  // bitmask of ranks ever requested per sensor
};

}  // namespace hs

#endif  // HS_FALLBACK_H
