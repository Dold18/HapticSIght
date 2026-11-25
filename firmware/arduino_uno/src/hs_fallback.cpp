#include "hs_fallback.h"

#include "hs_config.h"
#include "hs_patterns.h"

namespace hs {

namespace {
const uint8_t kRankToSlot[5] = {kSlotOff, kSlotDegraded, kSlotCaution, kSlotWarning,
                                kSlotCritical};
}

void FallbackPolicy::reset() {
  for (uint8_t s = 0; s < 4; ++s) {
    seen_[s] = 0;
    for (uint8_t r = 0; r < 5; ++r) last_[s][r] = 0;
  }
}

uint8_t FallbackPolicy::rankFor(const Reading& r, uint32_t now_ms) {
  if (r.kind == kRangeNone || now_ms - r.t_ms > HS_FB_STALE_MS) return 1;  // missing/stale
  if (r.kind == kRangeError) return 1;
  if (r.kind == kRangeNoEcho) return 0;  // nothing in this cone (this cone only)
  if (r.cm < HS_FB_CRITICAL_CM) return 4;
  if (r.cm < HS_FB_WARNING_CM) return 3;
  if (r.cm <= HS_FB_CAUTION_CM) return 2;
  return 0;
}

void FallbackPolicy::compute(const Reading readings[4], uint32_t now_ms, uint8_t slots[8]) {
  for (uint8_t m = 0; m < 8; ++m) slots[m] = kSlotOff;
  for (uint8_t s = 0; s < 4; ++s) {
    uint8_t req = rankFor(readings[s], now_ms);
    if (req > 0) {
      last_[s][req] = now_ms;
      seen_[s] |= static_cast<uint8_t>(1u << req);
    }
    uint8_t eff = req;
    for (uint8_t r = 4; r > req; --r) {  // release hysteresis: hold higher ranks briefly
      if ((seen_[s] & (1u << r)) && now_ms - last_[s][r] <= HS_FB_RELEASE_HOLD_MS) {
        eff = r;
        break;
      }
    }
    slots[s * 2] = kRankToSlot[eff];
  }
}

}  // namespace hs
