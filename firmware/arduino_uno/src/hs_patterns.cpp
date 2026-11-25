#include "hs_patterns.h"

namespace hs {

namespace {
const Pattern kSilent = {0, 0, 0, 0};

// Must match DEFAULT_PATTERNS in hapticsight/haptics/patterns.py (tested).
// {on, off, pause, count}
const Pattern kDefaults[9] = {
    {0, 0, 0, 0},         // 0 OFF
    {100, 0, 900, 1},     // 1 CAUTION            1 Hz, 10 %
    {150, 0, 350, 1},     // 2 WARNING            2 Hz, 30 %
    {180, 0, 70, 1},      // 3 CRITICAL           4 Hz, 72 %
    {80, 80, 760, 2},     // 4 CAUTION_APPROACH   1 Hz double tap
    {100, 60, 240, 2},    // 5 WARNING_APPROACH   2 Hz double tap
    {100, 30, 20, 2},     // 6 CRITICAL_APPROACH  4 Hz double tap
    {50, 80, 1690, 3},    // 7 DEGRADED           triple blip / 2 s
    {40, 0, 2960, 1},     // 8 ADVISORY (experimental) 1 tick / 3 s
};
}  // namespace

uint32_t patternBurstMs(const Pattern& p) {
  if (p.count == 0) return 0;
  return static_cast<uint32_t>(p.count) * p.on_ms + static_cast<uint32_t>(p.count - 1) * p.off_ms;
}

uint32_t patternCycleMs(const Pattern& p) {
  if (p.count == 0) return 0;
  return patternBurstMs(p) + p.pause_ms;
}

bool patternIsOn(const Pattern& p, uint32_t elapsed_ms) {
  if (p.count == 0 || p.on_ms == 0) return false;
  uint32_t cycle = patternCycleMs(p);
  uint32_t phase = elapsed_ms % cycle;
  if (phase >= patternBurstMs(p)) return false;
  uint32_t period = static_cast<uint32_t>(p.on_ms) + p.off_ms;
  return (phase % period) < p.on_ms;
}

void PatternTable::loadDefaults() {
  defined_ = 0;
  for (uint8_t i = 0; i < kSlots; ++i) {
    p_[i] = kSilent;
    rev_[i] = 0;
  }
  for (uint8_t i = 1; i < 9; ++i) {
    p_[i] = kDefaults[i];
    defined_ |= static_cast<uint16_t>(1u << i);
  }
}

bool PatternTable::define(uint8_t slot, const Pattern& p) {
  if (slot == 0 || slot >= kSlots) return false;
  if (p.count > 9 || (p.count > 0 && p.on_ms == 0)) return false;
  p_[slot] = p;
  defined_ |= static_cast<uint16_t>(1u << slot);
  ++rev_[slot];
  return true;
}

bool PatternTable::isDefined(uint8_t slot) const {
  return slot > 0 && slot < kSlots && (defined_ & (1u << slot)) != 0;
}

const Pattern& PatternTable::get(uint8_t slot) const {
  return isDefined(slot) ? p_[slot] : kSilent;
}

// ---------------------------------------------------------------- scheduler

void ChannelScheduler::reset() {
  for (uint8_t m = 0; m < 8; ++m) {
    slot_[m] = kSlotOff;
    rev_[m] = 0;
    start_[m] = 0;
  }
}

void ChannelScheduler::setSlots(const uint8_t slots[8], uint32_t now_ms,
                                const PatternTable& table) {
  for (uint8_t m = 0; m < 8; ++m) {
    uint8_t s = slots[m] & 0x0F;
    uint8_t r = table.revision(s);
    if (s != slot_[m] || r != rev_[m]) {
      slot_[m] = s;
      rev_[m] = r;
      start_[m] = now_ms;
    }
  }
}

uint8_t ChannelScheduler::outputByte(uint32_t now_ms, const PatternTable& table) const {
  uint8_t out = 0;
  for (uint8_t m = 0; m < 8; ++m) {
    uint32_t elapsed = now_ms - start_[m];  // unsigned: correct across millis() rollover
    if (patternIsOn(table.get(slot_[m]), elapsed)) out |= static_cast<uint8_t>(1u << m);
  }
  return out;
}

}  // namespace hs
