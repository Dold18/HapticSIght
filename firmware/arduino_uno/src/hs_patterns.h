// Pulse patterns and the 8-channel pulse scheduler.
// Timing semantics are identical to hapticsight/haptics/patterns.py
// (PulsePattern.is_on) and scheduler.py; tests cross-check both.
#ifndef HS_PATTERNS_H
#define HS_PATTERNS_H

#include <stdint.h>

namespace hs {

struct Pattern {
  uint16_t on_ms;
  uint16_t off_ms;
  uint16_t pause_ms;
  uint8_t count;  // 0 = silent
};

enum Slot {
  kSlotOff = 0,
  kSlotCaution = 1,
  kSlotWarning = 2,
  kSlotCritical = 3,
  kSlotCautionApproach = 4,
  kSlotWarningApproach = 5,
  kSlotCriticalApproach = 6,
  kSlotDegraded = 7,
  kSlotAdvisory = 8  // experimental, optional
};

uint32_t patternCycleMs(const Pattern& p);
uint32_t patternBurstMs(const Pattern& p);
bool patternIsOn(const Pattern& p, uint32_t elapsed_ms);

// 16 slots. Slot 0 is permanently OFF; slots 1-8 have built-in defaults;
// 9-15 are undefined (play as OFF) until defined with $PAT.
class PatternTable {
 public:
  static const uint8_t kSlots = 16;
  PatternTable() { loadDefaults(); }
  void loadDefaults();
  bool define(uint8_t slot, const Pattern& p);
  bool isDefined(uint8_t slot) const;
  // Pattern for a slot; undefined slots and slot 0 return a silent pattern.
  const Pattern& get(uint8_t slot) const;
  uint8_t revision(uint8_t slot) const { return slot < kSlots ? rev_[slot] : 0; }

 private:
  Pattern p_[kSlots];
  uint8_t rev_[kSlots];
  uint16_t defined_;
};

// Eight independent channels. Re-applying the same slot keeps its phase;
// a different slot (or a redefined slot) restarts the channel's timing.
class ChannelScheduler {
 public:
  ChannelScheduler() { reset(); }
  void reset();
  void setSlots(const uint8_t slots[8], uint32_t now_ms, const PatternTable& table);
  // bit k = motor Mk; rollover-safe elapsed-time arithmetic.
  uint8_t outputByte(uint32_t now_ms, const PatternTable& table) const;
  uint8_t slot(uint8_t m) const { return slot_[m & 7]; }

 private:
  uint8_t slot_[8];
  uint8_t rev_[8];
  uint32_t start_[8];
};

}  // namespace hs

#endif  // HS_PATTERNS_H
