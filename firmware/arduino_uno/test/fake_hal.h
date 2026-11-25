// Simulated HC-SR04 sensors for native tests and the virtual Uno (fw_sim).
// Timing model: echo rises 450 us after the trigger and stays high for
// 58 us/cm (distance) or ~38 ms (nothing in range), like a real module.
#ifndef HS_TEST_FAKE_HAL_H
#define HS_TEST_FAKE_HAL_H

#include <stdint.h>

#include "../src/hs_ultrasonic.h"

namespace hs_test {

enum SensorBehaviour { kEcho = 0, kNoEcho = 1, kNoRise = 2, kStuckHigh = 3 };

struct FakeSensor {
  uint8_t behaviour;
  uint16_t cm;
};

class FakeHal : public hs::UltrasonicHal {
 public:
  FakeHal() : now_us(0), armed(-1), fired_at(0), fire_count(0) {
    for (int i = 0; i < 4; ++i) {
      sensors[i].behaviour = kNoEcho;
      sensors[i].cm = 0;
      fires[i] = 0;
    }
  }
  void set(uint8_t s, uint8_t behaviour, uint16_t cm = 0) {
    sensors[s & 3].behaviour = behaviour;
    sensors[s & 3].cm = cm;
  }
  bool echoHigh(uint8_t s) { return sensors[s & 3].behaviour == kStuckHigh; }
  void fire(uint8_t s) {
    armed = s & 3;
    fired_at = now_us;
    ++fire_count;
    ++fires[s & 3];
  }
  uint8_t edges(uint32_t& rise, uint32_t& fall) {
    if (armed < 0) return 0;
    const FakeSensor& f = sensors[armed];
    if (f.behaviour == kNoRise || f.behaviour == kStuckHigh) return 0;
    uint32_t r = fired_at + 450;
    uint32_t width = f.behaviour == kEcho ? static_cast<uint32_t>(f.cm) * 58u : 38000u;
    uint8_t flags = 0;
    if (now_us - fired_at >= 450) {
      flags |= 1;
      rise = r;
    }
    if (now_us - fired_at >= 450 + width) {
      flags |= 2;
      fall = r + width;
    }
    return flags;
  }
  void disarm() { armed = -1; }

  FakeSensor sensors[4];
  uint32_t now_us;
  int armed;
  uint32_t fired_at;
  uint32_t fire_count;
  uint32_t fires[4];
};

}  // namespace hs_test

#endif  // HS_TEST_FAKE_HAL_H
