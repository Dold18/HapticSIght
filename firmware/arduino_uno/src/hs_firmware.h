// Hardware-independent firmware core: protocol handling, modes, watchdog,
// fallback, motor scheduling, and outgoing frame scheduling.
//
// The Arduino sketch (arduino_uno.ino) only supplies time, serial bytes,
// pins, and the shift register; everything here is unit-tested natively.
#ifndef HS_FIRMWARE_H
#define HS_FIRMWARE_H

#include <stdint.h>

#include "hs_config.h"
#include "hs_fallback.h"
#include "hs_patterns.h"
#include "hs_protocol.h"
#include "hs_ultrasonic.h"

namespace hs {

enum Mode { kModeBoot = 'B', kModeHost = 'H', kModeFallback = 'F' };

struct Counters {
  uint16_t rx_errors;     // rejected lines (saturating); reported in $ST
  uint32_t rx_hap;        // accepted HAP
  uint32_t rx_pat;        // accepted PAT
  uint32_t expiries;      // HOST -> FALLBACK transitions
  uint32_t tx_frames;
};

class Firmware {
 public:
  explicit Firmware(UltrasonicHal& hal) : ultrasonic_(hal) {}

  // Call once after reset. All motors OFF, mode BOOT, $HLO queued.
  void begin(uint32_t now_ms);
  void onRxByte(uint8_t c, uint32_t now_ms);
  void update(uint32_t now_ms, uint32_t now_us);
  // Motor states (bit k = Mk) for the shift register.
  uint8_t motorByte(uint32_t now_ms) const;

  // Outgoing frames: format the next pending frame into buf (>= 64 bytes) and
  // return its length (0 = nothing pending). Call frameSent() only once the
  // bytes were actually handed to the serial port, so nothing blocks.
  uint8_t pendingFrame(char* buf) const;
  void frameSent();

  Mode mode() const { return static_cast<Mode>(mode_); }
  const Counters& counters() const { return counters_; }
  const PatternTable& patterns() const { return patterns_; }
  const UltrasonicSequencer& ultrasonic() const { return ultrasonic_; }
  uint8_t slot(uint8_t m) const { return scheduler_.slot(m); }
  bool statusLed(uint32_t now_ms) const;

 private:
  enum Pending { kNone = 0, kHello = 1, kStatus = 2, kUltrasonic = 3 };
  void handleLine(const char* line, uint8_t len, uint32_t now_ms);
  void setMode(uint8_t m, uint32_t now_ms);
  void computeSlots(uint32_t now_ms, uint8_t slots[8]);
  void countError();
  uint8_t nextPending() const;

  UltrasonicSequencer ultrasonic_;
  PatternTable patterns_;
  ChannelScheduler scheduler_;
  FallbackPolicy fallback_;
  LineAssembler rx_;
  Counters counters_;
  HapMsg hap_;
  bool has_hap_;
  uint32_t hap_rx_ms_;
#if HS_BOOT_TO_FALLBACK_MS > 0
  uint32_t boot_ms_;
#endif
  uint32_t mode_since_ms_;
  uint32_t last_status_ms_;
  uint32_t status_t_ms_;
  uint32_t us_t_ms_;
  uint16_t us_seq_;
  uint16_t st_seq_;
  uint8_t mode_;
  bool hello_pending_;
  bool status_pending_;
  bool us_pending_;
  RangeField us_snapshot_[kSensors];
};

}  // namespace hs

#endif  // HS_FIRMWARE_H
