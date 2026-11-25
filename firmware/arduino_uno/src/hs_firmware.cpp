#include "hs_firmware.h"

#include "hs_config.h"

namespace hs {

void Firmware::begin(uint32_t now_ms) {
  patterns_.loadDefaults();
  scheduler_.reset();
  fallback_.reset();
  rx_.reset();
  ultrasonic_.reset(now_ms);
  counters_.rx_errors = 0;
  counters_.rx_hap = 0;
  counters_.rx_pat = 0;
  counters_.expiries = 0;
  counters_.tx_frames = 0;
  has_hap_ = false;
  hap_rx_ms_ = 0;
  hap_.seq = 0;
  hap_.ttl_ms = 0;
  for (uint8_t m = 0; m < kChannels; ++m) hap_.slots[m] = kSlotOff;
#if HS_BOOT_TO_FALLBACK_MS > 0
  boot_ms_ = now_ms;
#endif
  mode_ = kModeBoot;
  mode_since_ms_ = now_ms;
  last_status_ms_ = now_ms;
  status_t_ms_ = now_ms;
  us_t_ms_ = 0;
  us_seq_ = 0;
  st_seq_ = 0;
  hello_pending_ = true;
  status_pending_ = true;
  us_pending_ = false;
  for (uint8_t i = 0; i < kSensors; ++i) {
    us_snapshot_[i].kind = kRangeNone;
    us_snapshot_[i].cm = 0;
  }
}

void Firmware::countError() {
  if (counters_.rx_errors < 0xFFFF) ++counters_.rx_errors;
}

void Firmware::onRxByte(uint8_t c, uint32_t now_ms) {
  LineAssembler::Result r = rx_.push(c);
  if (r == LineAssembler::kOverflow) {
    countError();
  } else if (r == LineAssembler::kLine) {
    handleLine(rx_.line(), rx_.length(), now_ms);
  }
}

void Firmware::handleLine(const char* line, uint8_t len, uint32_t now_ms) {
  HostMessage msg;
  ParseError e = parseHostLine(line, len, msg);
  if (e == kEmpty) return;
  if (e != kOk) {
    countError();  // malformed: discard whole line, keep previous state
    return;
  }
  if (msg.type == kMsgHap) {
    hap_ = msg.hap;  // committed only after the entire line validated
    has_hap_ = true;
    hap_rx_ms_ = now_ms;
    ++counters_.rx_hap;
    if (mode_ != kModeHost) setMode(kModeHost, now_ms);
  } else if (msg.type == kMsgPat) {
    Pattern p;
    p.on_ms = msg.pat.on_ms;
    p.off_ms = msg.pat.off_ms;
    p.count = msg.pat.count;
    p.pause_ms = msg.pat.pause_ms;
    if (patterns_.define(msg.pat.slot, p)) {
      ++counters_.rx_pat;
    } else {
      countError();
    }
  }
}

void Firmware::setMode(uint8_t m, uint32_t now_ms) {
  if (m == kModeFallback && mode_ == kModeHost) ++counters_.expiries;
  mode_ = m;
  mode_since_ms_ = now_ms;
  status_pending_ = true;  // announce mode changes immediately
  status_t_ms_ = now_ms;
  if (m == kModeFallback) fallback_.reset();
}

void Firmware::computeSlots(uint32_t now_ms, uint8_t slots[8]) {
  for (uint8_t m = 0; m < 8; ++m) slots[m] = kSlotOff;
  if (mode_ == kModeHost) {
    for (uint8_t m = 0; m < 8; ++m) slots[m] = hap_.slots[m];
    return;
  }
  if (mode_ != kModeFallback) return;  // BOOT: everything off
  // Mode-change cue: one DEGRADED burst, then local ultrasonic warnings.
  uint32_t cue_ms = patternBurstMs(patterns_.get(kSlotDegraded));
  if (now_ms - mode_since_ms_ < cue_ms) {
    for (uint8_t m = 0; m < 8; ++m) {
      if (HS_FB_CUE_ALL_MOTORS || (m % 2) == 0) slots[m] = kSlotDegraded;
    }
    return;
  }
  Reading r[kSensors];
  for (uint8_t s = 0; s < kSensors; ++s) r[s] = ultrasonic_.reading(s);
  fallback_.compute(r, now_ms, slots);
}

void Firmware::update(uint32_t now_ms, uint32_t now_us) {
  if (ultrasonic_.update(now_ms, now_us)) {
    for (uint8_t s = 0; s < kSensors; ++s) {
      const Reading& r = ultrasonic_.reading(s);
      us_snapshot_[s].kind = r.kind;
      us_snapshot_[s].cm = r.cm;
    }
    us_t_ms_ = now_ms;
    us_pending_ = true;  // a newer cycle replaces an unsent one (latest wins)
  }
  // Watchdog: the latest valid command expires after its TTL.
  if (mode_ == kModeHost && now_ms - hap_rx_ms_ >= hap_.ttl_ms) {
    setMode(kModeFallback, now_ms);
  }
#if HS_BOOT_TO_FALLBACK_MS > 0
  if (mode_ == kModeBoot && now_ms - boot_ms_ >= HS_BOOT_TO_FALLBACK_MS) {
    setMode(kModeFallback, now_ms);
  }
#endif
  uint8_t slots[8];
  computeSlots(now_ms, slots);
  scheduler_.setSlots(slots, now_ms, patterns_);
  if (now_ms - last_status_ms_ >= HS_ST_PERIOD_MS) {
    last_status_ms_ = now_ms;
    status_t_ms_ = now_ms;
    status_pending_ = true;
  }
}

uint8_t Firmware::motorByte(uint32_t now_ms) const {
  if (mode_ == kModeBoot) return 0;
  return scheduler_.outputByte(now_ms, patterns_);
}

uint8_t Firmware::nextPending() const {
  if (hello_pending_) return kHello;
  if (status_pending_) return kStatus;
  if (us_pending_) return kUltrasonic;
  return kNone;
}

uint8_t Firmware::pendingFrame(char* buf) const {
  switch (nextPending()) {
    case kHello:
      return formatHello(buf);
    case kStatus:
      return formatStatus(buf, st_seq_, status_t_ms_, static_cast<char>(mode_), has_hap_,
                          hap_.seq, counters_.rx_errors);
    case kUltrasonic:
      return formatUltrasonic(buf, us_seq_, us_t_ms_, us_snapshot_);
    default:
      return 0;
  }
}

void Firmware::frameSent() {
  switch (nextPending()) {
    case kHello:
      hello_pending_ = false;
      break;
    case kStatus:
      status_pending_ = false;
      ++st_seq_;
      break;
    case kUltrasonic:
      us_pending_ = false;
      ++us_seq_;
      break;
    default:
      return;
  }
  ++counters_.tx_frames;
}

bool Firmware::statusLed(uint32_t now_ms) const {
  if (mode_ == kModeHost) return true;                  // solid
  if (mode_ == kModeFallback) return (now_ms / 125) & 1;  // 4 Hz blink
  return (now_ms / 500) & 1;                             // BOOT: 1 Hz blink
}

}  // namespace hs
