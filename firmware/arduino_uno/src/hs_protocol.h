// HapticSight serial protocol v1: framing, parsing, and formatting.
// Authoritative specification: docs/PROTOCOL.md. Acceptance rules mirror the
// Python codec (hapticsight/protocol/codec.py) and are cross-checked by
// tests/test_firmware_native.py.
//
// No dynamic memory, no Arduino String: fixed buffers only.
#ifndef HS_PROTOCOL_H
#define HS_PROTOCOL_H

#include <stdint.h>

namespace hs {

const uint8_t kMaxLineBytes = 64;  // including the terminating '\n'
const uint8_t kMaxLineChars = kMaxLineBytes - 1;
const uint8_t kChannels = 8;
const uint8_t kSensors = 4;

uint8_t checksum(const char* s, uint8_t n);

// Collects bytes into lines. Lines longer than 63 characters are discarded up to
// the next '\n' and reported once as kOverflow (resynchronisation).
class LineAssembler {
 public:
  enum Result { kNone = 0, kLine = 1, kOverflow = 2 };
  LineAssembler() { reset(); }
  void reset();
  Result push(uint8_t c);
  const char* line() const { return buf_; }
  uint8_t length() const { return len_; }

 private:
  char buf_[kMaxLineBytes + 1];
  uint8_t len_;
  bool discarding_;
  bool ready_;
};

enum ParseError {
  kOk = 0,
  kEmpty = 1,          // blank line: ignored, not an error
  kFraming = 2,        // missing '$', bad '*CS' suffix, non-printable, too long
  kChecksum = 3,
  kUnknownType = 4,
  kUnexpectedType = 5,  // valid v1 type, but not one the host may send (HLO/US/ST)
  kFieldCount = 6,
  kBadNumber = 7,
  kOutOfRange = 8,
  kBadPatterns = 9
};

enum MsgType { kMsgNone = 0, kMsgHap = 1, kMsgPat = 2 };

struct HapMsg {
  uint16_t seq;
  uint16_t ttl_ms;
  uint8_t slots[kChannels];  // M0..M7, each 0..15
};

struct PatMsg {
  uint8_t slot;  // 1..15 (0 is permanently OFF)
  uint16_t on_ms;
  uint16_t off_ms;
  uint8_t count;  // 0..9
  uint16_t pause_ms;
};

struct HostMessage {
  MsgType type;
  HapMsg hap;
  PatMsg pat;
};

// Parse one line (without '\n'; a trailing '\r' is tolerated). On any error
// `out` must be ignored; nothing is partially applied.
ParseError parseHostLine(const char* line, uint8_t len, HostMessage& out);

// ---------------------------------------------------------------- formatting
enum RangeKind { kRangeNone = 0, kRangeValid = 1, kRangeNoEcho = 2, kRangeError = 3 };

struct RangeField {
  uint8_t kind;  // RangeKind; kRangeNone is reported as 'E' (no measurement)
  uint16_t cm;
};

// Each returns the number of bytes written (including '\n'), buffer >= 64 bytes.
uint8_t formatHello(char* buf);
uint8_t formatUltrasonic(char* buf, uint16_t seq, uint32_t t_ms, const RangeField r[kSensors]);
uint8_t formatStatus(char* buf, uint16_t seq, uint32_t t_ms, char mode, bool has_hap,
                     uint16_t last_hap_seq, uint16_t rx_errors);

}  // namespace hs

#endif  // HS_PROTOCOL_H
