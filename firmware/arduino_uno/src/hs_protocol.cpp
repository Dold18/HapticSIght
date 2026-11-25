#include "hs_protocol.h"

#include <string.h>

#include "hs_config.h"

namespace hs {

uint8_t checksum(const char* s, uint8_t n) {
  uint8_t v = 0;
  for (uint8_t i = 0; i < n; ++i) v ^= static_cast<uint8_t>(s[i]);
  return v;
}

// ---------------------------------------------------------------- LineAssembler

void LineAssembler::reset() {
  len_ = 0;
  discarding_ = false;
  ready_ = false;
  buf_[0] = '\0';
}

LineAssembler::Result LineAssembler::push(uint8_t c) {
  if (ready_) {  // previous line consumed by the caller; start a new one
    len_ = 0;
    ready_ = false;
  }
  if (c == '\n') {
    if (discarding_) {
      discarding_ = false;
      len_ = 0;
      return kNone;
    }
    buf_[len_] = '\0';
    ready_ = true;
    return kLine;
  }
  if (discarding_) return kNone;
  buf_[len_++] = static_cast<char>(c);
  if (len_ >= kMaxLineBytes) {  // 64 bytes without '\n': same rule as the host decoder
    len_ = 0;
    discarding_ = true;
    return kOverflow;
  }
  return kNone;
}

// ---------------------------------------------------------------- parsing helpers

namespace {

int hexValue(char c, bool allow_lower) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (allow_lower && c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// Unsigned decimal, 1..10 digits, value <= max (mirrors the host regex ^[0-9]{1,10}$).
ParseError parseUint(const char* s, uint8_t n, uint32_t max, uint32_t& out) {
  if (n == 0 || n > 10) return kBadNumber;
  uint32_t v = 0;
  for (uint8_t i = 0; i < n; ++i) {
    if (s[i] < '0' || s[i] > '9') return kBadNumber;
    uint32_t d = static_cast<uint32_t>(s[i] - '0');
    if (v > (0xFFFFFFFFUL - d) / 10UL) return kOutOfRange;  // would overflow uint32
    v = v * 10UL + d;
  }
  if (v > max) return kOutOfRange;
  out = v;
  return kOk;
}

struct Fields {
  const char* ptr[8];
  uint8_t len[8];
  uint8_t count;
};

// Split body (after the type) by ','. Returns false if there are more than 8 fields.
bool splitFields(const char* s, uint8_t n, Fields& f) {
  f.count = 0;
  uint8_t start = 0;
  for (uint8_t i = 0; i <= n; ++i) {
    if (i == n || s[i] == ',') {
      if (f.count >= 8) return false;
      f.ptr[f.count] = s + start;
      f.len[f.count] = static_cast<uint8_t>(i - start);
      ++f.count;
      start = static_cast<uint8_t>(i + 1);
    }
  }
  return true;
}

bool typeIs(const char* s, uint8_t n, const char* name) {
  uint8_t m = static_cast<uint8_t>(strlen(name));
  return n == m && memcmp(s, name, m) == 0;
}

}  // namespace

ParseError parseHostLine(const char* line, uint8_t len, HostMessage& out) {
  out.type = kMsgNone;
  // strip trailing CR/LF (the host decoder uses rstrip("\r\n"))
  while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n')) --len;
  if (len == 0) return kEmpty;
  bool blank = true;
  for (uint8_t i = 0; i < len; ++i) {
    char c = line[i];
    if (c != ' ' && c != '\t') blank = false;
  }
  if (blank) return kEmpty;
  if (static_cast<uint16_t>(len) + 1 > kMaxLineBytes) return kFraming;
  for (uint8_t i = 0; i < len; ++i) {
    uint8_t c = static_cast<uint8_t>(line[i]);
    if (c < 0x20 || c > 0x7E) return kFraming;
  }
  if (line[0] != '$') return kFraming;
  int star = -1;
  for (int i = len - 1; i >= 0; --i) {
    if (line[i] == '*') {
      star = i;
      break;
    }
  }
  if (star < 0 || len - star != 3) return kFraming;
  int hi = hexValue(line[star + 1], true);
  int lo = hexValue(line[star + 2], true);
  if (hi < 0 || lo < 0) return kFraming;
  const char* body = line + 1;
  uint8_t body_len = static_cast<uint8_t>(star - 1);
  if (checksum(body, body_len) != static_cast<uint8_t>(hi * 16 + lo)) return kChecksum;

  // type = text up to the first ','
  uint8_t type_len = 0;
  while (type_len < body_len && body[type_len] != ',') ++type_len;
  const char* rest = body + type_len + (type_len < body_len ? 1 : 0);
  uint8_t rest_len = static_cast<uint8_t>(type_len < body_len ? body_len - type_len - 1 : 0);
  Fields f;
  bool has_args = type_len < body_len;

  if (typeIs(body, type_len, "HAP")) {
    if (!has_args || !splitFields(rest, rest_len, f) || f.count != 3) return kFieldCount;
    uint32_t seq, ttl;
    ParseError e = parseUint(f.ptr[0], f.len[0], 65535UL, seq);
    if (e != kOk) return e;
    e = parseUint(f.ptr[1], f.len[1], HS_MAX_TTL_MS, ttl);
    if (e != kOk) return e;
    if (ttl < 1) return kOutOfRange;
    if (f.len[2] != kChannels) return kBadPatterns;
    HapMsg hap;
    for (uint8_t i = 0; i < kChannels; ++i) {
      int v = hexValue(f.ptr[2][i], false);  // uppercase only, as on the host
      if (v < 0) return kBadPatterns;
      hap.slots[i] = static_cast<uint8_t>(v);
    }
    hap.seq = static_cast<uint16_t>(seq);
    hap.ttl_ms = static_cast<uint16_t>(ttl);
    out.hap = hap;
    out.type = kMsgHap;
    return kOk;
  }
  if (typeIs(body, type_len, "PAT")) {
    if (!has_args || !splitFields(rest, rest_len, f) || f.count != 5) return kFieldCount;
    uint32_t v[5];
    const uint32_t max[5] = {15UL, 65535UL, 65535UL, 9UL, 65535UL};
    for (uint8_t i = 0; i < 5; ++i) {
      ParseError e = parseUint(f.ptr[i], f.len[i], max[i], v[i]);
      if (e != kOk) return e;
    }
    if (v[0] < 1) return kOutOfRange;              // slot 0 is permanently OFF
    if (v[3] > 0 && v[1] == 0) return kOutOfRange;  // pulses need on time
    PatMsg pat;
    pat.slot = static_cast<uint8_t>(v[0]);
    pat.on_ms = static_cast<uint16_t>(v[1]);
    pat.off_ms = static_cast<uint16_t>(v[2]);
    pat.count = static_cast<uint8_t>(v[3]);
    pat.pause_ms = static_cast<uint16_t>(v[4]);
    out.pat = pat;
    out.type = kMsgPat;
    return kOk;
  }
  if (typeIs(body, type_len, "HLO") || typeIs(body, type_len, "US") ||
      typeIs(body, type_len, "ST")) {
    return kUnexpectedType;
  }
  return kUnknownType;
}

// ---------------------------------------------------------------- formatting

namespace {

class Writer {
 public:
  explicit Writer(char* buf) : buf_(buf), n_(0) {}
  void ch(char c) {
    if (n_ < kMaxLineBytes) buf_[n_++] = c;
  }
  void str(const char* s) {
    while (*s) ch(*s++);
  }
  void num(uint32_t v) {
    char tmp[10];
    uint8_t i = 0;
    do {
      tmp[i++] = static_cast<char>('0' + v % 10UL);
      v /= 10UL;
    } while (v && i < sizeof(tmp));
    while (i) ch(tmp[--i]);
  }
  uint8_t finish() {
    static const char kHex[] = "0123456789ABCDEF";
    uint8_t cs = checksum(buf_ + 1, static_cast<uint8_t>(n_ - 1));
    ch('*');
    ch(kHex[cs >> 4]);
    ch(kHex[cs & 0x0F]);
    ch('\n');
    return n_;
  }

 private:
  char* buf_;
  uint8_t n_;
};

}  // namespace

uint8_t formatHello(char* buf) {
  Writer w(buf);
  w.str("$HLO,");
  w.num(HS_PROTOCOL_VERSION);
  w.ch(',');
  w.str(HS_FIRMWARE_VERSION);
  return w.finish();
}

uint8_t formatUltrasonic(char* buf, uint16_t seq, uint32_t t_ms, const RangeField r[kSensors]) {
  Writer w(buf);
  w.str("$US,");
  w.num(seq);
  w.ch(',');
  w.num(t_ms);
  for (uint8_t i = 0; i < kSensors; ++i) {
    w.ch(',');
    if (r[i].kind == kRangeValid) {
      w.num(r[i].cm > 999 ? 999 : r[i].cm);
    } else if (r[i].kind == kRangeNoEcho) {
      w.ch('X');
    } else {
      w.ch('E');  // error, or no measurement completed yet
    }
  }
  return w.finish();
}

uint8_t formatStatus(char* buf, uint16_t seq, uint32_t t_ms, char mode, bool has_hap,
                     uint16_t last_hap_seq, uint16_t rx_errors) {
  Writer w(buf);
  w.str("$ST,");
  w.num(seq);
  w.ch(',');
  w.num(t_ms);
  w.ch(',');
  w.ch(mode);
  w.ch(',');
  if (has_hap) {
    w.num(last_hap_seq);
  } else {
    w.ch('-');
  }
  w.ch(',');
  w.num(rx_errors);
  return w.finish();
}

}  // namespace hs
