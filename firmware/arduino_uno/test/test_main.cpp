// Native unit tests for the hardware-independent firmware logic.
// Build & run:  make -C firmware/arduino_uno test
// (also run automatically by tests/test_firmware_native.py under pytest)
#include <stdio.h>
#include <string.h>

#include "../src/hs_config.h"
#include "../src/hs_fallback.h"
#include "../src/hs_firmware.h"
#include "../src/hs_patterns.h"
#include "../src/hs_protocol.h"
#include "../src/hs_ultrasonic.h"
#include "fake_hal.h"

using namespace hs;
using hs_test::FakeHal;

static int g_failures = 0;
static int g_checks = 0;
static const char* g_test = "";

#define CHECK(cond)                                                          \
  do {                                                                       \
    ++g_checks;                                                              \
    if (!(cond)) {                                                           \
      ++g_failures;                                                          \
      printf("FAIL %s:%d [%s] %s\n", __FILE__, __LINE__, g_test, #cond);    \
    }                                                                        \
  } while (0)
#define CHECK_EQ(a, b)                                                                    \
  do {                                                                                    \
    ++g_checks;                                                                           \
    long _a = static_cast<long>(a), _b = static_cast<long>(b);                            \
    if (_a != _b) {                                                                       \
      ++g_failures;                                                                       \
      printf("FAIL %s:%d [%s] %s == %s (%ld != %ld)\n", __FILE__, __LINE__, g_test, #a, \
             #b, _a, _b);                                                                 \
    }                                                                                     \
  } while (0)
#define TEST(name) static void name()
#define RUN(name)  \
  do {             \
    g_test = #name; \
    name();        \
  } while (0)

// ---------------------------------------------------------------- helpers

static void frame(char* out, const char* body) {
  snprintf(out, 96, "$%s*%02X", body, checksum(body, static_cast<uint8_t>(strlen(body))));
}

static ParseError parse(const char* line, HostMessage& m) {
  return parseHostLine(line, static_cast<uint8_t>(strlen(line)), m);
}

struct Rig {
  FakeHal hal;
  Firmware fw;
  uint32_t ms;
  char tx[80][72];
  int ntx;
  Rig() : fw(hal), ms(0), ntx(0) {}
  void begin(uint32_t start = 0) {
    ms = start;
    hal.now_us = start * 1000u;
    fw.begin(ms);
    drain();
  }
  void send(const char* text) {
    for (const char* p = text; *p; ++p) fw.onRxByte(static_cast<uint8_t>(*p), ms);
    fw.onRxByte('\n', ms);
  }
  void sendBody(const char* body) {
    char buf[96];
    frame(buf, body);
    send(buf);
  }
  void drain() {
    char buf[72];
    uint8_t n;
    while ((n = fw.pendingFrame(buf)) > 0) {
      buf[n] = '\0';
      if (ntx < 80) strcpy(tx[ntx++], buf);
      fw.frameSent();
    }
  }
  void advance(uint32_t dt_ms) {
    for (uint32_t i = 0; i < dt_ms; ++i) {
      ++ms;
      hal.now_us += 1000u;
      fw.update(ms, hal.now_us);
      drain();
    }
  }
  int countPrefix(const char* prefix) {
    int n = 0;
    for (int i = 0; i < ntx; ++i) n += strncmp(tx[i], prefix, strlen(prefix)) == 0;
    return n;
  }
  const char* last(const char* prefix) {
    for (int i = ntx - 1; i >= 0; --i)
      if (strncmp(tx[i], prefix, strlen(prefix)) == 0) return tx[i];
    return "";
  }
};

// ---------------------------------------------------------------- protocol

TEST(test_checksum_and_pinned_examples) {
  // Pinned examples from docs/PROTOCOL.md and tests/test_protocol.py
  CHECK_EQ(checksum("HAP,42,500,03007001", 19), 0x43);
  HostMessage m;
  CHECK_EQ(parse("$HAP,42,500,03007001*43", m), kOk);
  CHECK_EQ(m.type, kMsgHap);
  CHECK_EQ(m.hap.seq, 42);
  CHECK_EQ(m.hap.ttl_ms, 500);
  const uint8_t expect[8] = {0, 3, 0, 0, 7, 0, 0, 1};
  for (int i = 0; i < 8; ++i) CHECK_EQ(m.hap.slots[i], expect[i]);
  CHECK_EQ(parse("$PAT,2,150,0,1,350*58", m), kOk);
  CHECK_EQ(m.type, kMsgPat);
  CHECK_EQ(m.pat.slot, 2);
  CHECK_EQ(m.pat.on_ms, 150);
  CHECK_EQ(m.pat.pause_ms, 350);
  CHECK_EQ(parse("$HAP,42,500,03007001*43\r", m), kOk);  // CRLF tolerated
  CHECK_EQ(parse("$HAP,42,500,03007001*43", m), kOk);
}

TEST(test_formatting_matches_documented_examples) {
  char buf[72];
  RangeField r[4] = {{kRangeValid, 85}, {kRangeNoEcho, 0}, {kRangeError, 0}, {kRangeValid, 212}};
  uint8_t n = formatUltrasonic(buf, 1203, 845210, r);
  buf[n] = '\0';
  CHECK(strcmp(buf, "$US,1203,845210,85,X,E,212*2D\n") == 0);
  n = formatStatus(buf, 310, 845250, 'H', true, 42, 0);
  buf[n] = '\0';
  CHECK(strcmp(buf, "$ST,310,845250,H,42,0*69\n") == 0);
  n = formatStatus(buf, 311, 845500, 'F', false, 0, 3);
  buf[n] = '\0';
  CHECK(strcmp(buf, "$ST,311,845500,F,-,3*4C\n") == 0);
  n = formatStatus(buf, 65535, 4294967295UL, 'F', true, 65535, 65535);
  CHECK(n <= kMaxLineBytes);
  RangeField big[4] = {{kRangeValid, 999}, {kRangeValid, 999}, {kRangeValid, 999}, {kRangeValid, 999}};
  n = formatUltrasonic(buf, 65535, 4294967295UL, big);
  buf[n] = '\0';
  CHECK(strcmp(buf, "$US,65535,4294967295,999,999,999,999*3B\n") == 0);
  n = formatHello(buf);
  buf[n] = '\0';
  CHECK(strncmp(buf, "$HLO,1," HS_FIRMWARE_VERSION "*", 7) == 0);
}

TEST(test_parser_rejects_malformed_lines) {
  HostMessage m;
  char buf[128];
  CHECK_EQ(parse("HAP,1,500,00000000*00", m), kFraming);       // no '$'
  CHECK_EQ(parse("$HAP,1,500,00000000", m), kFraming);         // no checksum
  CHECK_EQ(parse("$HAP,1,500,00000000*0", m), kFraming);       // short checksum
  CHECK_EQ(parse("$HAP,1,500,00000000*ZZ", m), kFraming);      // non-hex checksum
  CHECK_EQ(parse("$HAP,1,500,00000000*00", m), kChecksum);     // wrong checksum
  CHECK_EQ(parse("$HAP,1\x01,500*00", m), kFraming);           // non-printable
  CHECK_EQ(parse("", m), kEmpty);
  CHECK_EQ(parse("   ", m), kEmpty);
  const char* bad_bodies[] = {
      "HAP,1,500",            "HAP,1,500,0000000",    "HAP,1,500,0000000g",
      "HAP,1,500,0000000a",   "HAP,1,0,00000000",     "HAP,1,2001,00000000",
      "HAP,65536,500,00000000", "HAP,-1,500,00000000", "HAP,,500,00000000",
      "HAP,99999999999,500,00000000", "HAP,4294967296,500,00000000",
      "PAT,0,100,0,1,0",      "PAT,3,0,0,1,100",      "PAT,3,100,0,10,100",
      "PAT,16,100,0,1,100",   "PAT,3,65536,0,1,0",    "PAT,3,100,0,1",
      "HAP",                  "HAP,",                 "HAP,1,500,00000000,9",
  };
  for (unsigned i = 0; i < sizeof(bad_bodies) / sizeof(bad_bodies[0]); ++i) {
    frame(buf, bad_bodies[i]);
    ParseError e = parse(buf, m);
    if (e == kOk) printf("  unexpectedly accepted: %s\n", buf);
    CHECK(e != kOk);
  }
  frame(buf, "FOO,1");
  CHECK_EQ(parse(buf, m), kUnknownType);
  frame(buf, "US,1,100,85,X,E,2");
  CHECK_EQ(parse(buf, m), kUnexpectedType);  // valid v1, but Arduino->host only
  frame(buf, "HLO,1,fw");
  CHECK_EQ(parse(buf, m), kUnexpectedType);
  // lowercase checksum digits are accepted (host decoder does the same)
  snprintf(buf, sizeof(buf), "$HAP,42,500,03007001*%02x", 0x43);
  CHECK_EQ(parse(buf, m), kOk);
}

TEST(test_line_assembler_overflow_and_resync) {
  LineAssembler a;
  int overflows = 0, lines = 0;
  for (int i = 0; i < 200; ++i) {
    LineAssembler::Result r = a.push('x');
    overflows += r == LineAssembler::kOverflow;
  }
  CHECK_EQ(overflows, 1);  // reported once, not per byte
  CHECK_EQ(a.push('\n'), LineAssembler::kNone);  // end of the discarded line
  const char* good = "$HAP,42,500,03007001*43\n";
  for (const char* p = good; *p; ++p) lines += a.push(static_cast<uint8_t>(*p)) == LineAssembler::kLine;
  CHECK_EQ(lines, 1);
  CHECK(strcmp(a.line(), "$HAP,42,500,03007001*43") == 0);
  // exactly 63 characters is allowed, 64 is not
  LineAssembler b;
  for (int i = 0; i < 63; ++i) CHECK_EQ(b.push('a'), LineAssembler::kNone);
  CHECK_EQ(b.push('\n'), LineAssembler::kLine);
  CHECK_EQ(b.length(), 63);
  LineAssembler c;
  LineAssembler::Result last = LineAssembler::kNone;
  for (int i = 0; i < 64; ++i) last = c.push('a');
  CHECK_EQ(last, LineAssembler::kOverflow);
}

// ---------------------------------------------------------------- patterns / scheduler

TEST(test_pattern_timing_matches_host_semantics) {
  Pattern p = {100, 50, 250, 2};  // same example as tests/test_haptics.py
  CHECK_EQ(patternCycleMs(p), 500);
  const uint32_t t[] = {0, 99, 100, 149, 150, 249, 250, 499, 500};
  const bool e[] = {true, true, false, false, true, true, false, false, true};
  for (int i = 0; i < 9; ++i) CHECK_EQ(patternIsOn(p, t[i]), e[i]);
  Pattern cont = {200, 0, 0, 1};
  for (uint32_t x = 0; x < 1000; x += 37) CHECK(patternIsOn(cont, x));
  Pattern off = {0, 0, 0, 0};
  CHECK(!patternIsOn(off, 5));
}

TEST(test_pattern_table_defined_and_undefined_slots) {
  PatternTable t;
  for (int s = 1; s <= 8; ++s) CHECK(t.isDefined(static_cast<uint8_t>(s)));
  for (int s = 9; s < 16; ++s) CHECK(!t.isDefined(static_cast<uint8_t>(s)));
  CHECK(!t.isDefined(0));
  CHECK_EQ(t.get(12).count, 0);  // undefined plays as OFF
  Pattern p = {10, 10, 10, 1};
  CHECK(!t.define(0, p));        // slot 0 is fixed
  CHECK(t.define(12, p));
  CHECK(t.isDefined(12));
  Pattern bad = {0, 0, 0, 2};
  CHECK(!t.define(13, bad));
  CHECK_EQ(t.get(2).on_ms, 150);  // WARNING default
  CHECK_EQ(t.get(7).count, 3);    // DEGRADED default
}

TEST(test_scheduler_phase_persistence_and_restart) {
  PatternTable t;
  ChannelScheduler s;
  uint8_t slots[8] = {2, 0, 0, 0, 0, 0, 0, 0};  // M0 WARNING: 150 on / 350 pause
  s.setSlots(slots, 1000, t);
  CHECK_EQ(s.outputByte(1000, t), 0x01);
  CHECK_EQ(s.outputByte(1200, t), 0x00);
  s.setSlots(slots, 1200, t);              // same slot again: keeps phase
  CHECK_EQ(s.outputByte(1250, t), 0x00);   // would be ON if it had restarted at 1200
  slots[0] = 3;                            // change -> restart
  s.setSlots(slots, 1300, t);
  CHECK_EQ(s.outputByte(1300, t), 0x01);
  // motor ordering: bit k is motor Mk
  uint8_t all[8] = {0, 3, 0, 0, 7, 0, 0, 1};
  ChannelScheduler s2;
  s2.setSlots(all, 0, t);
  CHECK_EQ(s2.outputByte(0, t), (1 << 1) | (1 << 4) | (1 << 7));
}

TEST(test_scheduler_across_millis_rollover) {
  PatternTable t;
  ChannelScheduler s;
  uint8_t slots[8] = {2, 0, 0, 0, 0, 0, 0, 0};
  uint32_t start = 0xFFFFFF00UL;  // 256 ms before rollover
  s.setSlots(slots, start, t);
  CHECK_EQ(s.outputByte(start + 100, t), 0x01);
  CHECK_EQ(s.outputByte(start + 200, t), 0x00);
  CHECK_EQ(s.outputByte(start + 500, t), 0x01);  // wrapped: now_ms < start numerically
  CHECK(start + 500 < start);
}

TEST(test_redefined_slot_restarts_channel) {
  PatternTable t;
  ChannelScheduler s;
  uint8_t slots[8] = {9, 0, 0, 0, 0, 0, 0, 0};
  s.setSlots(slots, 0, t);
  CHECK_EQ(s.outputByte(10, t), 0x00);  // slot 9 undefined -> OFF
  Pattern p = {100, 0, 0, 1};
  t.define(9, p);
  s.setSlots(slots, 50, t);
  CHECK_EQ(s.outputByte(60, t), 0x01);
}

// ---------------------------------------------------------------- ultrasonic

TEST(test_ultrasonic_classification) {
  uint16_t cm;
  CHECK_EQ(UltrasonicSequencer::classify(80 * 58, cm), kRangeValid);
  CHECK_EQ(cm, 80);
  CHECK_EQ(UltrasonicSequencer::classify(58, cm), kRangeError);       // 1 cm: implausible
  CHECK_EQ(UltrasonicSequencer::classify(2 * 58, cm), kRangeValid);
  CHECK_EQ(UltrasonicSequencer::classify(400 * 58, cm), kRangeValid);
  CHECK_EQ(UltrasonicSequencer::classify(401 * 58, cm), kRangeNoEcho);
  CHECK_EQ(UltrasonicSequencer::classify(38000, cm), kRangeNoEcho);
}

TEST(test_ultrasonic_staggered_schedule_and_rate) {
  FakeHal hal;
  UltrasonicSequencer seq(hal);
  hal.set(0, hs_test::kEcho, 80);
  hal.set(1, hs_test::kNoEcho);
  hal.set(2, hs_test::kNoRise);
  hal.set(3, hs_test::kStuckHigh);
  seq.reset(0);
  uint32_t cycles_done = 0, last_fire = 0, min_gap = 0xFFFFFFFF;
  uint32_t prev_count = 0;
  for (uint32_t ms = 0; ms <= 2000; ++ms) {
    hal.now_us = ms * 1000u;
    if (seq.update(ms, hal.now_us)) ++cycles_done;
    if (hal.fire_count != prev_count) {
      if (prev_count) {
        uint32_t gap = ms - last_fire;
        if (gap < min_gap) min_gap = gap;
      }
      last_fire = ms;
      prev_count = hal.fire_count;
    }
  }
  // 4 slots x 50 ms = 200 ms per cycle -> 10 cycles in 2 s -> 5 Hz per sensor
  CHECK_EQ(cycles_done, 10);
  CHECK(min_gap >= HS_US_SLOT_MS);  // never fire two sensors closer than one slot
  CHECK_EQ(seq.reading(0).kind, kRangeValid);
  CHECK_EQ(seq.reading(0).cm, 80);
  CHECK_EQ(seq.reading(1).kind, kRangeNoEcho);
  CHECK_EQ(seq.reading(2).kind, kRangeError);
  CHECK_EQ(seq.lastErrorCause(2), kCauseNoRise);
  CHECK_EQ(seq.reading(3).kind, kRangeError);
  CHECK_EQ(seq.lastErrorCause(3), kCauseStuckHigh);
  CHECK_EQ(hal.fires[3], 0u);  // a stuck-high sensor is never triggered
}

TEST(test_ultrasonic_never_blocks_for_missing_echo) {
  // Each update() returns immediately; a missing echo just ends the slot later.
  FakeHal hal;
  UltrasonicSequencer seq(hal);
  hal.set(0, hs_test::kNoEcho);
  seq.reset(0);
  hal.now_us = 0;
  seq.update(0, 0);
  for (uint32_t us = 0; us < 30000; us += 100) {
    hal.now_us = us;
    seq.update(us / 1000, us);
  }
  CHECK_EQ(seq.reading(0).kind, kRangeNoEcho);
}

TEST(test_ultrasonic_micros_rollover) {
  FakeHal hal;
  UltrasonicSequencer seq(hal);
  hal.set(0, hs_test::kEcho, 120);
  uint32_t base_us = 0xFFFFFFFFUL - 3000;  // micros() wraps during the echo
  seq.reset(0);
  for (uint32_t ms = 0; ms < 40; ++ms) {
    hal.now_us = base_us + ms * 1000u;
    seq.update(ms, hal.now_us);
  }
  CHECK_EQ(seq.reading(0).kind, kRangeValid);
  CHECK_EQ(seq.reading(0).cm, 120);
}

// ---------------------------------------------------------------- fallback policy

TEST(test_fallback_bands_and_missing_data) {
  FallbackPolicy fb;
  Reading r[4] = {{kRangeValid, 40, 1000}, {kRangeValid, 99, 1000}, {kRangeValid, 200, 1000},
                  {kRangeNoEcho, 0, 1000}};
  uint8_t slots[8];
  fb.compute(r, 1000, slots);
  CHECK_EQ(slots[0], kSlotCritical);
  CHECK_EQ(slots[2], kSlotWarning);
  CHECK_EQ(slots[4], kSlotCaution);
  CHECK_EQ(slots[6], kSlotOff);
  for (int m = 1; m < 8; m += 2) CHECK_EQ(slots[m], kSlotOff);  // diagonals off
  Reading bad[4] = {{kRangeError, 0, 1000}, {kRangeNone, 0, 0}, {kRangeValid, 100, 0},
                    {kRangeValid, 201, 1000}};
  FallbackPolicy fb2;
  fb2.compute(bad, 1000, slots);
  CHECK_EQ(slots[0], kSlotDegraded);  // error is not "clear"
  CHECK_EQ(slots[2], kSlotDegraded);  // never measured
  CHECK_EQ(slots[4], kSlotDegraded);  // stale (1000 ms old)
  CHECK_EQ(slots[6], kSlotOff);       // beyond caution band
}

TEST(test_fallback_hysteresis) {
  FallbackPolicy fb;
  Reading r[4] = {{kRangeValid, 40, 0}, {kRangeNoEcho, 0, 0}, {kRangeNoEcho, 0, 0},
                  {kRangeNoEcho, 0, 0}};
  uint8_t slots[8];
  fb.compute(r, 0, slots);
  CHECK_EQ(slots[0], kSlotCritical);
  r[0].kind = kRangeNoEcho;
  r[0].t_ms = 200;
  fb.compute(r, 200, slots);
  CHECK_EQ(slots[0], kSlotCritical);  // held
  r[0].t_ms = 400;
  fb.compute(r, 400, slots);
  CHECK_EQ(slots[0], kSlotOff);  // released after 350 ms
}

// ---------------------------------------------------------------- firmware modes

TEST(test_boot_announces_and_keeps_motors_off) {
  Rig rig;
  rig.begin();
  CHECK(strncmp(rig.tx[0], "$HLO,1,", 7) == 0);
  CHECK(strncmp(rig.tx[1], "$ST,0,0,B,-,0*", 14) == 0);
  CHECK_EQ(rig.fw.mode(), kModeBoot);
  rig.hal.set(0, hs_test::kEcho, 30);
  rig.advance(3000);
  CHECK_EQ(rig.fw.mode(), kModeBoot);  // no HAP yet: stays BOOT
  CHECK_EQ(rig.fw.motorByte(rig.ms), 0);
  CHECK(rig.countPrefix("$US,") >= 14);  // ultrasonic is reported in every mode
  CHECK(rig.countPrefix("$ST,") >= 15);  // heartbeat every 200 ms
}

TEST(test_hap_establishes_host_and_drives_motors) {
  Rig rig;
  rig.begin();
  rig.sendBody("HAP,7,500,30000000");  // M0 CRITICAL
  CHECK_EQ(rig.fw.mode(), kModeHost);
  rig.advance(1);
  CHECK_EQ(rig.fw.motorByte(rig.ms) & 0x01, 1);
  CHECK(strstr(rig.last("$ST,"), ",H,7,") != 0);  // mode change announced, seq echoed
}

TEST(test_command_expiry_enters_fallback_and_recovers) {
  Rig rig;
  rig.begin();
  rig.hal.set(1, hs_test::kEcho, 60);  // S1 -> M2 WARNING in fallback
  rig.sendBody("HAP,1,300,00000000");
  rig.advance(299);
  CHECK_EQ(rig.fw.mode(), kModeHost);
  rig.advance(1);
  CHECK_EQ(rig.fw.mode(), kModeFallback);
  CHECK(strstr(rig.last("$ST,"), ",F,1,") != 0);
  CHECK_EQ(rig.fw.counters().expiries, 1u);
  // entry cue: DEGRADED on all motors at the start of fallback
  CHECK_EQ(rig.fw.motorByte(rig.ms), 0xFF);
  rig.advance(600);  // cue over, sensors measured
  CHECK_EQ(rig.fw.slot(2), kSlotWarning);
  CHECK_EQ(rig.fw.slot(1), kSlotOff);
  rig.sendBody("HAP,2,500,00000000");
  CHECK_EQ(rig.fw.mode(), kModeHost);
  rig.advance(1);
  CHECK_EQ(rig.fw.motorByte(rig.ms), 0);
}

TEST(test_malformed_input_never_changes_commands) {
  Rig rig;
  rig.begin();
  rig.sendBody("HAP,1,1000,20000000");
  rig.advance(10);
  uint16_t errs = rig.fw.counters().rx_errors;
  rig.send("$HAP,2,1000,33333333*00");          // bad checksum
  rig.send("$HAP,2,1000,3333333");               // truncated
  rig.sendBody("HAP,2,1000,3333333g");          // invalid hex
  rig.sendBody("HAP,2,9999,33333333");          // ttl above cap
  char longline[120];
  memset(longline, 'A', 119);
  longline[119] = '\0';
  rig.send(longline);                            // overflow
  CHECK_EQ(rig.fw.counters().rx_errors, errs + 5);
  CHECK_EQ(rig.fw.slot(0), 2);
  CHECK_EQ(rig.fw.slot(1), 0);  // nothing partially applied
  // malformed lines do not refresh the watchdog either
  rig.advance(995);
  CHECK_EQ(rig.fw.mode(), kModeFallback);
  // and malformed lines during fallback do not leave it
  rig.send("$HAP,3,1000,00000000*00");
  CHECK_EQ(rig.fw.mode(), kModeFallback);
}

TEST(test_rapid_disconnect_reconnect_cycles) {
  Rig rig;
  rig.begin();
  for (int i = 0; i < 20; ++i) {
    char body[40];
    snprintf(body, sizeof(body), "HAP,%d,100,00000000", i);
    rig.sendBody(body);
    CHECK_EQ(rig.fw.mode(), kModeHost);
    rig.advance(static_cast<uint32_t>(150 + (i % 3) * 40));
    CHECK_EQ(rig.fw.mode(), kModeFallback);
  }
  CHECK_EQ(rig.fw.counters().expiries, 20u);
}

TEST(test_pat_defines_slot_and_undefined_slots_play_off) {
  Rig rig;
  rig.begin();
  rig.sendBody("HAP,1,1000,90000000");  // slot 9 undefined -> OFF
  rig.advance(5);
  CHECK_EQ(rig.fw.motorByte(rig.ms), 0);
  rig.sendBody("PAT,9,100,0,1,0");      // continuous
  rig.sendBody("HAP,2,1000,90000000");
  rig.advance(5);
  CHECK_EQ(rig.fw.motorByte(rig.ms), 0x01);
  CHECK_EQ(rig.fw.counters().rx_pat, 1u);
}

TEST(test_sequence_number_wraparound) {
  Rig rig;
  rig.begin();
  rig.sendBody("HAP,65535,1000,00000000");
  rig.advance(201);
  CHECK(strstr(rig.last("$ST,"), ",H,65535,") != 0);
  rig.sendBody("HAP,0,1000,00000000");
  rig.advance(201);
  CHECK(strstr(rig.last("$ST,"), ",H,0,") != 0);
}

TEST(test_millis_rollover_during_host_and_fallback) {
  Rig rig;
  rig.begin(0xFFFFFF00UL);  // 256 ms before millis() wraps
  rig.sendBody("HAP,1,500,20000000");
  rig.advance(400);         // crosses zero
  CHECK_EQ(rig.fw.mode(), kModeHost);
  rig.advance(100);
  CHECK_EQ(rig.fw.mode(), kModeFallback);
}

TEST(test_rx_errors_saturate) {
  Rig rig;
  rig.begin();
  for (long i = 0; i < 70000; ++i) rig.send("$X*00");
  CHECK_EQ(rig.fw.counters().rx_errors, 65535);
}

TEST(test_fallback_with_sensor_errors_signals_degraded) {
  Rig rig;
  rig.begin();
  rig.hal.set(0, hs_test::kNoRise);  // S0 missing
  rig.sendBody("HAP,1,100,00000000");
  rig.advance(1000);
  CHECK_EQ(rig.fw.mode(), kModeFallback);
  CHECK_EQ(rig.fw.slot(0), kSlotDegraded);  // missing front sensor is not "clear"
  CHECK_EQ(rig.fw.slot(2), kSlotOff);       // S1 no echo: clear in that cone
}

int main() {
  RUN(test_checksum_and_pinned_examples);
  RUN(test_formatting_matches_documented_examples);
  RUN(test_parser_rejects_malformed_lines);
  RUN(test_line_assembler_overflow_and_resync);
  RUN(test_pattern_timing_matches_host_semantics);
  RUN(test_pattern_table_defined_and_undefined_slots);
  RUN(test_scheduler_phase_persistence_and_restart);
  RUN(test_scheduler_across_millis_rollover);
  RUN(test_redefined_slot_restarts_channel);
  RUN(test_ultrasonic_classification);
  RUN(test_ultrasonic_staggered_schedule_and_rate);
  RUN(test_ultrasonic_never_blocks_for_missing_echo);
  RUN(test_ultrasonic_micros_rollover);
  RUN(test_fallback_bands_and_missing_data);
  RUN(test_fallback_hysteresis);
  RUN(test_boot_announces_and_keeps_motors_off);
  RUN(test_hap_establishes_host_and_drives_motors);
  RUN(test_command_expiry_enters_fallback_and_recovers);
  RUN(test_malformed_input_never_changes_commands);
  RUN(test_rapid_disconnect_reconnect_cycles);
  RUN(test_pat_defines_slot_and_undefined_slots_play_off);
  RUN(test_sequence_number_wraparound);
  RUN(test_millis_rollover_during_host_and_fallback);
  RUN(test_rx_errors_saturate);
  RUN(test_fallback_with_sensor_errors_signals_degraded);
  printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
