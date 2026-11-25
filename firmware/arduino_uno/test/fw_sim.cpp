// "Virtual Uno": the real firmware core (src/) compiled natively, with
// simulated HC-SR04 sensors. NOT hardware: anything it reports is simulated.
//
// Scripted mode (default): line commands on stdin, used by Python tests.
//   adv <ms>            advance time in 1 ms steps; prints TX/MOT/MODE events
//   rx <text>           deliver <text> + '\n' to the serial input
//   rxhex <hex>         deliver raw bytes
//   range <s> <v>       sensor s (0-3): <cm> | X (no echo) | NORISE | STUCK
//   parse <line>        parse a host line without changing state
//   patterns            dump the pattern table
//   state               mode and counters
//   reset               simulate an MCU reset (time continues)
//   quit
// Every command's output ends with a line containing a single '.'.
//
// Realtime mode (--realtime [--control-fd N]): raw serial bytes on stdin and
// stdout (e.g. a pseudo-terminal), real clock; control commands (range, reset,
// quit) arrive as lines on the control fd.
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "../src/hs_firmware.h"
#include "fake_hal.h"

using namespace hs;

namespace {

hs_test::FakeHal g_hal;
Firmware g_fw(g_hal);
uint32_t g_ms = 0;
uint8_t g_motor = 0;
char g_mode = 'B';

void setRange(int s, const char* v) {
  if (s < 0 || s > 3) return;
  if (strcmp(v, "X") == 0) {
    g_hal.set(static_cast<uint8_t>(s), hs_test::kNoEcho);
  } else if (strcmp(v, "NORISE") == 0) {
    g_hal.set(static_cast<uint8_t>(s), hs_test::kNoRise);
  } else if (strcmp(v, "STUCK") == 0) {
    g_hal.set(static_cast<uint8_t>(s), hs_test::kStuckHigh);
  } else {
    g_hal.set(static_cast<uint8_t>(s), hs_test::kEcho, static_cast<uint16_t>(atoi(v)));
  }
}

void flushTx(FILE* out, bool scripted) {
  char buf[72];
  uint8_t n;
  while ((n = g_fw.pendingFrame(buf)) > 0) {
    if (scripted) {
      buf[n - 1] = '\0';  // strip '\n'
      fprintf(out, "TX %s\n", buf);
    } else {
      fwrite(buf, 1, n, out);
    }
    g_fw.frameSent();
  }
  fflush(out);
}

void step(FILE* out, bool scripted, uint32_t now_us) {
  g_hal.now_us = now_us;
  g_fw.update(g_ms, now_us);
  flushTx(out, scripted);
  uint8_t m = g_fw.motorByte(g_ms);
  if (scripted && m != g_motor) printf("MOT %lu %02X\n", static_cast<unsigned long>(g_ms), m);
  g_motor = m;
  char mode = static_cast<char>(g_fw.mode());
  if (scripted && mode != g_mode) printf("MODE %lu %c\n", static_cast<unsigned long>(g_ms), mode);
  g_mode = mode;
}

int scripted() {
  char line[512];
  g_fw.begin(g_ms);
  flushTx(stdout, true);
  printf(".\n");
  fflush(stdout);
  while (fgets(line, sizeof(line), stdin)) {
    size_t n = strlen(line);
    while (n && (line[n - 1] == '\n' || line[n - 1] == '\r')) line[--n] = '\0';
    if (strncmp(line, "adv ", 4) == 0) {
      long dt = atol(line + 4);
      for (long i = 0; i < dt; ++i) {
        ++g_ms;
        step(stdout, true, g_ms * 1000u);
      }
    } else if (strncmp(line, "rx ", 3) == 0) {
      for (const char* p = line + 3; *p; ++p) g_fw.onRxByte(static_cast<uint8_t>(*p), g_ms);
      g_fw.onRxByte('\n', g_ms);
      step(stdout, true, g_ms * 1000u);
    } else if (strncmp(line, "rxhex ", 6) == 0) {
      for (const char* p = line + 6; p[0] && p[1]; p += 2) {
        char hx[3] = {p[0], p[1], 0};
        g_fw.onRxByte(static_cast<uint8_t>(strtol(hx, 0, 16)), g_ms);
      }
      step(stdout, true, g_ms * 1000u);
    } else if (strncmp(line, "range ", 6) == 0) {
      int s = atoi(line + 6);
      const char* v = strchr(line + 6, ' ');
      if (v) setRange(s, v + 1);
    } else if (strncmp(line, "parse ", 6) == 0) {
      HostMessage m;
      ParseError e = parseHostLine(line + 6, static_cast<uint8_t>(strlen(line + 6) > 255 ? 255 : strlen(line + 6)), m);
      if (strlen(line + 6) > 255) e = kFraming;
      if (e != kOk) {
        printf("ERR %d\n", static_cast<int>(e));
      } else if (m.type == kMsgHap) {
        printf("OK HAP %u %u", m.hap.seq, m.hap.ttl_ms);
        for (int i = 0; i < 8; ++i) printf(" %u", m.hap.slots[i]);
        printf("\n");
      } else {
        printf("OK PAT %u %u %u %u %u\n", m.pat.slot, m.pat.on_ms, m.pat.off_ms, m.pat.count,
               m.pat.pause_ms);
      }
    } else if (strcmp(line, "patterns") == 0) {
      for (uint8_t s = 0; s < PatternTable::kSlots; ++s) {
        const Pattern& p = g_fw.patterns().get(s);
        printf("PATTERN %u %d %u %u %u %u\n", s, g_fw.patterns().isDefined(s) ? 1 : 0, p.on_ms,
               p.off_ms, p.count, p.pause_ms);
      }
    } else if (strcmp(line, "state") == 0) {
      const Counters& c = g_fw.counters();
      printf("STATE %c %u %lu %lu %lu %02X\n", static_cast<char>(g_fw.mode()), c.rx_errors,
             static_cast<unsigned long>(c.rx_hap), static_cast<unsigned long>(c.rx_pat),
             static_cast<unsigned long>(c.expiries), g_fw.motorByte(g_ms));
    } else if (strcmp(line, "reset") == 0) {
      g_fw.begin(g_ms);
      flushTx(stdout, true);
    } else if (strcmp(line, "quit") == 0) {
      printf(".\n");
      return 0;
    }
    printf(".\n");
    fflush(stdout);
  }
  return 0;
}

uint64_t monotonicUs() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL + static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
}

int realtime(int control_fd) {
  uint64_t t0 = monotonicUs();
  g_fw.begin(0);
  char ctl[256];
  size_t ctl_len = 0;
  for (;;) {
    struct pollfd fds[2];
    int nfds = 1;
    fds[0].fd = 0;
    fds[0].events = POLLIN;
    if (control_fd >= 0) {
      fds[1].fd = control_fd;
      fds[1].events = POLLIN;
      nfds = 2;
    }
    poll(fds, static_cast<nfds_t>(nfds), 1);
    uint64_t now = monotonicUs() - t0;
    g_ms = static_cast<uint32_t>(now / 1000ULL);
    if (fds[0].revents & (POLLIN | POLLHUP)) {
      unsigned char buf[256];
      ssize_t n = read(0, buf, sizeof(buf));
      if (n <= 0 && (fds[0].revents & POLLHUP)) return 0;
      for (ssize_t i = 0; i < n; ++i) g_fw.onRxByte(buf[i], g_ms);
    }
    if (nfds == 2 && (fds[1].revents & POLLIN)) {
      char c;
      while (read(control_fd, &c, 1) == 1) {
        if (c == '\n') {
          ctl[ctl_len] = '\0';
          ctl_len = 0;
          if (strncmp(ctl, "range ", 6) == 0) {
            const char* v = strchr(ctl + 6, ' ');
            if (v) setRange(atoi(ctl + 6), v + 1);
          } else if (strcmp(ctl, "reset") == 0) {
            g_fw.begin(g_ms);
          } else if (strcmp(ctl, "quit") == 0) {
            return 0;
          }
          break;
        }
        if (ctl_len < sizeof(ctl) - 1) ctl[ctl_len++] = c;
      }
    } else if (nfds == 2 && (fds[1].revents & POLLHUP)) {
      return 0;  // controller went away
    }
    step(stdout, false, static_cast<uint32_t>(now));
  }
}

}  // namespace

int main(int argc, char** argv) {
  bool rt = false;
  int control_fd = -1;
  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--realtime") == 0) rt = true;
    if (strcmp(argv[i], "--control-fd") == 0 && i + 1 < argc) control_fd = atoi(argv[++i]);
  }
  if (rt) {
    setvbuf(stdout, 0, _IONBF, 0);
    return realtime(control_fd);
  }
  return scripted();
}
