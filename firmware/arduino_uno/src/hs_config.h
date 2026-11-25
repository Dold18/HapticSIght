// HapticSight Arduino Uno firmware: compile-time configuration.
//
// *** PIN ASSIGNMENTS ARE PROVISIONAL ***
// They are a software proposal pending confirmation by the CAD/electronics
// workstream (see docs/HARDWARE_INTERFACE.md section 3a). Change them here only,
// and update that document when the wiring is confirmed.
//
// Everything in this header is plain C/C++ so the logic can also be compiled
// and unit-tested natively (no Arduino headers).
#ifndef HS_CONFIG_H
#define HS_CONFIG_H

#include <stdint.h>

#define HS_FIRMWARE_VERSION "0.3.0"
#define HS_PROTOCOL_VERSION 1
#define HS_SERIAL_BAUD 115200UL

// ---------------------------------------------------------------- pins (PROVISIONAL)
// Ultrasonic sensors, order S0 front, S1 right, S2 rear, S3 left (protocol order).
// Echo pins must support pin-change interrupts (all Uno digital pins do).
#define HS_PIN_TRIG_S0 2
#define HS_PIN_ECHO_S0 3
#define HS_PIN_TRIG_S1 4
#define HS_PIN_ECHO_S1 5
#define HS_PIN_TRIG_S2 6
#define HS_PIN_ECHO_S2 7
#define HS_PIN_TRIG_S3 8
#define HS_PIN_ECHO_S3 9

// 74HC595: QA = M0 ... QH = M7 (shifted MSB = QH first).
#define HS_PIN_SR_DATA 10   // SER   (74HC595 pin 14)
#define HS_PIN_SR_CLOCK 11  // SRCLK (74HC595 pin 11)
#define HS_PIN_SR_LATCH 12  // RCLK  (74HC595 pin 12)
// Output enable, ACTIVE LOW (74HC595 pin 13). Hardware MUST also have an
// external pull-up (e.g. 10 kOhm to 5 V) so outputs stay disabled during
// reset and the bootloader, before this firmware runs. Set to -1 if OE is
// hard-wired (not recommended).
#define HS_PIN_SR_OE 14  // = A0 on the Uno (digital pin number 14)
#define HS_PIN_STATUS_LED 13  // on-board LED: mode indicator (optional, see below)
#define HS_USE_STATUS_LED 1

// ---------------------------------------------------------------- ultrasonic timing
// Sensors are fired one at a time in fixed slots to limit acoustic cross-talk.
// Per-sensor update rate = 1000 / (4 * HS_US_SLOT_MS) Hz = 5 Hz by default.
// Do not shorten the slot without measuring cross-talk on the real vest.
#define HS_US_SLOT_MS 50U            // trigger-to-trigger spacing between sensors
#define HS_US_TRIGGER_US 10U         // HC-SR04 trigger pulse width
#define HS_US_RISE_TIMEOUT_US 4000UL // echo must start within this after trigger
#define HS_US_MAX_ECHO_US 25000UL    // longer echo = nothing within ~430 cm -> NO_ECHO
#define HS_US_US_PER_CM 58U          // round trip at ~343 m/s (20 C); +/-1.7% per 10 C
#define HS_US_MIN_CM 2U              // shorter = implausible -> ERROR
#define HS_US_MAX_CM 400U            // longer = NO_ECHO (HC-SR04 rated range)

// ---------------------------------------------------------------- protocol timing
#define HS_ST_PERIOD_MS 200U   // status heartbeat (protocol requires <= 250 ms)
#define HS_MAX_TTL_MS 2000U    // protocol cap: longer HAP TTLs are rejected

// ---------------------------------------------------------------- fallback
// Ultrasonic-only local warnings when host commands expire (protocol section 6).
#define HS_FB_CRITICAL_CM 50U   // d < 50          -> CRITICAL (slot 3)
#define HS_FB_WARNING_CM 100U   // 50 <= d < 100   -> WARNING  (slot 2)
#define HS_FB_CAUTION_CM 200U   // 100 <= d <= 200 -> CAUTION  (slot 1)
#define HS_FB_STALE_MS 600U     // no completed reading for this long -> DEGRADED
#define HS_FB_RELEASE_HOLD_MS 350U  // anti-flicker hold before stepping down
// Mode-change cue on entering FALLBACK: one DEGRADED burst on all 8 motors.
// All eight motors start simultaneously; if the motor supply cannot deliver
// that start-up current (NOT YET VERIFIED), set to 0 to cue only M0/M2/M4/M6.
#define HS_FB_CUE_ALL_MOTORS 1

// Protocol: BOOT (motors off) lasts until the first valid HAP. Set non-zero to
// enter FALLBACK automatically after this many ms without any host command
// (standalone vest). OFF by default because it deviates from protocol v1 text;
// it is an open design decision for the project owner.
#define HS_BOOT_TO_FALLBACK_MS 0UL

#endif  // HS_CONFIG_H
