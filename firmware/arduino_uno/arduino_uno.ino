// HapticSight firmware for Arduino Uno (ATmega328P @ 16 MHz).
//
// Hardware glue only: pins, pin-change interrupts for echo timing, Serial, and
// the 74HC595 shift register. All behaviour lives in src/ (hardware-independent
// and unit-tested natively, see firmware/arduino_uno/README.md).
//
// PIN ASSIGNMENTS ARE PROVISIONAL: see src/hs_config.h and
// docs/HARDWARE_INTERFACE.md. ELECTRICAL PARAMETERS ARE NOT VERIFIED.

#include <Arduino.h>

#include "src/hs_config.h"
#include "src/hs_firmware.h"

namespace {

const uint8_t kTrigPins[4] = {HS_PIN_TRIG_S0, HS_PIN_TRIG_S1, HS_PIN_TRIG_S2, HS_PIN_TRIG_S3};
const uint8_t kEchoPins[4] = {HS_PIN_ECHO_S0, HS_PIN_ECHO_S1, HS_PIN_ECHO_S2, HS_PIN_ECHO_S3};

// ---------------------------------------------------------------- echo capture (ISR)
volatile uint8_t* g_echo_in_reg = 0;  // PINx register of the armed echo pin
volatile uint8_t g_echo_mask = 0;
volatile uint8_t g_edge_flags = 0;    // bit0 rise, bit1 fall
volatile uint32_t g_rise_us = 0;
volatile uint32_t g_fall_us = 0;

inline void handleEchoEdge() {
  if (g_echo_in_reg == 0) return;
  uint32_t t = micros();
  bool high = (*g_echo_in_reg & g_echo_mask) != 0;
  if (high && !(g_edge_flags & 1)) {
    g_rise_us = t;
    g_edge_flags |= 1;
  } else if (!high && (g_edge_flags & 1) && !(g_edge_flags & 2)) {
    g_fall_us = t;
    g_edge_flags |= 2;
  }
}

}  // namespace

ISR(PCINT0_vect) { handleEchoEdge(); }  // D8-D13
ISR(PCINT1_vect) { handleEchoEdge(); }  // A0-A5
ISR(PCINT2_vect) { handleEchoEdge(); }  // D0-D7

namespace {

class UnoUltrasonicHal : public hs::UltrasonicHal {
 public:
  void begin() {
    for (uint8_t s = 0; s < 4; ++s) {
      pinMode(kTrigPins[s], OUTPUT);
      digitalWrite(kTrigPins[s], LOW);
      // Pull-up: a disconnected echo wire reads HIGH and is reported as an error
      // instead of floating (HC-SR04 echo outputs are push-pull).
      pinMode(kEchoPins[s], INPUT_PULLUP);
      *digitalPinToPCICR(kEchoPins[s]) |= static_cast<uint8_t>(1 << digitalPinToPCICRbit(kEchoPins[s]));
    }
    disarm();
  }

  bool echoHigh(uint8_t s) override { return digitalRead(kEchoPins[s & 3]) == HIGH; }

  void fire(uint8_t s) override {
    s &= 3;
    disarm();
    uint8_t pin = kEchoPins[s];
    noInterrupts();
    g_echo_in_reg = portInputRegister(digitalPinToPort(pin));
    g_echo_mask = digitalPinToBitMask(pin);
    g_edge_flags = 0;
    *digitalPinToPCMSK(pin) |= static_cast<uint8_t>(1 << digitalPinToPCMSKbit(pin));
    interrupts();
    digitalWrite(kTrigPins[s], HIGH);
    delayMicroseconds(HS_US_TRIGGER_US);  // the only (10 us) busy wait
    digitalWrite(kTrigPins[s], LOW);
  }

  uint8_t edges(uint32_t& rise_us, uint32_t& fall_us) override {
    noInterrupts();
    uint8_t f = g_edge_flags;
    rise_us = g_rise_us;
    fall_us = g_fall_us;
    interrupts();
    return f;
  }

  void disarm() override {
    noInterrupts();
    for (uint8_t s = 0; s < 4; ++s) {
      *digitalPinToPCMSK(kEchoPins[s]) &= static_cast<uint8_t>(~(1 << digitalPinToPCMSKbit(kEchoPins[s])));
    }
    g_echo_in_reg = 0;
    interrupts();
  }
};

UnoUltrasonicHal g_hal;
hs::Firmware g_fw(g_hal);
char g_tx[hs::kMaxLineBytes + 1];
uint8_t g_last_motor_byte = 0;
uint32_t g_last_shift_ms = 0;

// ---------------------------------------------------------------- 74HC595
void shiftMotors(uint8_t value) {
  digitalWrite(HS_PIN_SR_LATCH, LOW);
  shiftOut(HS_PIN_SR_DATA, HS_PIN_SR_CLOCK, MSBFIRST, value);  // bit7 -> QH (M7), bit0 -> QA (M0)
  digitalWrite(HS_PIN_SR_LATCH, HIGH);
}

void setupOutputs() {
#if HS_PIN_SR_OE >= 0
  // Keep outputs disabled (OE high) while initialising. The external pull-up
  // already holds OE high through reset/bootloader; drive it high before
  // switching the pin to output so there is no low glitch.
  digitalWrite(HS_PIN_SR_OE, HIGH);
  pinMode(HS_PIN_SR_OE, OUTPUT);
#endif
  pinMode(HS_PIN_SR_DATA, OUTPUT);
  pinMode(HS_PIN_SR_CLOCK, OUTPUT);
  pinMode(HS_PIN_SR_LATCH, OUTPUT);
  digitalWrite(HS_PIN_SR_DATA, LOW);
  digitalWrite(HS_PIN_SR_CLOCK, LOW);
  shiftMotors(0x00);  // known all-off state in the storage register
#if HS_PIN_SR_OE >= 0
  digitalWrite(HS_PIN_SR_OE, LOW);  // now enable outputs (all off)
#endif
}

}  // namespace

void setup() {
  setupOutputs();
#if HS_USE_STATUS_LED
  pinMode(HS_PIN_STATUS_LED, OUTPUT);
#endif
  g_hal.begin();
  Serial.begin(HS_SERIAL_BAUD);
  g_fw.begin(millis());
}

void loop() {
  uint32_t now_ms = millis();
  // 1. Serial input: drain everything available (the RX buffer is only 64 bytes).
  while (Serial.available() > 0) {
    g_fw.onRxByte(static_cast<uint8_t>(Serial.read()), now_ms);
  }
  // 2. Sensors, watchdog, mode, pattern selection.
  g_fw.update(now_ms, micros());
  // 3. Motors: shift out on change, and refresh periodically against glitches.
  uint8_t motors = g_fw.motorByte(now_ms);
  if (motors != g_last_motor_byte || now_ms - g_last_shift_ms >= 100) {
    shiftMotors(motors);
    g_last_motor_byte = motors;
    g_last_shift_ms = now_ms;
  }
  // 4. Serial output without blocking: only write if the TX buffer has room.
  uint8_t n = g_fw.pendingFrame(g_tx);
  if (n > 0 && Serial.availableForWrite() >= n) {
    Serial.write(reinterpret_cast<const uint8_t*>(g_tx), n);
    g_fw.frameSent();
  }
#if HS_USE_STATUS_LED
  digitalWrite(HS_PIN_STATUS_LED, g_fw.statusLed(now_ms) ? HIGH : LOW);
#endif
}
