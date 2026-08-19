/**
 * adc_ad7606.cpp  --  see adc_ad7606.h
 *
 * Read sequence per sweep (all boards convert simultaneously off the shared
 * CONVST):
 *   pulse CONVST high->low  (rising edge starts the conversion)
 *   wait BUSY low           (~4us with OS off; hard timeout so a missing board
 *                            can never hang loop())
 *   for each board: CS low, read 8 x 16-bit on DOUTA, CS high
 *
 * Interface: hardware SPI, 5 MHz, MSB-first, mode 2 (CPOL=1, CPHA=0) -- matches
 * the bit-bang that proved out (SCK idle high, sample DOUTA on the falling edge).
 * NOTE: uses the `SPI1` object, NOT `SPI`. On the GIGA R1 the D11/12/13 header
 * pins silkscreened COPI/CIPO/SCK belong to SPI1; the default `SPI` object is
 * routed to a different header (pins D89/D90/D91 = PG9/PD7/PB3). SCK + CIPO are
 * the only lines used; COPI sends dummy 0x00 the AD7606 ignores.
 */
#include "adc_ad7606.h"
#include <SPI.h>

// ---- Pin map (single-board bring-up uses index 0 of the per-board arrays) ---
// SCK=D13, CIPO/DOUTA=D12, COPI=D11 are the fixed SPI1 pins (wired already).
static const uint8_t PIN_CONVST = 39;   // CVA + CVB tied together
static const uint8_t PIN_RST    = 40;
static const uint8_t CS_PINS[4]   = { 41, 42, 43, 44 };
static const uint8_t BUSY_PINS[4] = { 45, 46, 47, 48 };

static const SPISettings kSpi(5000000, MSBFIRST, SPI_MODE2);

// ---- State (written only from the main loop via tick(); no ISRs) -----------
static int16_t  g_raw[ADC_NUM_CH];
static bool     g_present[ADC_NUM_BOARDS];
static uint32_t g_seq       = 0;
static uint32_t g_lastUs    = 0;

// Rising edge on CONVST starts a conversion on every board at once.
static inline void pulseConvst() {
  digitalWrite(PIN_CONVST, HIGH);
  delayMicroseconds(1);
  digitalWrite(PIN_CONVST, LOW);
}

// Wait until no present board is still BUSY. Returns false on timeout.
static bool waitBusyDone(uint32_t timeoutUs = 300) {
  uint32_t t0 = micros();
  for (;;) {
    bool busy = false;
    for (int b = 0; b < ADC_NUM_BOARDS; b++) {
      if (g_present[b] && digitalRead(BUSY_PINS[b])) { busy = true; break; }
    }
    if (!busy) return true;
    if ((uint32_t)(micros() - t0) > timeoutUs) return false;
  }
}

// Convert every board, then clock 8 channels x 16 bits out of each.
static void acquire() {
  pulseConvst();
  waitBusyDone();

  SPI1.beginTransaction(kSpi);
  for (int b = 0; b < ADC_NUM_BOARDS; b++) {
    digitalWrite(CS_PINS[b], LOW);
    for (int ch = 0; ch < ADC_CH_PER_BOARD; ch++) {
      uint8_t hi = SPI1.transfer(0x00);
      uint8_t lo = SPI1.transfer(0x00);
      g_raw[b * ADC_CH_PER_BOARD + ch] = (int16_t)(((uint16_t)hi << 8) | lo);
    }
    digitalWrite(CS_PINS[b], HIGH);
  }
  SPI1.endTransaction();

  g_seq++;
}

namespace adc {

void begin() {
  pinMode(PIN_CONVST, OUTPUT); digitalWrite(PIN_CONVST, LOW);  // idle low
  pinMode(PIN_RST,    OUTPUT); digitalWrite(PIN_RST,    LOW);
  for (int b = 0; b < ADC_NUM_BOARDS; b++) {
    pinMode(CS_PINS[b], OUTPUT); digitalWrite(CS_PINS[b], HIGH);
    // Pulldown so a disconnected board reads BUSY=low (never a false timeout).
    pinMode(BUSY_PINS[b], INPUT_PULLDOWN);
  }
  for (int i = 0; i < ADC_NUM_CH; i++) g_raw[i] = 0;

  SPI1.begin();   // D13 SCK / D12 CIPO / D11 COPI

  // Reset pulse (RESET is active high; >50ns). Ignore the first conversion.
  digitalWrite(PIN_RST, HIGH);
  delayMicroseconds(5);
  digitalWrite(PIN_RST, LOW);
  delayMicroseconds(50);

  // Presence probe: after a CONVST, a wired board drives BUSY high (~4us).
  // Best-effort -- catches "BUSY/CS/power not connected".
  pulseConvst();
  for (int b = 0; b < ADC_NUM_BOARDS; b++) {
    bool seen = false;
    uint32_t t0 = micros();
    while ((uint32_t)(micros() - t0) < 60) {
      if (digitalRead(BUSY_PINS[b])) { seen = true; break; }
    }
    g_present[b] = seen;
    Serial.print("ADC board ");
    Serial.print(b);
    Serial.println(seen ? " present" : " ABSENT (check BUSY/CS/power/jumper)");
  }
  waitBusyDone();

  g_lastUs = micros();
}

void tick() {
  uint32_t now = micros();
  if ((uint32_t)(now - g_lastUs) < (1000000UL / ADC_SAMPLE_HZ)) return;
  g_lastUs = now;
  acquire();
}

int16_t raw(int ch) {
  if (ch < 0 || ch >= ADC_NUM_CH) return 0;
  return g_raw[ch];
}

// Calibration V_true = GAIN * V_measured + OFFSET, from a 5-point DMM sweep
// (0.5-9 V) with the source and ADC grounds tied. The module over-reads ~1.5%.
// Global (measured on one channel); per-channel constants could tighten it.
// Measured on the +/-10V range -- re-derive if ADC_RANGE_V changes.
static const float ADC_V_GAIN   = 0.9849f;
static const float ADC_V_OFFSET = 0.0054f;

float volts(int ch) {
  float v = raw(ch) * (ADC_RANGE_V / 32768.0f);   // bipolar full-scale / 2^15
  return v * ADC_V_GAIN + ADC_V_OFFSET;           // apply calibration
}

uint32_t seq() { return g_seq; }

bool present(int board) {
  if (board < 0 || board >= ADC_NUM_BOARDS) return false;
  return g_present[board];
}

void printAll() {
  Serial.print("ADC seq=");
  Serial.print(g_seq);
  for (int ch = 0; ch < ADC_NUM_CH; ch++) {
    Serial.print(" C");
    Serial.print(ch);
    Serial.print("=");
    Serial.print(raw(ch));
    Serial.print("(");
    Serial.print(volts(ch), 3);
    Serial.print("V)");
  }
  Serial.println();
}

} // namespace adc
