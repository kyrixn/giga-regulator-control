/**
 * valves_onoff.cpp  --  see valves_onoff.h
 */
#include "valves_onoff.h"

// IN1..IN6 on the driver board. D0/D1 are Serial1; D11/12/13 are SPI1 to the
// AD7606; D20/D21 are the DAC I2C bus; D39..D48 are the AD7606 control lines
// (only board 0 is configured, but the CS/BUSY arrays reserve the rest). That
// leaves D2..D7 as the one free contiguous block.
static const uint8_t DV_PINS[DV_NUM_VALVES] = { 2, 3, 4, 5, 6, 7 };

#if DV_ACTIVE_LOW
  static const uint8_t DV_ON_LEVEL  = LOW;
  static const uint8_t DV_OFF_LEVEL = HIGH;
#else
  static const uint8_t DV_ON_LEVEL  = HIGH;
  static const uint8_t DV_OFF_LEVEL = LOW;
#endif

// Commanded state. The driver board gives no readback, so this is the only
// record of what the coils were last told to do.
static bool g_on[DV_NUM_VALVES];

namespace dv {

void begin() {
  for (int i = 0; i < DV_NUM_VALVES; i++) {
    // Order matters. A pin switched to OUTPUT defaults to driving LOW, which on
    // an active-low board means ON -- all six valves would fire for as long as
    // it took to write them off again. Seed the output latch first so the pin
    // is already parked at the OFF level the instant the driver is enabled; the
    // trailing write covers cores where a write before pinMode() does not stick.
    digitalWrite(DV_PINS[i], DV_OFF_LEVEL);
    pinMode(DV_PINS[i], OUTPUT);
    digitalWrite(DV_PINS[i], DV_OFF_LEVEL);
    g_on[i] = false;
  }
}

bool set(int valve, bool on) {
  if (valve < 0 || valve >= DV_NUM_VALVES) return false;
  digitalWrite(DV_PINS[valve], on ? DV_ON_LEVEL : DV_OFF_LEVEL);
  g_on[valve] = on;
  return true;
}

bool get(int valve) {
  if (valve < 0 || valve >= DV_NUM_VALVES) return false;
  return g_on[valve];
}

// The index is not the pin number (index 0 is D2), and the UI labels its
// buttons with both, so the mapping is published rather than duplicated.
int pin(int valve) {
  if (valve < 0 || valve >= DV_NUM_VALVES) return -1;
  return DV_PINS[valve];
}

void allOff() {
  for (int i = 0; i < DV_NUM_VALVES; i++) set(i, false);
}

int activeCount() {
  int n = 0;
  for (int i = 0; i < DV_NUM_VALVES; i++) if (g_on[i]) n++;
  return n;
}

// Deliberately "D<i>=<0|1>", not "V<i>=...". The PC app treats any line that
// starts with V and contains '=' and '|' as a full regulator snapshot and
// clears its table from it, so the on/off valves must not look like one.
void printStatus() {
  Serial.println("=== On/Off valves: d2-d7 (number = Giga pin) ===");
  for (int i = 0; i < DV_NUM_VALVES; i++) {
    Serial.print("D");
    Serial.print(DV_PINS[i]);
    Serial.print("=");
    Serial.print(g_on[i] ? 1 : 0);
    if (i < DV_NUM_VALVES - 1) Serial.print(" | ");
  }
  Serial.println();
}

}  // namespace dv
