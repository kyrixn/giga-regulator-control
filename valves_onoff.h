/**
 * valves_onoff.h
 *
 * Six 2-position (on/off) solenoid valves driven through an 8-channel
 * optocoupler-isolated MOSFET board on GIGA pins D2..D7 (board inputs IN1..IN6).
 * IN7/IN8 are unused and left floating.
 *
 * Wiring: the board's logic side takes 3.3V + GND from the Giga; the 24V side
 * has its own supply and switches the low leg of each coil. The two grounds are
 * bonded at a single point, so the optical isolation is not in play -- keep the
 * 24V return out of the Giga's ground path or the switching noise lands on the
 * AD7606's 1-5V monitor inputs.
 *
 * These are entirely separate from the six proportional regulators: different
 * hardware, different serial namespace ('d' commands). The one thing they share
 * is the emergency stop, which drops both.
 *
 * Design contract: every call here is a handful of GPIO writes, so unlike adc::
 * and ui:: there is nothing to schedule and no tick(). Nothing blocks.
 */
#pragma once
#include <Arduino.h>

#define DV_NUM_VALVES 6

// Driver polarity. Optocoupler input boards are almost always active-LOW: the
// IN pin sinks current through the opto LED, so pulling IN to GND energises the
// channel and driving it high releases the valve. Set this to 0 if the bench
// test showed IN must go HIGH to actuate.
#define DV_ACTIVE_LOW 1

namespace dv {
  void begin();                  // park every channel OFF, then drive the pins
  bool set(int valve, bool on);  // false on a bad index
  bool get(int valve);           // last commanded state (no readback on the board)
  int  pin(int valve);           // Giga GPIO behind an index, -1 if out of range
  void allOff();
  int  activeCount();
  void printStatus();            // the 'd?' command
}
