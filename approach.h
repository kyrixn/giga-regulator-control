/**
 * approach.h
 *
 * Always arrive at a setpoint from below, so a calibration table taken on the
 * ascending pass is the table that actually applies.
 *
 * WHY
 *   V2 has ~28 kPa of average hysteresis (V0, the healthy reference, has 4).
 *   The calibration table in calib_table.cpp is built from the ascending sweep,
 *   so it is correct only when the pressure climbs into the setpoint. Come down
 *   onto the same setpoint and the real pressure sits up to ~35 kPa high, table
 *   or no table -- no lookup can correct an error that depends on direction.
 *
 *   So: rising commands are applied straight away, because they already arrive
 *   from below. Falling commands first undershoot past the target by more than
 *   the hysteresis, wait for the pressure to actually get there, then come back
 *   up to the setpoint.
 *
 * COST
 *   A downward move takes APPROACH_MS longer. Upward moves are unaffected.
 *
 * DRAGGING
 *   Each new request restarts the undershoot, so dragging a slider downwards
 *   parks the valve low and only settles on the final value once the finger
 *   stops. That is the right behaviour: mid-drag values are not setpoints.
 *
 * Design contract: tick() is cooperative and non-blocking, like everything else
 * called from loop().
 */
#pragma once
#include <Arduino.h>

// Off. Length from the RS-485 encoders is the primary feedback on this rig and
// pressure is auxiliary, so 800ms added to every downward move costs more than
// the hysteresis it removes. Set to 1 to trade that latency back for accuracy
// if a setpoint ever has to be trusted open-loop.
//
// With this off, V2 lands up to ~35 kPa high whenever a setpoint is approached
// from above. The calibration table still applies; it is simply the ascending
// table being used in both directions.
#define APPROACH_ENABLED 0

// How far below the target to dip, in mV of command. Must exceed the valve's
// hysteresis or the trick does nothing: V2's 36 kPa worst case is ~400mV at
// 0.09 kPa/mV, so 600mV (~54 kPa) leaves margin.
#define APPROACH_UNDERSHOOT_MV 600

// Dwell at the undershoot before rising to the target. Pneumatic settling.
#define APPROACH_MS 800

namespace apr {
  typedef void (*WriteFn)(int valve, int mV);

  void begin(WriteFn fn);
  void request(int valve, int mV);   // setpoint; may defer, may undershoot
  void force(int valve, int mV);     // write now, no undershoot (stops, init)
  void setEnabled(bool on);          // the calibration sweep turns this off
  bool enabled();
  void tick();
}
