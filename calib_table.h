/**
 * calib_table.h
 *
 * Per-regulator open-loop calibration: correct the commanded voltage so the
 * pressure that comes out is the pressure that was asked for.
 *
 * WHERE IT SITS
 *   setValve() computes the ideal command from the requested pressure, then
 *   passes it through calibApply() on the way to the DAC. Nothing else changes:
 *   currentValue[] still holds what was ASKED for, so the display, the status
 *   query and the PC app all keep showing the intent rather than the corrected
 *   voltage. Only the number reaching the DAC is different.
 *
 * HOW TO FILL A TABLE
 *   Run the sweep:  c<valve>,2200,3
 *   It prints, for each commanded point, the measured pressure. Each row of the
 *   table below is one of those points, entered as:
 *
 *       { measured_kPa, commanded_mV }
 *
 *   i.e. the axes SWAPPED from how the sweep prints them -- the table answers
 *   "to get this pressure, send this voltage", which is the inverse of what the
 *   sweep measures. Rows must be sorted by ascending kPa.
 *
 *   Use the ASCENDING pass only. Mixing both directions averages away the
 *   hysteresis instead of correcting it, and a table cannot correct hysteresis
 *   in the first place.
 *
 * WHAT IT CANNOT DO
 *   Nothing here beats the regulator's repeatability. If the same command
 *   measures 62, 68 and 66 kPa on three passes, the residual after calibration
 *   is still about +/-3 kPa. The table removes the systematic offset, not the
 *   scatter. It also cannot reach below the dead band: requests under the
 *   table's first point clamp to that point, and calibMinKpa() reports where
 *   that floor is so the UI can say so rather than lie.
 */
#pragma once
#include <Arduino.h>
#include "valve_core.h"

// Set to 0 to bypass every table and drive the DACs with the ideal mapping.
#define CALIB_ENABLED 1

// Max rows per valve. The sweep produces CAL_POINTS of them.
#define CALIB_MAX_POINTS 16

struct CalibPoint {
  float kPa;    // pressure actually measured
  int   mV;     // command that produced it
};

// Correct a command. `desiredMv` is the ideal command for the wanted pressure;
// the return value is what the DAC should actually be given. Falls through
// unchanged for a valve with no table, so an uncalibrated rig behaves exactly
// as it did before.
int   calibApply(int valve, int desiredMv);

// Lowest pressure this valve can actually reach (the table's first row), or
// VC_KPA_MIN when there is no table. Above the dead band this is not zero.
float calibMinKpa(int valve);

bool  calibHasTable(int valve);
void  calibPrint();          // the 'k' command: show every table
