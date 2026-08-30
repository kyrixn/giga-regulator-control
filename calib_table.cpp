/**
 * calib_table.cpp  --  see calib_table.h
 */
#include "calib_table.h"

// ===========================================================================
// THE TABLES -- one per regulator, filled from a 'c<valve>,2200,3' sweep.
//
// Each row is { measured_kPa, commanded_mV } from the ASCENDING pass, sorted
// by ascending kPa. Leave a valve empty to drive it with the ideal mapping.
//
// The data below is a placeholder shape, not measurements: every table is
// empty, so this file is a no-op until real numbers go in. That is deliberate
// -- a wrong table is worse than none, because it looks like it is working.
// ===========================================================================

// V0 -- the healthy reference. Sweep c0,2200,3 on 2025 hardware, 2 bar supply.
// Hysteresis 1..8 kPa, repeatability <=5 kPa. Values are the midpoint of the
// min/max the three ascending passes reported.
//
// The last row is SUPPLY LIMITED: at 2 bar this valve stops climbing around
// 176 kPa, so commanding 2200mV buys almost nothing over 2000mV. Re-sweep this
// valve if the supply is raised -- the top of the curve will move, the rest
// will not.
static const CalibPoint TABLE_V0[] = {
  {   2.5f,    0 },
  {   7.5f,  200 },
  {  25.0f,  400 },
  {  44.0f,  600 },
  {  62.0f,  800 },
  {  79.0f, 1000 },
  { 104.0f, 1200 },
  { 122.0f, 1400 },
  { 139.5f, 1600 },
  { 157.0f, 1800 },
  { 172.5f, 2000 },
  { 176.5f, 2200 },   // supply limited, slope has collapsed to 0.2
};

static const CalibPoint TABLE_V1[] = {};

// V2 -- the sticky one. Sweep c2,2200,3, same session and supply as V0.
//
// Repeatability is excellent (<=2 kPa), so this table is accurate. What it
// CANNOT fix is 28 kPa of average hysteresis, seven times V0's. Every row here
// comes from the ASCENDING pass, so the table is only right when the pressure
// arrives from below. Approach a setpoint from above and the real pressure sits
// up to ~35 kPa high, table or no table.
//
// Note the 94.5 -> 132.5 step between 1400 and 1600 mV: local slope 2.1 where
// every other segment is 0.9-1.1. That is the spool breaking free of stiction,
// and interpolating across it is a guess -- setpoints between about 95 and
// 130 kPa on this valve should not be trusted until it is cleaned.
//
// The first row is the residual: at 0 mV this valve still holds 9.5 kPa,
// against V0's 2.5. It does not vent completely. Another symptom of the same
// fault, and the reason nothing below 9.5 kPa is reachable here.
static const CalibPoint TABLE_V2[] = {
  {   9.5f,    0 },   // residual -- does not vent below this
  {  10.0f,  400 },   // dead band edge; 0..400mV all sit at ~10 kPa
  {  23.0f,  600 },
  {  42.5f,  800 },
  {  61.0f, 1000 },
  {  78.0f, 1200 },
  {  94.5f, 1400 },
  { 132.5f, 1600 },   // stiction step: +38 kPa for +200 mV
  { 149.5f, 1800 },
  { 165.5f, 2000 },
  { 181.0f, 2200 },
};

static const CalibPoint TABLE_V3[] = {};
static const CalibPoint TABLE_V4[] = {};
static const CalibPoint TABLE_V5[] = {};

struct CalibTable {
  const CalibPoint *pts;
  int               n;
};

static const CalibTable TABLES[VC_NUM_VALVES] = {
  { TABLE_V0, (int)(sizeof(TABLE_V0) / sizeof(CalibPoint)) },
  { TABLE_V1, (int)(sizeof(TABLE_V1) / sizeof(CalibPoint)) },
  { TABLE_V2, (int)(sizeof(TABLE_V2) / sizeof(CalibPoint)) },
  { TABLE_V3, (int)(sizeof(TABLE_V3) / sizeof(CalibPoint)) },
  { TABLE_V4, (int)(sizeof(TABLE_V4) / sizeof(CalibPoint)) },
  { TABLE_V5, (int)(sizeof(TABLE_V5) / sizeof(CalibPoint)) },
};

bool calibHasTable(int valve) {
  if (valve < 0 || valve >= VC_NUM_VALVES) return false;
  return TABLES[valve].n >= 2;      // one row defines no slope
}

float calibMinKpa(int valve) {
  if (!calibHasTable(valve)) return VC_KPA_MIN;
  return TABLES[valve].pts[0].kPa;
}

int calibApply(int valve, int desiredMv) {
#if !CALIB_ENABLED
  return desiredMv;
#else
  if (!calibHasTable(valve)) return desiredMv;

  const CalibPoint *p = TABLES[valve].pts;
  const int n = TABLES[valve].n;

  // The caller's mV is the ideal command, so turn it back into the pressure it
  // was meant to produce and look THAT up.
  float want = vcMvToKpa(desiredMv);

  // Below the dead band there is nothing to interpolate towards: the valve
  // cannot go lower, so clamp to the first row rather than extrapolating into
  // a region where the measurements say the output does not move.
  if (want <= p[0].kPa)     return p[0].mV;
  if (want >= p[n - 1].kPa) return p[n - 1].mV;

  for (int i = 1; i < n; i++) {
    if (want > p[i].kPa) continue;
    float span = p[i].kPa - p[i - 1].kPa;
    // Guard a flat segment: two rows at the same pressure would divide by zero,
    // and a dead band recorded as several identical readings does exactly that.
    if (span <= 0.0001f) return p[i].mV;
    float f = (want - p[i - 1].kPa) / span;
    return p[i - 1].mV + (int)(f * (p[i].mV - p[i - 1].mV) + 0.5f);
  }
  return p[n - 1].mV;
#endif
}

void calibPrint() {
  Serial.println("=== Regulator calibration ===");
#if !CALIB_ENABLED
  Serial.println("CALIB_ENABLED is 0 -- every table bypassed, ideal mapping in use.");
  return;
#endif
  int have = 0;
  for (int v = 0; v < VC_NUM_VALVES; v++) {
    Serial.print("V");
    Serial.print(v);
    if (!calibHasTable(v)) {
      Serial.println(": no table (ideal mapping)");
      continue;
    }
    have++;
    const CalibTable &t = TABLES[v];
    Serial.print(": ");
    Serial.print(t.n);
    Serial.print(" points, reachable ");
    Serial.print(t.pts[0].kPa, 0);
    Serial.print("..");
    Serial.print(t.pts[t.n - 1].kPa, 0);
    Serial.println(" kPa");
    for (int i = 0; i < t.n; i++) {
      Serial.print("    ");
      Serial.print(t.pts[i].kPa, 1);
      Serial.print(" kPa <- ");
      Serial.print(t.pts[i].mV);
      Serial.println(" mV");
    }
  }
  if (have == 0)
    Serial.println("No tables filled in. Run 'c<valve>,2200,3' and see calib_table.cpp.");
}
