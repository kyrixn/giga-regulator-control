/**
 * calibrate.cpp  --  see calibrate.h
 */
#include "calibrate.h"
#include "valve_core.h"

static bool     g_run     = false;
static int      g_valve   = 0;
static int      g_maxMv   = CAL_DEFAULT_MAX_MV;
static int      g_point   = 0;
static uint32_t g_pointMs = 0;      // when the current dwell started
static double   g_sum     = 0;      // running mean of the sample window
static int      g_n       = 0;
static bool     g_down    = false;  // second pass, stepping back down
static float    g_up[CAL_POINTS];   // ascending readings, kept to pair with
                                    // the descending ones and get hysteresis

static int mvAt(int point) {
  // Both ends inclusive: point 0 is 0mV, point CAL_POINTS-1 is the ceiling.
  return (int)((long)g_maxMv * point / (CAL_POINTS - 1));
}

namespace cal {

void start(int valve, int maxMv) {
  if (valve < 0 || valve >= VC_NUM_VALVES) {
    Serial.print("ERROR: invalid valve ");
    Serial.println(valve);
    return;
  }
  if (maxMv <= 0) maxMv = CAL_DEFAULT_MAX_MV;
  if (maxMv > VC_CMD_MV_FS) maxMv = VC_CMD_MV_FS;

  g_valve = valve;
  g_maxMv = maxMv;
  g_point = 0;
  g_run   = true;
  g_down  = false;
  g_sum   = 0;
  g_n     = 0;
  for (int i = 0; i < CAL_POINTS; i++) g_up[i] = 0.0f;

  Serial.print("=== Calibration sweep: V");
  Serial.print(g_valve);
  Serial.print("  0..");
  Serial.print(g_maxMv);
  Serial.print(" mV (0..");
  Serial.print(vcMvToKpa(g_maxMv), 0);
  Serial.print(" kPa) in ");
  Serial.print(CAL_POINTS);
  Serial.println(" points ===");
  // Up then down. A single ascending sweep cannot tell a fixed offset from
  // hysteresis, and that distinction decides whether a calibration table can
  // work at all: no table can correct an error that depends on which direction
  // the pressure arrived from.
  Serial.print("Up then down, about ");
  Serial.print((2 * CAL_POINTS * CAL_SETTLE_MS) / 1000);
  Serial.println("s. 's' or E-STOP aborts.");
  Serial.println("  dir     mV  cmd_kPa  meas_kPa    err   hyst");

  vcSetValveMv(g_valve, mvAt(0));
  g_pointMs = millis();
}

void abort() {
  if (!g_run) return;
  g_run = false;
  vcSetValveMv(g_valve, 0);
  Serial.println("=== Calibration aborted ===");
}

bool running() { return g_run; }

void tick() {
  if (!g_run) return;

  uint32_t elapsed = (uint32_t)(millis() - g_pointMs);

  // Average only over the tail of the dwell, once the pressure has settled.
  if (elapsed >= CAL_SETTLE_MS - CAL_SAMPLE_MS && elapsed < CAL_SETTLE_MS) {
    g_sum += vcMeasuredKpa(g_valve);
    g_n++;
    return;
  }
  if (elapsed < CAL_SETTLE_MS) return;

  int   mv   = mvAt(g_point);
  float want = vcMvToKpa(mv);
  float got  = g_n ? (float)(g_sum / g_n) : vcMeasuredKpa(g_valve);

  char buf[72];
  if (!g_down) {
    g_up[g_point] = got;
    snprintf(buf, sizeof(buf), "  up   %5d  %7d  %8d  %+5d      -",
             mv, (int)(want + 0.5f), (int)(got + 0.5f), (int)(got - want));
  } else {
    snprintf(buf, sizeof(buf), "  down %5d  %7d  %8d  %+5d  %+5d",
             mv, (int)(want + 0.5f), (int)(got + 0.5f), (int)(got - want),
             (int)(got - g_up[g_point]));
  }
  Serial.println(buf);

  g_sum = 0;
  g_n   = 0;

  if (!g_down) {
    g_point++;
    if (g_point >= CAL_POINTS) {          // turn around, do not repeat the top
      g_down  = true;
      g_point = CAL_POINTS - 2;
    }
  } else {
    g_point--;
    if (g_point < 0) {
      g_run = false;
      vcSetValveMv(g_valve, 0);
      Serial.println("=== Sweep done, valve released ===");
      return;
    }
  }

  vcSetValveMv(g_valve, mvAt(g_point));
  g_pointMs = millis();
}

}  // namespace cal
