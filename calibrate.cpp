/**
 * calibrate.cpp  --  see calibrate.h
 */
#include "calibrate.h"
#include "valve_core.h"
#include "approach.h"

static bool     g_run     = false;
static int      g_valve   = 0;
static int      g_maxMv   = CAL_DEFAULT_MAX_MV;
static int      g_point   = 0;
static uint32_t g_pointMs = 0;      // when the current dwell started
static double   g_sum     = 0;      // running mean of the sample window
static int      g_n       = 0;
static bool     g_down    = false;  // second pass, stepping back down
static float    g_up[CAL_POINTS];   // this cycle's ascending readings, kept to
                                    // pair with the descending ones
static int      g_cycles  = 1;      // requested up-down cycles
static int      g_cycle   = 0;
// Spread of the ascending reading at each point ACROSS cycles. This is the
// repeatability, and it is the floor on what any calibration table can achieve:
// no table corrects an error that is different next time the same command is
// given. Measured 62/68/66 kPa at one command on this rig, so it is not small.
static float    g_min[CAL_POINTS];
static float    g_max[CAL_POINTS];

static int mvAt(int point) {
  // Both ends inclusive: point 0 is 0mV, point CAL_POINTS-1 is the ceiling.
  return (int)((long)g_maxMv * point / (CAL_POINTS - 1));
}

namespace cal {

static void printSummary() {
  Serial.println("--- repeatability of the ascending pass ---");
  Serial.println(" cmd_kPa    min    max  spread");
  float worst = 0;
  for (int i = 0; i < CAL_POINTS; i++) {
    if (g_max[i] < g_min[i]) continue;              // never sampled
    float sp = g_max[i] - g_min[i];
    if (sp > worst) worst = sp;
    char buf[64];
    snprintf(buf, sizeof(buf), "%8d %6d %6d  %6d",
             (int)(vcMvToKpa(mvAt(i)) + 0.5f),
             (int)(g_min[i] + 0.5f), (int)(g_max[i] + 0.5f), (int)(sp + 0.5f));
    Serial.println(buf);
  }
  Serial.print("worst spread ");
  Serial.print((int)(worst + 0.5f));
  Serial.println(" kPa -- a calibration table cannot do better than half this.");
}

void start(int valve, int maxMv, int cycles) {
  if (valve < 0 || valve >= VC_NUM_VALVES) {
    Serial.print("ERROR: invalid valve ");
    Serial.println(valve);
    return;
  }
  if (maxMv <= 0) maxMv = CAL_DEFAULT_MAX_MV;
  if (maxMv > VC_CMD_MV_FS) maxMv = VC_CMD_MV_FS;

  if (cycles < 1) cycles = 1;
  if (cycles > 9) cycles = 9;

  g_valve  = valve;
  g_maxMv  = maxMv;
  g_cycles = cycles;
  g_cycle  = 0;
  g_point = 0;
  g_run   = true;
  g_down  = false;
  g_sum   = 0;
  g_n     = 0;
  for (int i = 0; i < CAL_POINTS; i++) {
    g_up[i]  = 0.0f;
    g_min[i] =  1e9f;      // inverted so the first sample sets both
    g_max[i] = -1e9f;
  }

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
  Serial.print("Up then down x");
  Serial.print(g_cycles);
  Serial.print(", about ");
  Serial.print((2 * CAL_POINTS * CAL_SETTLE_MS * g_cycles) / 1000);
  Serial.println("s. 's' or E-STOP aborts.");
  Serial.println("  dir     mV  cmd_kPa  meas_kPa    err   hyst");

  // The sweep must see the valve's own behaviour. Leaving the undershoot trick
  // on would hide exactly the hysteresis this is here to measure.
  apr::setEnabled(false);
  vcSetValveMv(g_valve, mvAt(0));
  g_pointMs = millis();
}

void abort() {
  if (!g_run) return;
  g_run = false;
  vcSetValveMv(g_valve, 0);
  apr::setEnabled(APPROACH_ENABLED != 0);
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
    if (got < g_min[g_point]) g_min[g_point] = got;
    if (got > g_max[g_point]) g_max[g_point] = got;
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
      g_cycle++;
      if (g_cycle >= g_cycles) {
        g_run = false;
        vcSetValveMv(g_valve, 0);
        apr::setEnabled(APPROACH_ENABLED != 0);
        if (g_cycles > 1) printSummary();
        Serial.println("=== Sweep done, valve released ===");
        return;
      }
      Serial.print("--- cycle ");
      Serial.print(g_cycle + 1);
      Serial.print(" of ");
      Serial.print(g_cycles);
      Serial.println(" ---");
      g_down  = false;
      g_point = 0;
    }
  }

  vcSetValveMv(g_valve, mvAt(g_point));
  g_pointMs = millis();
}

}  // namespace cal
