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
  g_sum   = 0;
  g_n     = 0;

  Serial.print("=== Calibration sweep: V");
  Serial.print(g_valve);
  Serial.print("  0..");
  Serial.print(g_maxMv);
  Serial.print(" mV (0..");
  Serial.print(vcMvToKpa(g_maxMv), 0);
  Serial.print(" kPa) in ");
  Serial.print(CAL_POINTS);
  Serial.println(" points ===");
  Serial.print("Takes about ");
  Serial.print((CAL_POINTS * CAL_SETTLE_MS) / 1000);
  Serial.println("s. 's' or E-STOP aborts.");
  Serial.println("  mV  cmd_kPa  meas_kPa    err");

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

  char buf[64];
  snprintf(buf, sizeof(buf), "%5d  %7d  %8d  %+5d",
           mv, (int)(want + 0.5f), (int)(got + 0.5f), (int)(got - want));
  Serial.println(buf);

  g_sum = 0;
  g_n   = 0;
  g_point++;

  if (g_point >= CAL_POINTS) {
    g_run = false;
    vcSetValveMv(g_valve, 0);
    Serial.println("=== Sweep done, valve released ===");
    return;
  }

  vcSetValveMv(g_valve, mvAt(g_point));
  g_pointMs = millis();
}

}  // namespace cal
