/**
 * approach.cpp  --  see approach.h
 */
#include "approach.h"
#include "valve_core.h"

static apr::WriteFn g_write  = NULL;
static bool         g_on     = (APPROACH_ENABLED != 0);

static int      g_target[VC_NUM_VALVES];   // where we are heading
static int      g_applied[VC_NUM_VALVES];  // what the DAC was last given
static bool     g_dipping[VC_NUM_VALVES];  // sitting at the undershoot
static uint32_t g_dipMs[VC_NUM_VALVES];

namespace apr {

void begin(WriteFn fn) {
  g_write = fn;
  for (int v = 0; v < VC_NUM_VALVES; v++) {
    g_target[v]  = 0;
    g_applied[v] = 0;
    g_dipping[v] = false;
    g_dipMs[v]   = 0;
  }
}

void setEnabled(bool on) {
  g_on = on;
  if (!on) {
    // Leaving the trick mid-dip would strand a valve below its setpoint.
    for (int v = 0; v < VC_NUM_VALVES; v++) {
      if (g_dipping[v]) {
        g_dipping[v] = false;
        g_applied[v] = g_target[v];
        if (g_write) g_write(v, g_target[v]);
      }
    }
  }
}

bool enabled() { return g_on; }

void force(int valve, int mV) {
  if (valve < 0 || valve >= VC_NUM_VALVES) return;
  g_target[valve]  = mV;
  g_applied[valve] = mV;
  g_dipping[valve] = false;
  if (g_write) g_write(valve, mV);
}

void request(int valve, int mV) {
  if (valve < 0 || valve >= VC_NUM_VALVES) return;
  if (!g_on) { force(valve, mV); return; }

  g_target[valve] = mV;

  // Rising into the setpoint is already the direction the table was taken in.
  // Note this compares against what the DAC currently holds, not against the
  // previous target: mid-dip the valve really is low, so a new request above
  // that can go straight out.
  if (mV >= g_applied[valve]) {
    g_applied[valve] = mV;
    g_dipping[valve] = false;
    if (g_write) g_write(valve, mV);
    return;
  }

  int dip = mV - APPROACH_UNDERSHOOT_MV;
  if (dip < 0) dip = 0;
  g_applied[valve] = dip;
  g_dipping[valve] = true;
  g_dipMs[valve]   = millis();
  if (g_write) g_write(valve, dip);
}

void tick() {
  if (!g_on) return;
  uint32_t now = millis();
  for (int v = 0; v < VC_NUM_VALVES; v++) {
    if (!g_dipping[v]) continue;
    if ((uint32_t)(now - g_dipMs[v]) < APPROACH_MS) continue;
    g_dipping[v] = false;
    g_applied[v] = g_target[v];
    if (g_write) g_write(v, g_target[v]);
  }
}

}  // namespace apr
