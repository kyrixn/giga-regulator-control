/**
 * valve_core.h
 *
 * Minimal read-only interface the on-Giga display UI (ui_display.cpp) uses to
 * observe valve state. Implemented in vc2.ino.  Compact build: 6 regulators.
 *
 * The UI NEVER touches the serial link or the DAC buses -- it only reads state
 * through these accessors and triggers an emergency stop. That keeps the
 * display from interfering with PC<->Giga communication timing.
 */
#pragma once

// Layout constants. Kept in sync with vc2.ino via a static_assert there.
static const int   VC_NUM_VALVES = 6;
static const int   VC_GROUP_SIZE = 6;      // all 6 fit in one display window

// Output-pressure range the regulators map 0-10V onto (kPa).
static const float VC_KPA_MIN = -100.0f;
static const float VC_KPA_MAX =  500.0f;

// Standalone slider control range (mV). The touchscreen sliders map their
// travel onto this band; the DAC still accepts 0..10000 mV.
static const int VC_SB_MV_MIN = 1600;
static const int VC_SB_MV_MAX = 3000;

// --- Read accessors (implemented in vc2.ino) -------------------------------
float vcValueKpa(int valve);   // current setpoint as output pressure (kPa)
int   vcValueMv(int valve);    // current setpoint in mV (raw, voltage mode)
int   vcValveOn(int valve);    // 1 if the valve has a non-zero setpoint, else 0
int   vcActiveCount(void);     // number of valves currently on

// Measured (real-time) output pressure from the AD7606 analog inputs. Board 0
// channel N reads valve N's analog output (channels 0..5); only valves with a
// wired ADC channel have a reading (vcHasMeasure). Same 0-10V -> kPa mapping
// as the DAC side.
float vcMeasuredKpa(int valve);
bool  vcHasMeasure(int valve);

// --- Write / control (implemented in vc2.ino) ------------------------------
void  vcSetValveMv(int valve, int mV);  // drive a valve (standalone sliders)
void  vcEmergencyStop(void);            // all valves off + serial notice
void  vcSetSerialIgnore(bool ignore);   // true = discard incoming serial cmds
