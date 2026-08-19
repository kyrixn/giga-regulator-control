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
static const int   VC_GROUP_SIZE = 3;      // regulators per display page (2 pages)

// ---------------------------------------------------------------------------
// Regulator transfer functions (compact rig)
// ---------------------------------------------------------------------------
// COMMAND side: the regulator's 0-10V input maps linearly onto 0..900 kPa.
static const float VC_KPA_MIN   = 0.0f;
static const float VC_KPA_MAX   = 900.0f;
static const int   VC_CMD_MV_FS = 10000;   // command mV at VC_KPA_MAX

// FEEDBACK side: the regulator's pressure-monitor output is 1-5V across that
// SAME 0..900 kPa span. Note it is NOT the 0-10V the command side uses -- the
// two sides have different slopes, so never reuse one conversion for the other.
static const float VC_FB_V_MIN = 1.0f;     // volts read at VC_KPA_MIN
static const float VC_FB_V_MAX = 5.0f;     // volts read at VC_KPA_MAX

// Datasheet linearity is +/-1% F.S. = +/-9 kPa, so a reading is only good to
// about a kPa; the display deliberately does not chase digits below that.
static const float VC_KPA_ACCURACY = 9.0f;

// Touchscreen slider range (mV). The sliders map their travel onto this band;
// the DAC still accepts 0..VC_CMD_MV_FS mV, and serial is not limited by it.
//
// SAFETY: the band is chosen in PRESSURE and converted to mV, so it must be
// revisited whenever VC_KPA_MAX / VC_CMD_MV_FS change. At 0.09 kPa/mV the
// regulator's full 10000 mV would be 900 kPa; the slider deliberately reaches
// only ~100 kPa.
static const int VC_SB_MV_MIN = 0;         //   0 kPa
static const int VC_SB_MV_MAX = 1100;      //  99 kPa (~100 kPa)

// --- Read accessors (implemented in vc2.ino) -------------------------------
float vcMvToKpa(int mV);       // command mV -> output pressure (kPa)
float vcValueKpa(int valve);   // current setpoint as output pressure (kPa)
int   vcValueMv(int valve);    // current setpoint in mV (raw, voltage mode)
int   vcValveOn(int valve);    // 1 if the valve has a non-zero setpoint, else 0
int   vcActiveCount(void);     // number of valves currently on

// Measured (real-time) output pressure from the AD7606 analog inputs. Board 0
// channel N reads valve N's 1-5V monitor output (channels 0..5); only valves
// with a wired ADC channel have a reading (vcHasMeasure). A reading well below
// VC_KPA_MIN means the monitor line is dead (channel floating near 0V), not a
// vacuum -- the regulator cannot output below 1V in normal operation.
float vcMeasuredKpa(int valve);
bool  vcHasMeasure(int valve);

// --- Write / control (implemented in vc2.ino) ------------------------------
// Serial is never gated: a PC command always lands, and the touchscreen slider
// follows it because the slider renders vcValueMv() rather than a position of
// its own. The UI's LOCK gates the touchscreen only.
void  vcSetValveMv(int valve, int mV);  // drive a valve (touchscreen sliders)
void  vcEmergencyStop(void);            // all valves off + serial notice
