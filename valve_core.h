/**
 * valve_core.h
 *
 * Minimal read-only interface the on-Giga display UI (ui_display.cpp) uses to
 * observe valve state. Implemented in vc2.ino.
 *
 * The UI NEVER touches the serial link or the DAC buses -- it only reads state
 * through these accessors and triggers an emergency stop. That keeps the
 * display from interfering with PC<->Giga communication timing.
 */
#pragma once

// Layout constants. Kept in sync with vc2.ino via a static_assert there.
static const int   VC_NUM_VALVES = 32;
static const int   VC_GROUP_SIZE = 8;      // valves shown per display window
static const int   VC_BANK_SPLIT = 16;     // V0..15 -> Wire, V16..31 -> Wire2

// Output-pressure range the regulators map 0-10V onto (kPa).
static const float VC_KPA_MIN = -100.0f;
static const float VC_KPA_MAX =  500.0f;

// --- Accessors implemented in vc2.ino --------------------------------------
float vcValueKpa(int valve);   // current setpoint as output pressure (kPa)
int   vcValveOn(int valve);    // 1 if the valve has a non-zero setpoint, else 0
int   vcActiveCount(void);     // number of valves currently on
void  vcEmergencyStop(void);   // UI-triggered: all valves off + serial notice
