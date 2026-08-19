/**
 * ui_display.h
 *
 * On-Giga touchscreen UI for the compact 6-valve controller (Arduino GIGA
 * Display Shield, 800x480 landscape). A single page shows the pressure (kPa)
 * of all 6 regulators, with a MODE button and an always-visible E-STOP.
 *
 * Design contract: ui::tick() is cooperative and non-blocking -- it returns
 * immediately unless a millis()-gated timer is due, and even then does only a
 * few small draws. Call it AFTER processSerialCommand() every loop so serial
 * always has priority. The UI reads valve state through valve_core.h and never
 * touches the serial port (except the E-STOP, which emits the same notice line
 * the 's' command does, to keep the PC in sync).
 *
 * Touch (GT911) lives on Wire1; the DACs are on Wire, so there is no bus or
 * address contention.
 */
#pragma once

namespace ui {
  void begin();   // init display + touch and paint the initial screen
  void tick();    // non-blocking; call every loop() iteration
}
