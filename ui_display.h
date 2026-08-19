/**
 * ui_display.h
 *
 * On-Giga touchscreen UI for the compact 6-valve controller (Arduino GIGA
 * Display Shield, 800x480 landscape). Two pages of 3 regulators, chosen from
 * the top bar, which also carries LOCK and an always-visible E-STOP. Each row
 * shows a regulator's setpoint and live pressure on the left and the slider
 * that commands it on the right.
 *
 * LOCK gates the touchscreen only; serial commands are never gated and always
 * win, with the sliders redrawing to follow them.
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
