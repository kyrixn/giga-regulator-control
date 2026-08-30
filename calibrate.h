/**
 * calibrate.h
 *
 * Automated command-vs-measured sweep for one regulator, so a calibration
 * curve is a serial command rather than an afternoon with a notebook.
 *
 * Steps the commanded voltage from 0 to a ceiling, waits for the pneumatics to
 * settle at each point, averages the AD7606 reading, and prints a table of
 * commanded kPa, measured kPa and the error between them.
 *
 * Design contract (mirrors adc, enc and ui): cal::tick() is cooperative and
 * non-blocking. A sweep takes tens of seconds of wall-clock and MUST NOT be a
 * blocking loop -- the touchscreen E-STOP has to stay live while a regulator is
 * being driven up its range, and serial has to keep answering.
 */
#pragma once
#include <Arduino.h>

// Points per sweep, including both ends.
#define CAL_POINTS 12

// Time held at each point before sampling. Pneumatic settling dominates:
// the regulator itself is fast, the volume downstream is not.
#define CAL_SETTLE_MS 1500

// Averaging window at the end of each dwell, to take the noise off the reading.
#define CAL_SAMPLE_MS 300

// Default ceiling (mV) when the command does not give one. Matches
// MAX_INPUT_VALUE in vc2_control.py -- about 198 kPa.
#define CAL_DEFAULT_MAX_MV 2200

namespace cal {
  void start(int valve, int maxMv);  // begin a sweep; clamps to the DAC range
  void abort();                      // stop and release the valve
  bool running();
  void tick();                       // non-blocking; call every loop()
}
