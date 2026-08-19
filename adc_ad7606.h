/**
 * adc_ad7606.h
 *
 * AD7606 analog-input acquisition for the compact 6-valve station (serial/SPI
 * mode).
 *
 * Each 8-channel AD7606 board shares one SPI bus (SCK + DOUTA), a common
 * CONVST and RST, and has a per-board CS and BUSY. Nothing else on the Giga
 * uses SPI (display = MIPI-DSI, touch = Wire1, DACs = Wire), so the ADCs own
 * the bus outright.  This build wires board 0 only, channels 0..5 -> V0..V5.
 *
 * Design contract (mirrors ui_display): adc::tick() is cooperative and
 * non-blocking -- it returns immediately unless its millis()-gated timer is due,
 * and even then a full sweep costs only ~50us/board. Call it AFTER
 * processSerialCommand() so serial always keeps priority.
 *
 * The regulators' pressure-monitor outputs are 1-5V, measured on the AD7606's
 * bipolar range selected by the module's RANGE ("RAGE") pin -- see ADC_RANGE_V.
 * The raw code is twos-complement, so a 1-5V signal lands well inside 0..32767
 * and volts() = raw * ADC_RANGE_V / 32768.
 */
#pragma once
#include <Arduino.h>

// Boards wired. Compact build uses board 0 only (CS D41, BUSY D45); the driver
// still clocks out all 8 of its channels, of which 0..5 map to V0..V5.
#define ADC_NUM_BOARDS   1
#define ADC_CH_PER_BOARD 8
#define ADC_NUM_CH       (ADC_NUM_BOARDS * ADC_CH_PER_BOARD)

// Full-scale of the bipolar input range, set in HARDWARE by the module's RANGE
// ("RAGE") pin -- there is no software control of it:
//   RANGE -> VIO/3.3V  =>  +/-10V  =>  ADC_RANGE_V 10.0f   (as wired today)
//   RANGE -> GND       =>  +/- 5V  =>  ADC_RANGE_V  5.0f
// A 1-5V monitor signal fits either way. Moving to +/-5V halves the quantisation
// step (0.069 -> 0.034 kPa), which is far below the regulator's own +/-9 kPa
// linearity, so it is optional. If you do rewire it, re-run the DMM sweep that
// produced ADC_V_GAIN / ADC_V_OFFSET in adc_ad7606.cpp -- those were measured
// on the +/-10V range.
#define ADC_RANGE_V      10.0f

// Background sample rate (Hz): tick() re-samples every board at this cadence.
#define ADC_SAMPLE_HZ    200

namespace adc {
  void     begin();            // init SPI + pins, reset the AD7606(s), probe presence
  void     tick();             // non-blocking; acquires a sweep when the timer is due
  int16_t  raw(int ch);        // latest signed raw code, channel 0..ADC_NUM_CH-1
  float    volts(int ch);      // latest voltage for a channel (see ADC_RANGE_V)
  uint32_t seq();              // increments once per completed sweep
  bool     present(int board); // BUSY toggled during begin()'s probe
  void     printAll();         // dump every channel to Serial (the 'a' command)
}
