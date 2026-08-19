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
 * Signals are 0-10V, measured on the AD7606's bipolar +/-10V range, so the raw
 * code lands in 0..32767 and volts() = raw * 10 / 32768.
 */
#pragma once
#include <Arduino.h>

// Boards wired. Compact build uses board 0 only (CS D41, BUSY D45); the driver
// still clocks out all 8 of its channels, of which 0..5 map to V0..V5.
#define ADC_NUM_BOARDS   1
#define ADC_CH_PER_BOARD 8
#define ADC_NUM_CH       (ADC_NUM_BOARDS * ADC_CH_PER_BOARD)

// Background sample rate (Hz): tick() re-samples every board at this cadence.
#define ADC_SAMPLE_HZ    200

namespace adc {
  void     begin();            // init SPI + pins, reset the AD7606(s), probe presence
  void     tick();             // non-blocking; acquires a sweep when the timer is due
  int16_t  raw(int ch);        // latest signed raw code, channel 0..ADC_NUM_CH-1
  float    volts(int ch);      // latest voltage (0-10V range) for a channel
  uint32_t seq();              // increments once per completed sweep
  bool     present(int board); // BUSY toggled during begin()'s probe
  void     printAll();         // dump every channel to Serial (the 'a' command)
}
