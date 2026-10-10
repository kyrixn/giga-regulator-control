/**
 * imu_bno055.h
 *
 * BNO055 9-DOF IMU (on-chip fusion, NDOF mode) on Wire1, streamed to the PC.
 *
 * Bus: Wire1 (SDA1/SCL1), shared with the display shield's GT911 touch
 * controller at 0x5D. The BNO055 answers at 0x28 (ADR low, breakout default)
 * or 0x29 (ADR high); the probe tries both. It is kept off the DAC buses on
 * purpose: a BNO055 that hangs its bus (SDA held low after a brown-out) must
 * never block a valve write or the E-STOP -- at worst it costs the touch.
 * ui::begin() brings Wire1 up (400 kHz, polled from loop() only), so call
 * imu::begin() after it and never call Wire1.begin() again.
 *
 * Stream ('i1' on, 'i0' off; off at boot), one frame per 1/IMU_STREAM_HZ s on
 * the Giga's clock, as one line of fixed-width uppercase hex:
 *   I SSSS TTTTTTTT QW QX QY QZ GX GY GZ AX AY AZ CC     (no spaces)
 *   seq (16-bit, counts every scheduled frame, so a gap = a dropped frame),
 *   millis() at the read, the raw int16 registers -- quaternion (1/16384),
 *   gyro (1/16 deg/s), linear accel (1/100 m/s^2, gravity removed) -- and
 *   CALIB_STAT (sys<<6 | gyr<<4 | acc<<2 | mag).
 * 56 bytes with the newline = one USB packet, handed over with send_nb(), which
 * takes it whole or not at all and never waits. If the PC stops reading
 * (script paused, PC asleep) frames are dropped instead of stalling loop(), so
 * touch, the E-STOP and serial commands keep working. 'i' prints one reading
 * in decimal for a human.
 *
 * Design contract (mirrors adc/ui): tick() is cooperative and non-blocking;
 * its only wait is the I2C burst read (34 B, ~1 ms at 400 kHz) once per frame.
 */
#pragma once
#include <Arduino.h>

// Stream rate. 40-50 Hz is the target band; the BNO055 fuses at 100 Hz, so
// every frame carries a sample at most 10 ms old.
#define IMU_STREAM_HZ 45

namespace imu {
  void begin();          // probe + start fusion; reports found/absent on Serial
  void tick();           // non-blocking; sends a frame when one is due
  void stream(bool on);  // 'i1' / 'i0' (i1 re-probes first if absent)
  void printOne();       // 'i': one reading, decimal, plus the stream counters
}
