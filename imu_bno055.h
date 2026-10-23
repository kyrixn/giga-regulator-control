/**
 * imu_bno055.h
 *
 * Up to two BNO055 9-DOF IMUs (on-chip fusion, NDOF mode) on Wire1, streamed
 * to the PC.
 *
 * Bus: Wire1 (SDA1/SCL1), shared with the display shield's GT911 touch
 * controller at 0x5D. A BNO055 answers at 0x28 (ADR low, breakout default) or
 * 0x29 (ADR high), so two fit on the bus: the hand IMU at 0x28 and a second one
 * with its ADR pin tied to 3V3. Either may be absent; both are looked for at
 * boot and again by 'i1' / 'i'. They are kept off the DAC buses on purpose: a
 * BNO055 that hangs its bus (SDA held low after a brown-out) must never block a
 * valve write or the E-STOP. ui::begin() brings Wire1 up (400 kHz, polled from
 * loop() only), so call imu::begin() after it and never call Wire1.begin()
 * again.
 *
 * Stream ('i1' on, 'i0' off; off at boot), one frame per IMU per
 * 1/IMU_STREAM_HZ s on the Giga's clock, as one line of fixed-width uppercase
 * hex:
 *   T SSSS TTTTTTTT QW QX QY QZ GX GY GZ AX AY AZ CC     (no spaces)
 *   tag ('I' = the BNO055 at 0x28, 'J' = the one at 0x29), seq (16-bit, per
 *   IMU, counts every scheduled frame, so a gap = a dropped frame), millis() at
 *   the read, the raw int16 registers -- quaternion (1/16384), gyro (1/16
 *   deg/s), linear accel (1/100 m/s^2, gravity removed) -- and CALIB_STAT
 *   (sys<<6 | gyr<<4 | acc<<2 | mag).
 * With only the 0x28 IMU connected the stream is exactly the single-IMU one; a
 * PC that predates the 'J' tag ignores the second IMU's frames.
 * 56 bytes with the newline = one USB packet, handed over with send_nb(), which
 * takes it whole or not at all and never waits. If the PC stops reading
 * (script paused, PC asleep) frames are dropped instead of stalling loop(), so
 * touch, the E-STOP and serial commands keep working. 'i' prints one reading
 * per IMU in decimal for a human.
 *
 * Design contract (mirrors adc/ui): tick() is cooperative and non-blocking;
 * its only wait is the I2C burst read (34 B, ~1 ms at 400 kHz) per IMU per
 * frame.
 */
#pragma once
#include <Arduino.h>

// Stream rate. 40-50 Hz is the target band; the BNO055 fuses at 100 Hz, so
// every frame carries a sample at most 10 ms old.
#define IMU_STREAM_HZ 45

namespace imu {
  void begin();          // probe + start fusion; reports found/absent on Serial
  void tick();           // non-blocking; sends each IMU's frame when one is due
  void stream(bool on);  // 'i1' / 'i0' (i1 first looks for any IMU not found yet)
  void printOne();       // 'i': one reading per IMU, decimal, plus its counters
}
