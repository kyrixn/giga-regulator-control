/**
 * bno055_test.ino
 *
 * Standalone smoke test for a BNO055 9-DOF IMU on the GIGA's third I2C bus.
 * No libraries needed beyond Wire; register-level, so a failure points at
 * wiring or the chip, never at a library version.
 *
 * OUTPUT FORMAT (one line per frame, ~10 Hz) — the shoulder GUI's IMU link
 * parses exactly this, do not reorder the fields:
 *   heading roll pitch | qw qx qy qz | gx gy gz | ax ay az | T | S. G. A. M.
 *
 * WIRING (as built on this station)
 *   BNO055 SDA -> D9   (silkscreen "9",  chip pin SDA2)
 *   BNO055 SCL -> D8   (silkscreen "8",  chip pin SCL2)
 *   VIN/3V3, GND       (GIGA is 3.3 V logic; a breakout with a regulator may
 *                       take 5 V on VIN, the I2C lines stay 3.3 V either way)
 *
 * PIN TRAP, third of the family: the Arduino object for D9/D8 is `Wire2`,
 * not Wire or Wire1 -- same silkscreen/object mismatch as TX1/RX1 = Serial2
 * (encoder_rs485.h) and the SPI header = SPI1 (adc_ad7606.cpp). Wire is the
 * DACs on D20/D21, Wire1 is the display's touch controller; this bus has the
 * IMU alone on it, so nothing can clash.
 *
 * PULL-UPS: unlike SDA/SCL and SDA1/SCL1, the GIGA has NO internal pull-ups
 * on SDA2/SCL2 (documented in the GIGA cheat sheet). The breakout's onboard
 * pull-ups must provide them -- Adafruit and the common GY modules carry 10k
 * to 3V3, a bare chip does not. If the scanner below finds no device, check
 * this first: both lines must idle at 3.3 V.
 *
 * BNO055 QUIRKS the code works around
 *   - Address is 0x28 with ADR/COM3 low (breakout default), 0x29 tied high.
 *     The sketch probes both.
 *   - ~650 ms boot after power-on or soft reset before the chip answers with
 *     its ID; polling earlier looks like a dead bus. The sketch polls with a
 *     deadline instead of a fixed delay.
 *   - The chip clock-stretches; stay at 100 kHz like the rest of the rig.
 *   - Registers live on two pages; everything used here is on PAGE 0, which
 *     is selected explicitly because a warm restart may leave page 1 active.
 *
 * WHAT "WORKING" LOOKS LIKE
 *   IDs print as A0/FB/32/F, then a 10 Hz stream of Euler angles, quaternion
 *   and calibration status. Fused heading is only trustworthy once the CAL
 *   digits reach S3 G3 A3 M3: gyro calibrates by holding still, accel by
 *   resting in ~6 different orientations, mag by waving a figure-8. Until the
 *   mag is in, yaw wanders -- pitch/roll are usable from gyro+acc alone.
 */

#include <Arduino.h>
#include <Wire.h>

#define BNO_WIRE      Wire2       // D9 = SDA2, D8 = SCL2 (see header)
#define BNO_I2C_HZ    100000

// --- BNO055 registers (page 0) ---------------------------------------------
static const uint8_t REG_CHIP_ID    = 0x00;  // reads 0xA0
static const uint8_t REG_ACC_ID     = 0x01;  // 0xFB
static const uint8_t REG_MAG_ID     = 0x02;  // 0x32
static const uint8_t REG_GYR_ID     = 0x03;  // 0x0F
static const uint8_t REG_PAGE_ID    = 0x07;
static const uint8_t REG_GYR_DATA   = 0x14;  // 6 B, int16, 16 LSB/(deg/s)
static const uint8_t REG_EUL_DATA   = 0x1A;  // 6 B, int16, 16 LSB/deg (heading, roll, pitch)
static const uint8_t REG_QUA_DATA   = 0x20;  // 8 B, int16, 1/16384 (w,x,y,z)
static const uint8_t REG_LIA_DATA   = 0x28;  // 6 B, int16, 100 LSB/(m/s^2), gravity removed
static const uint8_t REG_TEMP      = 0x34;
static const uint8_t REG_CALIB_STAT = 0x35;  // sys<<6 | gyr<<4 | acc<<2 | mag
static const uint8_t REG_SYS_STATUS = 0x39;  // 5 = fusion running
static const uint8_t REG_SYS_ERR    = 0x3A;
static const uint8_t REG_OPR_MODE   = 0x3D;
static const uint8_t REG_PWR_MODE   = 0x3E;
static const uint8_t REG_SYS_TRIGGER= 0x3F;  // 0x20 = soft reset

static const uint8_t MODE_CONFIG = 0x00;
static const uint8_t MODE_NDOF   = 0x0C;

static uint8_t g_addr = 0;        // filled in by probe: 0x28 or 0x29

// --- tiny register helpers; every call reports failure loudly --------------
static bool write8(uint8_t reg, uint8_t val) {
  BNO_WIRE.beginTransmission(g_addr);
  BNO_WIRE.write(reg);
  BNO_WIRE.write(val);
  return BNO_WIRE.endTransmission() == 0;
}

static bool readLen(uint8_t reg, uint8_t *buf, uint8_t n) {
  BNO_WIRE.beginTransmission(g_addr);
  BNO_WIRE.write(reg);
  if (BNO_WIRE.endTransmission(false) != 0) return false;   // repeated start
  if (BNO_WIRE.requestFrom(g_addr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = BNO_WIRE.read();
  return true;
}

static int read8(uint8_t reg) {                  // -1 on bus error
  uint8_t v;
  return readLen(reg, &v, 1) ? v : -1;
}

static int16_t le16(const uint8_t *p) { return (int16_t)(p[0] | (p[1] << 8)); }

// --- bring-up ---------------------------------------------------------------
static void scanBus() {
  Serial.print("scan Wire2:");
  uint8_t found = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    BNO_WIRE.beginTransmission(a);
    if (BNO_WIRE.endTransmission() == 0) {
      Serial.print(" 0x"); Serial.print(a, HEX);
      found++;
    }
  }
  if (!found) Serial.print(" nothing -- check pull-ups to 3V3 on D8/D9 "
                           "(this bus has none on the board) and the wiring");
  Serial.println();
}

static bool waitChipId(uint32_t deadline_ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < deadline_ms) {
    if (read8(REG_CHIP_ID) == 0xA0) return true;
    delay(20);
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}

  BNO_WIRE.begin();
  BNO_WIRE.setClock(BNO_I2C_HZ);

  scanBus();

  // Probe both possible addresses. Power-on boot takes ~650 ms, so give the
  // first probe a real deadline instead of failing on a fresh power-up.
  const uint8_t addrs[2] = { 0x28, 0x29 };
  for (int i = 0; i < 2; i++) {
    g_addr = addrs[i];
    if (waitChipId(1000)) break;
    g_addr = 0;
  }
  if (!g_addr) {
    Serial.println("BNO055 not found at 0x28/0x29 -- halting. (ADR pin picks "
                   "the address; breakout default is 0x28.)");
    while (true) delay(1000);
  }
  Serial.print("BNO055 at 0x"); Serial.println(g_addr, HEX);

  // Known-clean start: soft reset, then wait out the boot again.
  write8(REG_SYS_TRIGGER, 0x20);
  delay(700);
  if (!waitChipId(1000)) {
    Serial.println("no reply after soft reset -- halting.");
    while (true) delay(1000);
  }

  Serial.print("IDs chip/acc/mag/gyr = ");
  Serial.print(read8(REG_CHIP_ID), HEX); Serial.print('/');
  Serial.print(read8(REG_ACC_ID),  HEX); Serial.print('/');
  Serial.print(read8(REG_MAG_ID),  HEX); Serial.print('/');
  Serial.print(read8(REG_GYR_ID),  HEX);
  Serial.println("  (want A0/FB/32/F)");

  write8(REG_PAGE_ID, 0);              // warm restarts can leave page 1 active
  write8(REG_OPR_MODE, MODE_CONFIG);   // mode changes only legal from CONFIG
  delay(25);
  write8(REG_PWR_MODE, 0x00);          // normal power
  write8(REG_SYS_TRIGGER, 0x00);       // internal oscillator
  write8(REG_OPR_MODE, MODE_NDOF);     // 9-DOF fusion; datasheet: 7..19 ms
  delay(25);

  int st = read8(REG_SYS_STATUS);
  Serial.print("sys status = "); Serial.print(st);
  Serial.println(st == 5 ? " (fusion running)" : " (want 5; 1 = see sys err)");
  if (st == 1) { Serial.print("sys err = "); Serial.println(read8(REG_SYS_ERR)); }
  Serial.println("heading roll pitch [deg] | quat wxyz | gyro xyz [dps] | "
                 "linacc xyz [m/s^2] | T | CAL S G A M (3 = done)");
}

void loop() {
  uint8_t eul[6], qua[8], gyr[6], lia[6];
  bool ok = readLen(REG_EUL_DATA, eul, 6)
         && readLen(REG_QUA_DATA, qua, 8)
         && readLen(REG_GYR_DATA, gyr, 6)
         && readLen(REG_LIA_DATA, lia, 6);
  int cal = read8(REG_CALIB_STAT), temp = read8(REG_TEMP);
  if (!ok || cal < 0) {
    Serial.println("bus error mid-stream (loose wire?)");
    delay(500);
    return;
  }

  Serial.print(le16(eul)     / 16.0, 2); Serial.print(' ');   // heading 0..360
  Serial.print(le16(eul + 2) / 16.0, 2); Serial.print(' ');   // roll
  Serial.print(le16(eul + 4) / 16.0, 2); Serial.print(" | "); // pitch
  for (int i = 0; i < 4; i++) {
    Serial.print(le16(qua + 2 * i) / 16384.0, 4); Serial.print(' ');
  }
  Serial.print("| ");
  for (int i = 0; i < 3; i++) {
    Serial.print(le16(gyr + 2 * i) / 16.0, 1); Serial.print(' ');
  }
  Serial.print("| ");
  for (int i = 0; i < 3; i++) {
    Serial.print(le16(lia + 2 * i) / 100.0, 2); Serial.print(' ');
  }
  Serial.print("| "); Serial.print(temp);
  Serial.print(" | S"); Serial.print((cal >> 6) & 3);
  Serial.print(" G");   Serial.print((cal >> 4) & 3);
  Serial.print(" A");   Serial.print((cal >> 2) & 3);
  Serial.print(" M");   Serial.println(cal & 3);

  delay(100);                          // ~10 Hz; fusion updates at 100 Hz
}