/**
 * imu_bno055.cpp  --  see imu_bno055.h
 *
 * Register-level (Wire only, no library), page 0 throughout. One burst read of
 * 0x14..0x35 per frame covers gyro, Euler, quaternion, linear accel, gravity,
 * temperature and CALIB_STAT; the frame uses gyro, quaternion, linear accel and
 * calibration.
 *
 * Non-blocking USB: `Serial` on the GIGA is a wrapper that forwards to
 * `_SerialUSB` (arduino::USBSerial, a USBCDC). Its write() waits until the host
 * takes the data -- forever, if the host has the port open but is not reading.
 * USBCDC::send_nb() instead copies what fits into the endpoint buffer (64 B at
 * full speed) and returns at once; it takes nothing while a transfer is in
 * flight or the port is closed. A frame is 56 B, so it goes out whole or waits
 * in g_frame for the next try.
 */
#include "imu_bno055.h"
#include <Wire.h>
#include "USB/PluggableUSBSerial.h"   // _SerialUSB, for send_nb()

#define IMU_WIRE Wire1

// ---- BNO055 registers (page 0) ---------------------------------------------
static const uint8_t REG_CHIP_ID    = 0x00;   // reads 0xA0
static const uint8_t REG_PAGE_ID    = 0x07;
static const uint8_t REG_BURST      = 0x14;   // GYR_DATA_X_LSB: start of the burst
static const uint8_t REG_CALIB_STAT = 0x35;   // end of the burst
static const uint8_t REG_UNIT_SEL   = 0x3B;
static const uint8_t REG_OPR_MODE   = 0x3D;
static const uint8_t REG_PWR_MODE   = 0x3E;

static const uint8_t CHIP_ID      = 0xA0;
static const uint8_t MODE_CONFIG  = 0x00;
static const uint8_t MODE_NDOF    = 0x0C;
static const uint8_t UNITS        = 0x80;     // reset default: m/s^2, deg/s, deg, degC

// Offsets into the burst (little-endian int16 x/y/z, quaternion w/x/y/z).
static const uint8_t BURST_LEN = REG_CALIB_STAT - REG_BURST + 1;   // 34
static const uint8_t OFF_GYR   = 0x14 - REG_BURST;
static const uint8_t OFF_QUA   = 0x20 - REG_BURST;
static const uint8_t OFF_LIA   = 0x28 - REG_BURST;
static const uint8_t OFF_CAL   = 0x35 - REG_BURST;

static const uint32_t PERIOD_US       = 1000000UL / IMU_STREAM_HZ;
static const uint32_t RETRY_US        = 1000000UL;  // after a failed read: don't hammer a dead bus
static const uint32_t BOOT_PROBE_MS   = 800;        // the chip answers ~650 ms after power-on
static const uint32_t RETRY_PROBE_MS  = 30;         // 'i1' / 'i' when it was absent

// ---- State (main loop only; no ISRs) ---------------------------------------
static uint8_t  g_addr   = 0;          // 0x28 / 0x29, 0 = not found
static bool     g_stream = false;
static uint32_t g_next   = 0;          // micros() deadline of the next frame
static uint16_t g_seq    = 0;
static uint32_t g_sent = 0, g_dropped = 0, g_readErrors = 0;

static char     g_frame[64];
static uint32_t g_pendOff = 0;         // bytes of g_frame already handed to USB
static uint32_t g_pendLen = 0;         // bytes still to hand over (0 = none)

// ---- Register helpers ------------------------------------------------------
static bool write8(uint8_t addr, uint8_t reg, uint8_t val) {
  IMU_WIRE.beginTransmission(addr);
  IMU_WIRE.write(reg);
  IMU_WIRE.write(val);
  return IMU_WIRE.endTransmission() == 0;
}

static bool readLen(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  IMU_WIRE.beginTransmission(addr);
  IMU_WIRE.write(reg);
  if (IMU_WIRE.endTransmission(false) != 0) return false;   // repeated start
  if (IMU_WIRE.requestFrom(addr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = IMU_WIRE.read();
  return true;
}

// Poll both addresses until one returns the chip ID, or the deadline passes.
// PAGE_ID is set first: a Giga reset leaves the BNO055 powered, page unknown.
static uint8_t probe(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  for (;;) {
    for (uint8_t a = 0x28; a <= 0x29; a++) {
      uint8_t id = 0;
      write8(a, REG_PAGE_ID, 0);
      if (readLen(a, REG_CHIP_ID, &id, 1) && id == CHIP_ID) return a;
    }
    if (millis() - t0 >= timeoutMs) return 0;
    delay(10);
  }
}

// CONFIG -> normal power, fixed units -> NDOF fusion. ~60 ms of mode-switch waits.
static bool configure(uint8_t a) {
  if (!write8(a, REG_OPR_MODE, MODE_CONFIG)) return false;
  delay(25);                                   // any mode -> CONFIG: 19 ms
  if (!write8(a, REG_PWR_MODE, 0x00)) return false;
  delay(10);
  if (!write8(a, REG_UNIT_SEL, UNITS)) return false;
  if (!write8(a, REG_OPR_MODE, MODE_NDOF)) return false;
  delay(20);                                   // CONFIG -> fusion: 7 ms
  return true;
}

static bool bringUp(uint32_t probeMs) {
  uint8_t a = probe(probeMs);
  g_addr = (a && configure(a)) ? a : 0;
  return g_addr != 0;
}

static void printWhere() {
  Serial.print("BNO055 0x");
  Serial.print(g_addr, HEX);
  Serial.print(" on Wire1");
}

// ---- Frame -----------------------------------------------------------------
static const char HEXDIG[] = "0123456789ABCDEF";

static char *putHex(char *p, uint32_t v, int digits) {
  for (int i = digits - 1; i >= 0; i--) { p[i] = HEXDIG[v & 0xF]; v >>= 4; }
  return p + digits;
}

static inline uint16_t le16(const uint8_t *b) {
  return (uint16_t)(b[0] | (b[1] << 8));
}

static void buildFrame(uint16_t seq, uint32_t ms, const uint8_t *b) {
  char *p = g_frame;
  *p++ = 'I';
  p = putHex(p, seq, 4);
  p = putHex(p, ms, 8);
  for (int i = 0; i < 4; i++) p = putHex(p, le16(b + OFF_QUA + 2 * i), 4);
  for (int i = 0; i < 3; i++) p = putHex(p, le16(b + OFF_GYR + 2 * i), 4);
  for (int i = 0; i < 3; i++) p = putHex(p, le16(b + OFF_LIA + 2 * i), 4);
  p = putHex(p, b[OFF_CAL], 2);
  *p++ = '\n';
  g_pendOff = 0;
  g_pendLen = p - g_frame;                     // 56
}

// Hand the rest of the frame to USB without waiting.
static void flushFrame() {
  uint32_t actual = 0;
  _SerialUSB.send_nb((uint8_t *)g_frame + g_pendOff, g_pendLen, &actual, true);
  g_pendOff += actual;
  g_pendLen -= actual;
  if (actual && g_pendLen == 0) g_sent++;
}

namespace imu {

void begin() {
  Serial.print("IMU: ");
  if (bringUp(BOOT_PROBE_MS)) {
    printWhere();
    Serial.println(" - i1/i0 = stream on/off, i = one reading");
  } else {
    Serial.println("no BNO055 on Wire1 (0x28/0x29) - i1 retries");
  }
}

void tick() {
  if (!g_stream) return;
  if (g_pendLen) flushFrame();                 // retry until the endpoint is free

  uint32_t now = micros();
  if ((int32_t)(now - g_next) < 0) return;

  if (g_pendLen) {                             // last frame never got out: PC not reading
    g_dropped++;
    if (g_pendOff == 0) g_pendLen = 0;         // drop it whole, never wait
  }
  g_next += PERIOD_US;
  if ((int32_t)(now - g_next) > 0) g_next = now + PERIOD_US;   // >1 period late: resync, no burst

  uint16_t seq = g_seq++;
  if (g_pendLen) return;                       // half a frame out (not expected): finish it first

  uint8_t b[BURST_LEN];
  if (!readLen(g_addr, REG_BURST, b, BURST_LEN)) {
    g_readErrors++;
    g_next = now + RETRY_US;
    return;
  }
  buildFrame(seq, millis(), b);
  flushFrame();
}

void stream(bool on) {
  if (on && !g_addr && !bringUp(RETRY_PROBE_MS)) {
    Serial.println("IMU ERROR: no BNO055 on Wire1 (0x28/0x29)");
    return;
  }
  if (on) {
    g_sent = g_dropped = g_readErrors = 0;
    g_pendLen = 0;
    g_next = micros();
    g_stream = true;
    Serial.print("IMU: stream on, ");
    Serial.print(IMU_STREAM_HZ);
    Serial.print(" Hz, ");
    printWhere();
    Serial.println();
  } else {
    g_stream = false;
    g_pendLen = 0;
    Serial.print("IMU: stream off (sent ");
    Serial.print(g_sent);
    Serial.print(", dropped ");
    Serial.print(g_dropped);
    Serial.print(", read errors ");
    Serial.print(g_readErrors);
    Serial.println(")");
  }
}

void printOne() {
  uint8_t b[BURST_LEN];
  if ((!g_addr && !bringUp(RETRY_PROBE_MS)) || !readLen(g_addr, REG_BURST, b, BURST_LEN)) {
    Serial.println("IMU ERROR: no reading from a BNO055 on Wire1 (0x28/0x29)");
    return;
  }
  Serial.print("IMU: q=(");
  for (int i = 0; i < 4; i++) {
    if (i) Serial.print(",");
    Serial.print((int16_t)le16(b + OFF_QUA + 2 * i) / 16384.0f, 4);
  }
  Serial.print(") gyro=(");
  for (int i = 0; i < 3; i++) {
    if (i) Serial.print(",");
    Serial.print((int16_t)le16(b + OFF_GYR + 2 * i) / 16.0f, 2);
  }
  Serial.print(") deg/s lin_acc=(");
  for (int i = 0; i < 3; i++) {
    if (i) Serial.print(",");
    Serial.print((int16_t)le16(b + OFF_LIA + 2 * i) / 100.0f, 2);
  }
  uint8_t c = b[OFF_CAL];
  Serial.print(") m/s^2 cal=S");
  Serial.print(c >> 6 & 3);
  Serial.print("G");
  Serial.print(c >> 4 & 3);
  Serial.print("A");
  Serial.print(c >> 2 & 3);
  Serial.print("M");
  Serial.print(c & 3);
  Serial.print(" | stream ");
  Serial.print(g_stream ? "on" : "off");
  Serial.print(", sent ");
  Serial.print(g_sent);
  Serial.print(", dropped ");
  Serial.print(g_dropped);
  Serial.print(", read errors ");
  Serial.println(g_readErrors);
}

} // namespace imu
