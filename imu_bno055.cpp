/**
 * imu_bno055.cpp  --  see imu_bno055.h
 *
 * Register-level (Wire only, no library), page 0 throughout. One burst read of
 * 0x14..0x35 per frame covers gyro, Euler, quaternion, linear accel, gravity,
 * temperature and CALIB_STAT; the frame uses gyro, quaternion, linear accel and
 * calibration.
 *
 * Up to two BNO055s share Wire1, one per address. Each has its own frame
 * buffer, seq and counters; both are read off the same 45 Hz schedule.
 *
 * Non-blocking USB: `Serial` on the GIGA is a wrapper that forwards to
 * `_SerialUSB` (arduino::USBSerial, a USBCDC). Its write() waits until the host
 * takes the data -- forever, if the host has the port open but is not reading.
 * USBCDC::send_nb() instead copies what fits into the endpoint buffer (64 B at
 * full speed) and returns at once; it takes nothing while a transfer is in
 * flight or the port is closed. A frame is 56 B, so it goes out whole or waits
 * in its buffer for the next try. With two IMUs the second frame of a period
 * usually waits ~1 ms for the first one's packet to clear.
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
static const uint32_t RETRY_PROBE_MS  = 30;         // 'i1' / 'i' when none was found

// ---- Devices (main loop only; no ISRs) -------------------------------------
// The address fixes the frame tag, so a frame always names its chip whichever
// of them is connected: 0x28 (ADR low) streams 'I' frames, the single-IMU
// format; 0x29 (ADR high) streams 'J' frames.
struct Imu {
  uint8_t  addr;
  char     tag;
  bool     found;
  bool     backoff;                    // a read failed: no reads until retryAt
  uint32_t retryAt;                    // micros()
  uint16_t seq;
  uint32_t sent, dropped, readErrors;
  char     frame[64];
  uint32_t pendOff;                    // bytes of frame already handed to USB
  uint32_t pendLen;                    // bytes still to hand over (0 = none)
};
static Imu g_imu[] = { {0x28, 'I'}, {0x29, 'J'} };

static bool     g_stream = false;
static uint32_t g_next   = 0;          // micros() deadline of the next frames

static int foundCount() {
  int n = 0;
  for (const Imu &m : g_imu) n += m.found;
  return n;
}

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

// True if a BNO055 at `a` returns its chip ID. PAGE_ID is set first: a Giga
// reset leaves the BNO055 powered, page unknown.
static bool answers(uint8_t a) {
  uint8_t id = 0;
  write8(a, REG_PAGE_ID, 0);
  return readLen(a, REG_CHIP_ID, &id, 1) && id == CHIP_ID;
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

// Look for the IMUs not found yet and start fusion on each one that answers.
// Polls until the deadline only while none is found: chips on one supply boot
// together, so once one answers the other has had its try in the same pass
// (an absent 0x29 then costs one NACK, well under 1 ms). True if any is found.
static bool bringUp(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  for (;;) {
    for (Imu &m : g_imu) {
      if (!m.found && answers(m.addr)) m.found = configure(m.addr);
    }
    if (foundCount() || millis() - t0 >= timeoutMs) return foundCount() > 0;
    delay(10);
  }
}

// "BNO055 0x28 on Wire1", or "BNO055 0x28 + 0x29 on Wire1" with both.
static void printWhere() {
  Serial.print("BNO055");
  const char *sep = " 0x";
  for (const Imu &m : g_imu) {
    if (!m.found) continue;
    Serial.print(sep);
    Serial.print(m.addr, HEX);
    sep = " + 0x";
  }
  Serial.print(" on Wire1");
}

// "sent N, dropped N, read errors N"; prefixed "0x28 " when two IMUs report.
static void printCounters(const Imu &m, bool labelled) {
  if (labelled) {
    Serial.print("0x");
    Serial.print(m.addr, HEX);
    Serial.print(" ");
  }
  Serial.print("sent ");
  Serial.print(m.sent);
  Serial.print(", dropped ");
  Serial.print(m.dropped);
  Serial.print(", read errors ");
  Serial.print(m.readErrors);
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

static void buildFrame(Imu &m, uint16_t seq, uint32_t ms, const uint8_t *b) {
  char *p = m.frame;
  *p++ = m.tag;
  p = putHex(p, seq, 4);
  p = putHex(p, ms, 8);
  for (int i = 0; i < 4; i++) p = putHex(p, le16(b + OFF_QUA + 2 * i), 4);
  for (int i = 0; i < 3; i++) p = putHex(p, le16(b + OFF_GYR + 2 * i), 4);
  for (int i = 0; i < 3; i++) p = putHex(p, le16(b + OFF_LIA + 2 * i), 4);
  p = putHex(p, b[OFF_CAL], 2);
  *p++ = '\n';
  m.pendOff = 0;
  m.pendLen = p - m.frame;                     // 56
}

// Hand the rest of the frame to USB without waiting.
static void flushFrame(Imu &m) {
  uint32_t actual = 0;
  _SerialUSB.send_nb((uint8_t *)m.frame + m.pendOff, m.pendLen, &actual, true);
  m.pendOff += actual;
  m.pendLen -= actual;
  if (actual && m.pendLen == 0) m.sent++;
}

// One scheduled frame for one IMU.
static void sample(Imu &m, uint32_t now) {
  if (m.backoff) {
    if ((int32_t)(now - m.retryAt) < 0) return;  // wait out RETRY_US after a failed read
    m.backoff = false;
  }
  if (m.pendLen) {                             // last frame never got out: PC not reading
    m.dropped++;
    if (m.pendOff == 0) m.pendLen = 0;         // drop it whole, never wait
  }
  uint16_t seq = m.seq++;
  if (m.pendLen) return;                       // half a frame out (not expected): finish it first

  uint8_t b[BURST_LEN];
  if (!readLen(m.addr, REG_BURST, b, BURST_LEN)) {
    m.readErrors++;
    m.backoff = true;
    m.retryAt = now + RETRY_US;
    return;
  }
  buildFrame(m, seq, millis(), b);
  flushFrame(m);
}

// 'i' for one IMU: "IMU: q=(...) ... | stream on, sent N, ..." ("IMU 0x29: ..."
// when two IMUs report).
static void printReading(Imu &m, bool labelled) {
  uint8_t b[BURST_LEN];
  if (!readLen(m.addr, REG_BURST, b, BURST_LEN)) {
    if (labelled) {
      Serial.print("IMU ERROR: no reading from the BNO055 at 0x");
      Serial.println(m.addr, HEX);
    } else {
      Serial.println("IMU ERROR: no reading from a BNO055 on Wire1 (0x28/0x29)");
    }
    return;
  }
  if (labelled) {
    Serial.print("IMU 0x");
    Serial.print(m.addr, HEX);
    Serial.print(": q=(");
  } else {
    Serial.print("IMU: q=(");
  }
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
  Serial.print(", ");
  printCounters(m, false);
  Serial.println();
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
  for (Imu &m : g_imu) {
    if (m.pendLen) flushFrame(m);              // retry until the endpoint is free
  }

  uint32_t now = micros();
  if ((int32_t)(now - g_next) < 0) return;
  g_next += PERIOD_US;
  if ((int32_t)(now - g_next) > 0) g_next = now + PERIOD_US;   // >1 period late: resync, no burst

  for (Imu &m : g_imu) {
    if (m.found) sample(m, now);
  }
}

void stream(bool on) {
  if (on && foundCount() < 2) bringUp(RETRY_PROBE_MS);
  if (on && !foundCount()) {
    Serial.println("IMU ERROR: no BNO055 on Wire1 (0x28/0x29)");
    return;
  }
  if (on) {
    for (Imu &m : g_imu) {
      m.sent = m.dropped = m.readErrors = 0;
      m.pendLen = 0;
      m.backoff = false;
    }
    g_next = micros();
    g_stream = true;
    Serial.print("IMU: stream on, ");
    Serial.print(IMU_STREAM_HZ);
    Serial.print(" Hz, ");
    printWhere();
    Serial.println();
  } else {
    g_stream = false;
    bool labelled = foundCount() > 1;
    const char *sep = "";
    Serial.print("IMU: stream off (");
    for (Imu &m : g_imu) {
      m.pendLen = 0;
      if (!m.found) continue;
      Serial.print(sep);
      printCounters(m, labelled);
      sep = "; ";
    }
    if (!foundCount()) printCounters(g_imu[0], false);   // none found: the zeros, as ever
    Serial.println(")");
  }
}

void printOne() {
  if (foundCount() < 2) bringUp(RETRY_PROBE_MS);
  if (!foundCount()) {
    Serial.println("IMU ERROR: no reading from a BNO055 on Wire1 (0x28/0x29)");
    return;
  }
  bool labelled = foundCount() > 1;
  for (Imu &m : g_imu) {
    if (m.found) printReading(m, labelled);
  }
}

} // namespace imu
