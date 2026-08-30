/**
 * encoder_rs485.cpp  --  see encoder_rs485.h
 *
 * One Modbus transaction is a three-state machine, so tick() never blocks:
 *
 *   ST_GAP  inter-frame silence. Modbus RTU wants 3.5 character times of quiet
 *           between frames; at 115200 that is ~304us, and ENC_GAP_US clears it.
 *   ST_TX   the request is in the UART's buffer and DE is HIGH. We hold it there
 *           for as long as the bytes take to clock out, then drop DE. Timed
 *           rather than flush()ed on purpose: flush() blocks for ~700us, and
 *           releasing DE one bit early truncates the CRC and the encoder never
 *           answers.
 *   ST_RX   bytes are accumulated as they arrive. The expected frame length is
 *           known once three bytes are in (byte 2 is the payload size), so a
 *           short reply is caught by the timeout rather than by a fixed wait.
 *
 * The scan sweeps ENC_SCAN_LO..ENC_SCAN_HI once, keeps the ids that answered,
 * and then polls only those -- an absent id costs a full ENC_RX_TIMEOUT_MS, so
 * polling all 31 every cycle would dominate the loop.
 */
#include "encoder_rs485.h"
#include <string.h>

// GJW group-14 live state block (holding registers).
static const uint8_t  FN_READ_HOLDING   = 0x03;
static const uint16_t GJW_STATE_REGISTER = 0x0380;
static const uint16_t GJW_STATE_COUNT    = ENC_REG_COUNT;

// 1 slave + 1 function + 1 byte-count + 32 data + 2 CRC.
static const int RESP_MAX = 64;

static const uint32_t ENC_GAP_US        = 500;   // >= 3.5 char times at 115200
static const uint32_t ENC_RX_TIMEOUT_MS = 50;    // matches the webapp's 0.06s

// DE is asserted this long before the first bit, letting the driver enable so
// the start bit is not clipped. ref/sketch_position_speed.ino uses 15us.
static const uint32_t ENC_TX_SETTLE_US = 15;

// ...and held this long past the computed end of transmission.
//
// The reference sketch's 30us is measured from AFTER flush(), and that is what
// broke this on the GIGA: AVR's flush() waits for the shift register to empty,
// mbed's does not, so DE dropped mid-byte and truncated the CRC. Raising it to
// 200us on the Mega-derived code is what finally made the encoder answer.
//
// This code never calls flush(); it times from before the write, so the hold
// already covers the whole transmission. The guard is only slack for however
// long the UART takes to actually start shifting. It must stay under the 3.5
// character times (~304us at 115200) a slave waits before replying, or we are
// still driving the bus when the answer starts.
static const uint32_t ENC_TX_GUARD_US = 150;

// Nothing answered the sweep: retry this often rather than sitting dead.
static const uint32_t ENC_RESCAN_MS = 5000;

// ---- Per-encoder state ----------------------------------------------------
struct Enc {
  uint8_t  slave;
  bool     online;
  uint32_t singleTurn;
  int32_t  turns;
  int16_t  speed;
  uint16_t status;
  uint16_t errors;
  uint16_t badFrames;      // CRC / framing rejects since boot
};

static Enc      g_enc[ENC_MAX];
static int      g_count = 0;

// ---- Transaction state machine --------------------------------------------
enum State { ST_GAP, ST_TX, ST_RX };
static State    g_state = ST_GAP;

static bool     g_scanning  = true;
static int      g_scanLo    = ENC_SCAN_LO;   // range currently being swept
static int      g_scanHi    = ENC_SCAN_HI;
static int      g_scanId    = ENC_SCAN_LO;   // id being probed during a scan
static int      g_pollIdx   = 0;             // index into g_enc when polling
static uint8_t  g_curSlave  = 0;             // slave of the transaction in flight

static uint8_t  g_rx[RESP_MAX];
static int      g_rxLen     = 0;
static int      g_rxExpect  = 0;             // 0 until the header reveals it

static uint32_t g_txStartUs = 0;
static uint32_t g_txHoldUs  = 0;
static uint32_t g_gapStartUs = 0;
static uint32_t g_rxStartMs = 0;

static bool     g_hexDump   = false;

// Bus-level diagnostics. Zero bytes ever received means nothing is reaching the
// UART at all; bytes arriving with nothing validating means it hears but does
// not understand; echoes means /RE is not following DE.
static uint32_t g_rxBytes    = 0;
static uint32_t g_badFrames  = 0;
static uint32_t g_echoes     = 0;
// Transactions that timed out holding SOME bytes that never formed a frame.
// This is where stray bytes go: counted in g_rxBytes but never reaching
// acceptFrame(), so "rx bytes high, bad frames ~0" is line noise rather than a
// garbled reply. Keeping the last few makes which one it is obvious.
static uint32_t g_partials   = 0;
static uint32_t g_probes     = 0;
static uint8_t  g_lastPartial[8];
static int      g_lastPartialLen = 0;
static uint32_t g_scanEndMs  = 0;   // when the last fruitless sweep finished

// The request currently in flight, kept so a transceiver echo can be recognised
// byte-for-byte rather than guessed at from its length.
static uint8_t  g_req[8];

// 'el' raw-listen and 'et' loopback modes suspend the state machine.
static uint32_t g_listenUntilMs = 0;
static uint32_t g_testUntilMs   = 0;
static int      g_testGot       = 0;
static const uint8_t TEST_PATTERN[6] = { 0x55, 0xAA, 0x00, 0xFF, 0x5A, 0xA5 };

// ============================================================
// Modbus helpers
// ============================================================
static uint16_t modbusCrc(const uint8_t *data, int len) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else         crc >>= 1;
    }
  }
  return crc;
}

static void printHexFrame(const char *tag, const uint8_t *d, int n) {
  Serial.print(tag);
  for (int i = 0; i < n; i++) {
    if (d[i] < 0x10) Serial.print(" 0");
    else             Serial.print(' ');
    Serial.print(d[i], HEX);
  }
  Serial.println();
}

// Serial.print has no int64 overload; build the digits by hand.
static void printI64(int64_t v) {
  if (v < 0) { Serial.print('-'); v = -v; }
  char buf[21];
  int i = 0;
  if (v == 0) buf[i++] = '0';
  while (v > 0) { buf[i++] = (char)('0' + (int)(v % 10)); v /= 10; }
  while (i > 0) Serial.print(buf[--i]);
}

// Runtime, not compile-time: which pin the DE wire actually landed on is the
// single most common thing to get wrong here, and reflashing to try another is
// a slow way to find out. 'ep<n>' moves it.
static int g_dePin = ENC_DE_PIN;

static inline void deWrite(bool transmitting) {
  if (g_dePin >= 0) digitalWrite(g_dePin, transmitting ? HIGH : LOW);
}

// ============================================================
// Transaction steps
// ============================================================
static void startRequest(uint8_t slave) {
  uint8_t req[8];
  req[0] = slave;
  req[1] = FN_READ_HOLDING;
  req[2] = (uint8_t)(GJW_STATE_REGISTER >> 8);
  req[3] = (uint8_t)(GJW_STATE_REGISTER & 0xFF);
  req[4] = (uint8_t)(GJW_STATE_COUNT >> 8);
  req[5] = (uint8_t)(GJW_STATE_COUNT & 0xFF);
  uint16_t crc = modbusCrc(req, 6);
  req[6] = (uint8_t)(crc & 0xFF);       // CRC goes out low byte first
  req[7] = (uint8_t)(crc >> 8);

  g_probes++;
  g_curSlave = slave;
  g_rxLen    = 0;
  g_rxExpect = 0;
  memcpy(g_req, req, sizeof(req));

  while (ENC_UART.available()) ENC_UART.read();   // drop any stale bytes

  deWrite(true);
  delayMicroseconds(ENC_TX_SETTLE_US);   // let the driver enable first

  // Stamp the clock BEFORE the write, not after. Whether write() buffers and
  // returns at once or blocks until the bytes are out, the deadline then
  // measures from the instant transmission began: buffered, we hold the full
  // byte-time; blocking, the time is already spent and DE drops immediately.
  // Stamping afterwards adds a second transmission's worth of hold on a
  // blocking core and talks straight over the encoder's reply.
  g_txStartUs = micros();
  ENC_UART.write(req, sizeof(req));

  // 10 bits per byte on the wire (start + 8 data + stop), plus the guard.
  g_txHoldUs = ((uint32_t)sizeof(req) * 10UL * 1000000UL) / ENC_BAUD
               + ENC_TX_GUARD_US;
  g_state    = ST_TX;

  if (g_hexDump) printHexFrame("[enc] TX ->", req, sizeof(req));
}

// Decode a validated 16-register state block into slot `idx`.
static void storeState(int idx, const uint8_t *regs) {
  // Registers are big-endian pairs starting at g_rx[3]; reg[n] = regs[2n..2n+1].
  #define REG(n) ((uint16_t)((regs[(n) * 2] << 8) | regs[(n) * 2 + 1]))
  g_enc[idx].singleTurn = ((uint32_t)REG(0) << 16) | REG(1);
  g_enc[idx].turns      = (int32_t)(((uint32_t)REG(2) << 16) | REG(3));
  // Status, speed and error count live past register 3, so they only exist
  // when the larger block is being read. Left at zero otherwise rather than
  // decoded out of registers we never asked for.
  #if ENC_REG_COUNT >= 13
    g_enc[idx].status = REG(4);
    g_enc[idx].speed  = (int16_t)REG(5);
    g_enc[idx].errors = REG(12);
  #endif
  #undef REG
  g_enc[idx].online = true;
}

static int slotOf(uint8_t slave) {
  for (int i = 0; i < g_count; i++) if (g_enc[i].slave == slave) return i;
  return -1;
}

// True if what we just collected is our own request coming back. Happens when
// the transceiver's /RE is not tied to DE, so the receiver stays enabled while
// we transmit. Recognised byte-for-byte rather than by length: our request is a
// well-formed Modbus frame with a valid CRC, so it survives every other check
// and would otherwise be counted as a protocol error and hide the real cause.
static bool isEcho() {
  return g_rxLen == (int)sizeof(g_req) && memcmp(g_rx, g_req, sizeof(g_req)) == 0;
}

// Validate the frame sitting in g_rx. Returns true if it was stored.
static bool acceptFrame() {
  if (g_hexDump) printHexFrame("[enc] RX <-", g_rx, g_rxLen);

  // Anything that arrived but did not validate is counted: that is the signal
  // the bus is alive and the misunderstanding is in baud/parity/protocol.
  if (g_rxLen < 5)                    { g_badFrames++; return false; }
  uint16_t want = modbusCrc(g_rx, g_rxLen - 2);
  uint16_t got  = (uint16_t)g_rx[g_rxLen - 2] | ((uint16_t)g_rx[g_rxLen - 1] << 8);
  if (want != got)                    { g_badFrames++; return false; }
  if (g_rx[0] != g_curSlave)          { g_badFrames++; return false; }
  if (g_rx[1] & 0x80)                 { g_badFrames++; return false; }  // exception
  if (g_rx[1] != FN_READ_HOLDING)     { g_badFrames++; return false; }
  if (g_rx[2] != GJW_STATE_COUNT * 2) { g_badFrames++; return false; }

  int idx = slotOf(g_curSlave);
  if (idx < 0) {
    if (g_count >= ENC_MAX) return false;       // more encoders than we track
    idx = g_count++;
    g_enc[idx].slave     = g_curSlave;
    g_enc[idx].badFrames = 0;
  }
  storeState(idx, &g_rx[3]);
  return true;
}

// Advance to whatever transaction comes next.
static void finishTransaction(bool ok) {
  if (g_scanning) {
    if (!ok && g_hexDump) {
      Serial.print("[enc] no reply from slave ");
      Serial.println(g_curSlave);
    }
    g_scanId++;
    if (g_scanId > g_scanHi || g_count >= ENC_MAX) {
      g_scanning = false;
      g_pollIdx  = 0;
      Serial.print("Encoders: scan done, ");
      Serial.print(g_count);
      Serial.print(" online (");
      for (int i = 0; i < g_count; i++) {
        if (i) Serial.print(",");
        Serial.print(g_enc[i].slave);
      }
      Serial.print(") in ids ");
      Serial.print(g_scanLo); Serial.print("-"); Serial.print(g_scanHi);
      Serial.print("  [rx bytes "); Serial.print(g_rxBytes);
      Serial.print(", bad frames "); Serial.print(g_badFrames);
      Serial.print(", echoes "); Serial.print(g_echoes);
      Serial.println("]");
      g_scanEndMs = millis();
    }
  } else {
    int idx = slotOf(g_curSlave);
    if (idx >= 0 && !ok) {
      g_enc[idx].online = false;
      g_enc[idx].badFrames++;
    }
    if (g_count > 0) g_pollIdx = (g_pollIdx + 1) % g_count;
  }
  g_gapStartUs = micros();
  g_state = ST_GAP;
}

namespace enc {

void begin() {
  if (g_dePin >= 0) {
    // Receive by default: DE must be low before the UART comes up, or this node
    // drives the bus while every other device is trying to talk.
    digitalWrite(g_dePin, LOW);
    pinMode(g_dePin, OUTPUT);
    digitalWrite(g_dePin, LOW);
  }
  ENC_UART.begin(ENC_BAUD, ENC_PARITY);

  for (int i = 0; i < ENC_MAX; i++) g_enc[i] = Enc();
  g_count      = 0;
  g_scanning   = true;
  g_scanLo     = ENC_SCAN_LO;
  g_scanHi     = ENC_SCAN_HI;
  g_scanId     = g_scanLo;
  g_pollIdx    = 0;
  g_state      = ST_GAP;
  g_gapStartUs = micros();
}

void tick() {
  // 'el': pure receive. Dumps whatever lands on the UART with no framing at
  // all, so a bus driven by some other master (or a scope-less sanity check on
  // baud) shows up even when nothing here validates.
  if (g_listenUntilMs) {
    while (ENC_UART.available()) {
      uint8_t b = (uint8_t)ENC_UART.read();
      g_rxBytes++;
      if (b < 0x10) Serial.print(" 0"); else Serial.print(' ');
      Serial.print(b, HEX);
    }
    if ((int32_t)(millis() - g_listenUntilMs) >= 0) {
      g_listenUntilMs = 0;
      Serial.println();
      Serial.print("Encoders: listen done, ");
      Serial.print(g_rxBytes);
      Serial.println(" byte(s) total since the last rescan");
      g_gapStartUs = micros();
      g_state = ST_GAP;
    }
    return;
  }

  // 'et': loopback. Proves the UART, the baud rate and the two pins work
  // without any encoder or transceiver in the picture.
  if (g_testUntilMs) {
    while (ENC_UART.available()) {
      uint8_t b = (uint8_t)ENC_UART.read();
      if (g_testGot < (int)sizeof(TEST_PATTERN) && b == TEST_PATTERN[g_testGot]) {
        g_testGot++;
      }
    }
    if (g_testGot >= (int)sizeof(TEST_PATTERN)) {
      g_testUntilMs = 0;
      Serial.println("Encoders: LOOPBACK PASS - the UART, baud and pins are fine.");
      Serial.println("  So the fault is past the Giga: transceiver, DE, A/B, or the bus.");
      g_gapStartUs = micros();
      g_state = ST_GAP;
    } else if ((int32_t)(millis() - g_testUntilMs) >= 0) {
      g_testUntilMs = 0;
      Serial.print("Encoders: LOOPBACK FAIL - got ");
      Serial.print(g_testGot);
      Serial.print(" of ");
      Serial.print((int)sizeof(TEST_PATTERN));
      Serial.println(" pattern byte(s).");
      Serial.println("  With D18 jumpered to D19 this must pass. If it does not,");
      Serial.println("  the UART or the pins are wrong, not the RS-485 side.");
      g_gapStartUs = micros();
      g_state = ST_GAP;
    }
    return;
  }

  switch (g_state) {

    case ST_GAP:
      if ((uint32_t)(micros() - g_gapStartUs) < ENC_GAP_US) return;
      if (g_scanning) {
        startRequest((uint8_t)g_scanId);
      } else if (g_count > 0) {
        startRequest(g_enc[g_pollIdx].slave);
      } else {
        // Nothing answered. Retry on a timer rather than sitting dead, so
        // plugging the bus in after boot is enough to bring it up.
        if ((uint32_t)(millis() - g_scanEndMs) >= ENC_RESCAN_MS) {
          g_scanning = true;
          g_scanId   = g_scanLo;
        }
        g_gapStartUs = micros();
      }
      return;

    case ST_TX:
      if ((uint32_t)(micros() - g_txStartUs) < g_txHoldUs) return;
      deWrite(false);                      // release the bus, start listening
      g_rxStartMs = millis();
      g_state = ST_RX;
      return;

    case ST_RX: {
      while (ENC_UART.available() && g_rxLen < RESP_MAX) {
        g_rx[g_rxLen++] = (uint8_t)ENC_UART.read();
        g_rxBytes++;
        // Byte 2 carries the payload size, so the total length is known as
        // soon as the header lands -- exception frames are 5 bytes total.
        if (g_rxLen == 3) {
          g_rxExpect = (g_rx[1] & 0x80) ? 5 : (3 + g_rx[2] + 2);
          if (g_rxExpect > RESP_MAX) g_rxExpect = RESP_MAX;
        }
        if (g_rxExpect && g_rxLen >= g_rxExpect) {
          if (isEcho()) {
            // Our own request came back. Drop it, restart the receive window
            // and keep listening -- the encoder's reply is still to come.
            g_echoes++;
            if (g_hexDump) printHexFrame("[enc] echo <-", g_rx, g_rxLen);
            g_rxLen = 0;
            g_rxExpect = 0;
            g_rxStartMs = millis();
            continue;
          }
          finishTransaction(acceptFrame());
          return;
        }
      }
      if ((uint32_t)(millis() - g_rxStartMs) >= ENC_RX_TIMEOUT_MS) {
        if (g_rxLen) {
          g_partials++;
          g_lastPartialLen = g_rxLen < 8 ? g_rxLen : 8;
          memcpy(g_lastPartial, g_rx, g_lastPartialLen);
          if (g_hexDump) printHexFrame("[enc] RX partial <-", g_rx, g_rxLen);
        }
        finishTransaction(false);
      }
      return;
    }
  }
}

int      count()            { return g_count; }
uint8_t  slaveId(int i)     { return (i >= 0 && i < g_count) ? g_enc[i].slave : 0; }
bool     online(int i)      { return (i >= 0 && i < g_count) && g_enc[i].online; }
uint32_t singleTurn(int i)  { return (i >= 0 && i < g_count) ? g_enc[i].singleTurn : 0; }
int32_t  turns(int i)       { return (i >= 0 && i < g_count) ? g_enc[i].turns : 0; }
int16_t  speed(int i)       { return (i >= 0 && i < g_count) ? g_enc[i].speed : 0; }
uint16_t statusCode(int i)  { return (i >= 0 && i < g_count) ? g_enc[i].status : 0; }
uint16_t errorCount(int i)  { return (i >= 0 && i < g_count) ? g_enc[i].errors : 0; }

int64_t absolute(int i) {
  if (i < 0 || i >= g_count) return 0;
  return (int64_t)g_enc[i].turns * (int64_t)ENC_COUNTS_PER_TURN
       + (int64_t)g_enc[i].singleTurn;
}

void rescan(int lo, int hi) {
  // Modbus reserves 248..255; clamp rather than reject so a fat-fingered
  // 'es0,300' still does something sane instead of hanging.
  if (lo < 0)   lo = 0;
  if (hi > 247) hi = 247;
  if (hi < lo)  hi = lo;

  g_scanLo   = lo;
  g_scanHi   = hi;
  g_count    = 0;
  g_scanning = true;
  g_scanId   = g_scanLo;
  g_pollIdx  = 0;
  g_rxBytes   = 0;
  g_badFrames = 0;
  g_echoes    = 0;
  g_partials  = 0;
  g_probes    = 0;
  g_lastPartialLen = 0;
  for (int i = 0; i < ENC_MAX; i++) g_enc[i] = Enc();
  Serial.print("Encoders: rescanning ids ");
  Serial.print(g_scanLo); Serial.print("-"); Serial.println(g_scanHi);
}

void rescan() { rescan(g_scanLo, g_scanHi); }

int scanLo() { return g_scanLo; }
int scanHi() { return g_scanHi; }

uint32_t rxBytes()   { return g_rxBytes; }
uint32_t badFrames() { return g_badFrames; }
uint32_t echoes()    { return g_echoes; }

void listen(uint32_t ms) {
  while (ENC_UART.available()) ENC_UART.read();
  g_listenUntilMs = millis() + ms;
  if (g_listenUntilMs == 0) g_listenUntilMs = 1;   // 0 is the "off" sentinel
  Serial.print("Encoders: listening on D18/D19 for ");
  Serial.print(ms);
  Serial.println(" ms (no framing, raw hex):");
}

void loopbackTest() {
  Serial.println("Encoders: loopback test -- jumper D18 to D19 first.");
  // DE stays low so an attached transceiver keeps its receiver on and its
  // driver off; with a direct jumper the transceiver plays no part either way.
  deWrite(false);
  while (ENC_UART.available()) ENC_UART.read();
  g_testGot = 0;
  ENC_UART.write(TEST_PATTERN, sizeof(TEST_PATTERN));
  g_testUntilMs = millis() + 200;
  if (g_testUntilMs == 0) g_testUntilMs = 1;
}

// Move the DE line to another pin without a reflash.
void setDePin(int pin) {
  if (g_dePin >= 0) {
    pinMode(g_dePin, INPUT);         // release the old one so it cannot fight
  }
  g_dePin = pin;
  if (g_dePin >= 0) {
    digitalWrite(g_dePin, LOW);
    pinMode(g_dePin, OUTPUT);
    digitalWrite(g_dePin, LOW);
  }
  Serial.print("Encoders: DE pin now D");
  if (g_dePin < 0) Serial.println("(none, auto-direction module)");
  else             Serial.println(g_dePin);
  if (g_dePin >= 2 && g_dePin <= 7) {
    Serial.println("  WARNING: D2-D7 drive the on/off solenoids. Both will fight");
    Serial.println("  for this pin -- move the DE wire or the solenoid wire.");
  }
}

int dePin() { return g_dePin; }

void setHexDump(bool on) {
  g_hexDump = on;
  Serial.print("Encoders: frame hex dump ");
  Serial.println(on ? "ON" : "OFF");
}

bool hexDump() { return g_hexDump; }

// Raw values only -- no mm conversion here. The drum geometry that turns counts
// into displacement lives in the PC app (vc2_webapp), and duplicating it on the
// Giga would give two places to disagree about what a count means.
void printAll() {
  Serial.println("=== RS-485 encoders (Serial2 = pins D18/D19) ===");
  if (g_scanning) {
    Serial.print("scanning... at id ");
    Serial.println(g_scanId);
  }
  Serial.print("range "); Serial.print(g_scanLo);
  Serial.print("-"); Serial.print(g_scanHi);
  Serial.print("  rx bytes "); Serial.print(g_rxBytes);
  Serial.print("  bad frames "); Serial.print(g_badFrames);
  Serial.print("  echoes "); Serial.println(g_echoes);
  Serial.print("probes "); Serial.print(g_probes);
  Serial.print("  short timeouts "); Serial.print(g_partials);
  if (g_probes) {
    Serial.print("  avg bytes/probe ");
    Serial.print((float)g_rxBytes / (float)g_probes, 2);
  }
  Serial.print("  DE pin D"); Serial.println(g_dePin);
  if (g_lastPartialLen) printHexFrame("last stray bytes:", g_lastPartial, g_lastPartialLen);

  if (g_count == 0) {
    Serial.println("no encoders found.");
    if (g_echoes > 0) {
      Serial.println("  ECHO: our own request is coming back, so /RE is not");
      Serial.print("  following DE. Tie /RE to DE (both to D");
      Serial.print(ENC_DE_PIN);
      Serial.println("). Echo is now");
      Serial.println("  discarded rather than mistaken for a reply, but the");
      Serial.println("  encoder still never answers if it never heard us.");
    } else if (g_rxBytes == 0) {
      // Not one edge got through: the fault is upstream of the protocol.
      Serial.println("  NOTHING received at all. In likelihood order:");
      Serial.print("   1. DE+/RE wired to D"); Serial.print(ENC_DE_PIN);
      Serial.println("? Floating or stuck high = transmit-only, deaf.");
      Serial.println("   2. Run 'et' with D18 jumpered to D19 to prove the UART.");
      Serial.println("   3. A/B swapped, or no 120R at the bus ends?");
      Serial.println("   4. Transceiver powered? RO->D19, DI->D18, GND common?");
      Serial.println("   5. 5V MAX485 drives RO to 5V; the Giga needs 3.3V (MAX3485).");
    } else if (g_probes && g_rxBytes < g_probes * 3) {
      // About a byte per probe, never a 13-byte reply. That is an undriven
      // bus, not an encoder answering badly.
      Serial.println("  ~1 stray byte per probe, never a reply-sized frame.");
      Serial.println("  Nothing is driving the bus. Most likely, in order:");
      Serial.print("   1. DE is not on D"); Serial.print(g_dePin);
      Serial.println(". The reference sketch used the Mega's");
      Serial.println("      pin 2 -- if that wire moved across unchanged it is on");
      Serial.println("      the Giga's D2, which this build drives as solenoid d0.");
      Serial.println("      A DE held high is transmit-only: the encoder hears us");
      Serial.println("      but we are deaf to it, and RO floats into noise.");
      Serial.println("      'ep<n>' moves the DE pin live, e.g. ep2 or ep10.");
      Serial.println("   2. A/B swapped, so the encoder cannot decode the request.");
      Serial.println("   3. No failsafe bias on A/B (~560R-1k) -- that is what");
      Serial.println("      turns an idle bus into these stray bytes.");
      Serial.println("  'el' listens without transmitting: stray bytes while idle");
      Serial.println("  confirms 3; silence means the noise is DE switching.");
    } else {
      Serial.println("  reply-sized traffic but nothing validated:");
      Serial.println("   - baud or parity mismatch? this build is 115200 8N1.");
      Serial.println("   - run 'ex' then 'es' to see the raw frames.");
    }
    return;
  }
  Serial.println("slave  on  single_turn      turns    absolute  speed  stat  err  bad");
  for (int i = 0; i < g_count; i++) {
    char buf[80];
    snprintf(buf, sizeof(buf), "%5u  %2d  %11lu %10ld  ",
             (unsigned)g_enc[i].slave,
             g_enc[i].online ? 1 : 0,
             (unsigned long)g_enc[i].singleTurn,
             (long)g_enc[i].turns);
    Serial.print(buf);
    printI64(absolute(i));
    snprintf(buf, sizeof(buf), "  %5d  %4u %4u %4u",
             (int)g_enc[i].speed, (unsigned)g_enc[i].status,
             (unsigned)g_enc[i].errors, (unsigned)g_enc[i].badFrames);
    Serial.println(buf);
  }
}

}  // namespace enc
