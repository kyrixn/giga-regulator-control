/**
 * encoder_rs485.h
 *
 * Up to ENC_MAX GJW absolute encoders on one RS-485 bus, read over Modbus RTU
 * through a MAX485 transceiver on the header pins silkscreened TX1/RX1,
 * D18 and D19.
 *
 * NOTE the object name: those pins are `Serial2`, NOT `Serial1`. The GIGA
 * variant maps SERIAL1_TX/RX to D1/D0 and SERIAL2_TX/RX to D18/D19
 * (variants/GIGA/pins_arduino.h), so the silkscreen's "1" and the Arduino
 * object's "1" are different numbers. Same trap as the SPI/SPI1 split on
 * D11/12/13 -- see the note in adc_ad7606.cpp. Opening Serial1 here talks to
 * D0/D1 and the bus stays silent with no error anywhere.
 *
 * Protocol (identical to the vc2_webapp project's modbus_rtu.py, which is the
 * reference implementation): read holding registers, function 0x03, address
 * 0x0380, count 16 -- the GJW "group 14" live state block. Decoded fields:
 *
 *   reg[0..1]   single-turn position   u32   (0..counts_per_turn-1)
 *   reg[2..3]   turns                  s32
 *   reg[4]      status code            u16
 *   reg[5]      angular velocity       s16
 *   reg[8..9]   original single-turn   u32   (pre-offset, unused here)
 *   reg[12]     error count            u16
 *
 * absolute position = turns * ENC_COUNTS_PER_TURN + single_turn_position, which
 * needs 64 bits: at 2^21 counts/turn it leaves int32 after ~1024 turns.
 *
 * WIRING
 *   MAX485 RO  -> Giga D19 (RX1 pin, = Serial2 RX)
 *   MAX485 DI  -> Giga D18 (TX1 pin, = Serial2 TX)
 *   MAX485 DE + /RE tied together     -> Giga ENC_DE_PIN
 *   A / B to the encoder bus, 120R termination at both physical ends.
 *
 *   The transceiver is half duplex, so DE must be driven: HIGH while the
 *   request goes out, LOW the rest of the time or the Giga cannot hear the
 *   reply. Set ENC_DE_PIN to -1 only for an auto-direction module.
 *
 *   VOLTAGE: a plain MAX485 is a 5V part and its RO pin then swings to 5V,
 *   which the Giga's 3.3V-only GPIO will not survive. Use a MAX3485 (the 3.3V
 *   sibling), or a level shifter / divider on RO. See the note in the .cpp.
 *
 * Design contract (mirrors adc_ad7606 and ui_display): enc::tick() is
 * cooperative and non-blocking. A Modbus transaction is a state machine spread
 * across many calls -- it never waits on the wire -- so a silent encoder costs
 * a timeout in wall-clock, not in blocked loop() iterations. Call it AFTER
 * processSerialCommand() so USB serial keeps priority.
 */
#pragma once
#include <Arduino.h>

#define ENC_MAX        6        // encoders tracked (the compact rig's muscles)

// Default slave-id range swept looking for encoders. Nothing is hardcoded to a
// particular id: whatever answers inside the range is kept, in the order found.
// enc::rescan(lo, hi) narrows it at runtime ('es50,80') without a reflash.
//
// A full 0..200 sweep costs ~10s at ENC_RX_TIMEOUT_MS per silent id. That is
// wall-clock only -- tick() stays non-blocking throughout -- but it is why the
// sweep narrows to the ids that answered once it has found them.
#define ENC_SCAN_LO    0
#define ENC_SCAN_HI    200
#define ENC_BAUD       115200
#define ENC_PARITY     SERIAL_8N1

// The UART behind D18/D19. See the note above before changing this.
#define ENC_UART       Serial2

// MAX485 DE + /RE, tied together. -1 for an auto-direction module.
// D10 is free: D2..D7 are the solenoids, D11..D13 SPI1, D18/D19 the RS-485 UART.
#define ENC_DE_PIN     10

// 21-bit single-turn resolution, matching DEFAULT_COUNTS_PER_TURN in
// vc2_webapp/encoder_controller.py.
#define ENC_COUNTS_PER_TURN 2097152L

namespace enc {
  void begin();
  void tick();                    // non-blocking; drives one transaction at a time

  int      count();               // encoders that answered the scan
  uint8_t  slaveId(int i);        // Modbus id of tracked encoder i
  bool     online(int i);         // answered its most recent poll
  uint32_t singleTurn(int i);     // raw single-turn counts
  int32_t  turns(int i);          // raw multi-turn count
  int64_t  absolute(int i);       // turns * ENC_COUNTS_PER_TURN + singleTurn
  int16_t  speed(int i);          // raw angular velocity
  uint16_t statusCode(int i);
  uint16_t errorCount(int i);

  // Bus-level counters, for telling "nothing is reaching the UART" apart from
  // "bytes arrive but nothing validates". The two have disjoint causes.
  uint32_t rxBytes();
  uint32_t badFrames();
  uint32_t echoes();              // frames recognised as our own request coming back

  void printAll();                // the 'e' command: raw values for every encoder
  void listen(uint32_t ms);       // 'el': dump everything the UART hears, unframed
  void loopbackTest();            // 'et': prove the UART itself works
  void rescan();                  // the 'es' command: sweep the current range
  void rescan(int lo, int hi);    // 'es<lo>,<hi>': sweep a different range
  int  scanLo();
  int  scanHi();
  void setHexDump(bool on);       // the 'ex' command: dump every frame as hex
  bool hexDump();
}
