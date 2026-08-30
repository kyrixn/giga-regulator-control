/**
 * vc2.ino
 *
 * 6-Valve Controller (Feedforward) for Arduino Giga R1 - compact build.
 *
 * Hardware:
 *   Wire (SDA/SCL, D20/D21): 3 GP8403 DACs at 0x58..0x5A -> valves 0..5
 *   DAC output: 0-10V   (2 channels per DAC -> 2 valves per DAC)
 *
 *   Only the default I2C bus is used. Wire1 belongs to the GIGA Display
 *   Shield's GT911 touch controller (0x5D, which would clash with a DAC);
 *   Wire2 is unused in this build.
 *
 * 3 DACs total, 6 regulators total.
 *
 * Valve-to-DAC mapping (2 channels per DAC):
 *   Bus Wire   DAC 0x58 -> V0, V1
 *              DAC 0x59 -> V2, V3
 *              DAC 0x5A -> V4, V5
 *
 * Regulator (compact rig):
 *   command  0-10V in  -> 0..900 kPa
 *   monitor  1-5V  out -> 0..900 kPa   (different slope from the command side)
 *   linearity +/-1% F.S. = +/-9 kPa
 *
 * On/off valves: six 2-position solenoids on D2..D7 through an 8-channel
 * opto-isolated MOSFET board (see valves_onoff.h). Independent of the
 * regulators above; addressed with 'd' commands.
 *
 * Length feedback: up to 6 GJW absolute encoders on RS-485, read as Modbus RTU
 * through a MAX485 on pins D18/D19 (the TX1/RX1 silkscreen = Serial2), DE on D10.
 * same register block as the vc2_webapp project; see encoder_rs485.h.
 *
 * Analog feedback: one AD7606 board (board 0), channels 0..5 -> valves 0..5.
 * The 1-5V monitor sits inside the ADC's +/-10V range as wired; see
 * ADC_RANGE_V in adc_ad7606.h if the module's RANGE pin is moved to +/-5V.
 *
 * Commands (Serial @ 115200):
 *   Proportional regulators (bare index 0..5, value in mV 0..10000):
 *     valve,value        Set one:       0,3000
 *     v1,val1,v2,val2,.. Set several:   0,3000,4,2500
 *     valve,off          Release one:   4,off
 *     ?                  Status of all 6 regulators
 *   On/off valves ('d' prefix, index 0..5 -> D2..D7):
 *     dN,V               Set one:       d0,1  d0,on  d0,0  d0,off
 *     dN,V,dM,V,..       Set several:   d0,1,d3,0
 *     d,off              Release all on/off valves
 *     d  or  d?          Status of all 6 on/off valves
 *   RS-485 encoders:
 *     e                  Raw values for every encoder found
 *     es                 Re-sweep the slave-id range
 *     es<lo>,<hi>        Re-sweep a different id range, e.g. es50,80
 *     ep<n>              Move the DE line to pin n live (ep10, ep2, ep-1=none)
 *     el                 Dump the bus unframed for 3s (raw hex)
 *     et                 Loopback self-test (jumper D18 to D19 first)
 *     ex                 Toggle hex dump of every Modbus frame
 *   c<v>[,maxMv[,n]]   Sweep regulator v and print commanded vs measured
 *   k                  Show the per-regulator calibration tables
 *   s                  Emergency stop (regulators AND on/off valves)
 *   p                  Ping test
 *   a                  Dump every AD7606 channel
 *   i                  I2C bus scan
 */

#include <Arduino.h>
#include <Wire.h>
#include "DFRobot_GP8403.h"

#include "valve_core.h"   // read-only accessors exposed to the display UI
#include "ui_display.h"   // on-Giga touchscreen UI (non-blocking)
#include "adc_ad7606.h"   // AD7606 analog acquisition on SPI (non-blocking)
#include "valves_onoff.h" // six 2-position solenoids on D2..D7 (GPIO only)
#include "encoder_rs485.h"// GJW encoders on D18/D19 via MAX485 (non-blocking)
#include "calibrate.h"   // command-vs-measured sweep for one regulator
#include "calib_table.h"// per-regulator open-loop command correction

#define NUM_DACS     3                    // 0x58..0x5A on Wire
#define NUM_VALVES   (NUM_DACS * 2)       // 6 valves (2 channels per DAC)

static_assert(NUM_VALVES == VC_NUM_VALVES,
              "valve_core.h VC_NUM_VALVES must match NUM_VALVES");
static_assert(DV_NUM_VALVES == VC_NUM_ONOFF,
              "valve_core.h VC_NUM_ONOFF must match DV_NUM_VALVES");
static_assert(ENC_MAX == VC_NUM_SENSORS,
              "valve_core.h VC_NUM_SENSORS must match ENC_MAX");

// First contiguous I2C address of the DACs on the bus (0x58..0x5A).
#define DAC_ADDR_BASE 0x58

// ============================================================
// INPUT MODE: Change this flag to switch input format
// ============================================================
// true  = PRESSURE mode: input in kPa, mapped to 0-10V
// false = VOLTAGE mode:  input in mV (0 to 10000), direct output
#define INPUT_PRESSURE_MODE false

// Pressure range in kPa (only used in PRESSURE mode)
// Device mapping: 0-10V corresponds to 0 to 900 kPa
#define PRESSURE_MIN 0
#define PRESSURE_RANGE_MAX 900

// Safety limit: output pressure should not exceed this value
#define PRESSURE_SAFETY_MAX 900

// Voltage range in mV (only used in VOLTAGE mode)
#define VOLTAGE_MIN 0
#define VOLTAGE_MAX 10000

// ============================================================
// DAC instances
// ============================================================
// One object per physical DAC, all on the default bus (Wire) driving
// valves 0..5. Addresses run 0x58..0x5A. Wire1 is intentionally NOT used
// here -- it belongs to the display shield's touch controller.

DFRobot_GP8403 dacBus0[NUM_DACS] = {
  DFRobot_GP8403(&Wire, DAC_ADDR_BASE + 0),
  DFRobot_GP8403(&Wire, DAC_ADDR_BASE + 1),
  DFRobot_GP8403(&Wire, DAC_ADDR_BASE + 2)
};

// Mapping arrays: valve index -> DAC pointer and channel.
// Filled once in buildMapping(). Even valve indices are channel 0 of a
// DAC, odd indices channel 1, so unique DACs sit at even indices.
DFRobot_GP8403* dacs[NUM_VALVES];
int dacChannels[NUM_VALVES];

// Track current value for each valve (pressure in kPa or voltage in mV)
int currentValue[NUM_VALVES] = {0};

// Serial input buffer. Serial is never gated -- the display's LOCK affects
// touch only -- so there is no discard path here.
String inputBuffer = "";

/**
 * Build the valve -> (DAC, channel) mapping.
 *   valves 0..5 -> dacBus0[0..2], channels 0/1
 */
void buildMapping() {
  for (int d = 0; d < NUM_DACS; d++) {
    int v = d * 2;
    dacs[v]     = &dacBus0[d]; dacChannels[v]     = 0;
    dacs[v + 1] = &dacBus0[d]; dacChannels[v + 1] = 1;
  }
}

// ============================================================
// Valve Functions
// ============================================================

/**
 * Initialize all DACs
 * Call this in setup()
 * Returns true if all DACs initialized successfully
 */
bool initValves() {
  bool success = true;

  // Initialize each unique DAC (indices 0, 2, 4, ..., NUM_VALVES-2)
  for (int i = 0; i < NUM_VALVES; i += 2) {
    bool ok = (dacs[i]->begin() == 0);

    // Set the range unconditionally, even when begin() failed.
    //
    // The library converts mV to a DAC code by dividing by an internal
    // `voltage` member, and setDACOutRange() is the only thing that ever sets
    // it -- it starts at 0. Skip this call and every later setDACOutVoltage()
    // computes data/0, producing one fixed code no matter what value is asked
    // for. The output then sticks at some arbitrary level while commands are
    // still accepted and '?' still shows the setpoint changing, which makes it
    // look like a wiring fault anywhere except here. Setting the range even on
    // a failed probe also means a DAC that was merely unplugged at boot starts
    // working the moment it is plugged back in.
    dacs[i]->setDACOutRange(DFRobot_GP8403::eOutputRange10V);

    Serial.print(ok ? "DAC initialized for valves "
                    : "ERROR: DAC init failed for valves ");
    Serial.print(i);
    Serial.print(" and ");
    Serial.print(i + 1);
    Serial.print("  (0x");
    Serial.print(DAC_ADDR_BASE + i / 2, HEX);
    Serial.println(ok ? ")" : " did not ACK on Wire)");

    if (!ok) success = false;
  }

  // Set all valves to 0 initially
  for (int i = 0; i < NUM_VALVES; i++) {
    setValve(i, 0);
  }

  return success;
}

/**
 * Set valve output
 *
 * @param valve Valve index (0-5)
 * @param value Pressure in kPa (if INPUT_PRESSURE_MODE) or voltage in mV (if not)
 * @return true if successful, false if invalid valve index
 */
bool setValve(int valve, int value) {
  if (valve < 0 || valve >= NUM_VALVES) {
    Serial.print("ERROR: Invalid valve index ");
    Serial.println(valve);
    return false;
  }

  int mV;

  #if INPUT_PRESSURE_MODE
    value = constrain(value, PRESSURE_MIN, PRESSURE_SAFETY_MAX);
    mV = map(value, PRESSURE_MIN, PRESSURE_RANGE_MAX, 0, 10000);
  #else
    value = constrain(value, VOLTAGE_MIN, VOLTAGE_MAX);
    mV = value;
  #endif

  // Correct on the way out only. currentValue keeps what was ASKED for, so the
  // display, '?' and the PC app all report intent rather than the compensated
  // voltage -- otherwise a calibrated valve would appear to ignore commands.
  dacs[valve]->setDACOutVoltage(calibApply(valve, mV), dacChannels[valve]);
  currentValue[valve] = value;

  return true;
}

/**
 * Turn off a valve (set to 0V output)
 */
void valveOff(int valve) {
  if (valve < 0 || valve >= NUM_VALVES) return;
  dacs[valve]->setDACOutVoltage(0, dacChannels[valve]);
  #if INPUT_PRESSURE_MODE
    currentValue[valve] = PRESSURE_MIN;
  #else
    currentValue[valve] = 0;
  #endif
}

/**
 * Turn off all valves -- regulators AND on/off solenoids.
 *
 * Every stop path funnels through here (the 's' command, vcEmergencyStop() for
 * the touchscreen E-STOP), so putting dv::allOff() in this one place is what
 * makes an emergency stop actually stop everything.
 */
void allValvesOff() {
  cal::abort();               // a sweep in progress must not re-drive the valve
  for (int i = 0; i < NUM_VALVES; i++) {
    valveOff(i);
  }
  dv::allOff();
}

/**
 * Get current value for a valve
 */
int getValveValue(int valve) {
  if (valve < 0 || valve >= NUM_VALVES) return 0;
  return currentValue[valve];
}

/**
 * Set all valves to the same value
 */
void setAllValves(int value) {
  for (int i = 0; i < NUM_VALVES; i++) {
    setValve(i, value);
  }
}

/**
 * Count active valves (non-zero output)
 */
int countActiveValves() {
  int count = 0;
  for (int i = 0; i < NUM_VALVES; i++) {
    if (currentValue[i] != 0) count++;
  }
  return count;
}

// ============================================================
// UI accessors (declared in valve_core.h)
// ------------------------------------------------------------
// Read-only view of valve state for the on-Giga display. These never touch the
// serial link or the DAC buses (except vcEmergencyStop, which is a deliberate
// safety action mirroring the 's' command).
// ============================================================

// Command mV -> kPa: 0..VC_CMD_MV_FS spans VC_KPA_MIN..VC_KPA_MAX. Exposed so
// the display can label the slider from the same formula the DAC path uses.
float vcMvToKpa(int mV) {
  return VC_KPA_MIN + mV * ((VC_KPA_MAX - VC_KPA_MIN) / (float)VC_CMD_MV_FS);
}

float vcValueKpa(int valve) {
  int v = getValveValue(valve);
  #if INPUT_PRESSURE_MODE
    return (float)v;                 // already kPa
  #else
    return vcMvToKpa(v);
  #endif
}

int vcValueMv(int valve) {
  return getValveValue(valve);            // currentValue is mV in voltage mode
}

int vcValveOn(int valve) {
  return getValveValue(valve) != 0 ? 1 : 0;
}

int vcActiveCount(void) {
  return countActiveValves();
}

// Real-time measured pressure from the AD7606 (channel N -> valve N). The
// regulator's monitor output is 1-5V, NOT the 0-10V the command side uses, so
// this needs its own slope: kPa = (V - 1) * 225.
bool vcHasMeasure(int valve) {
  return valve >= 0 && valve < ADC_NUM_CH;
}

float vcMeasuredKpa(int valve) {
  if (!vcHasMeasure(valve)) return 0.0f;
  // 1V => 0 kPa, 5V => 900 kPa. Left unclamped on purpose: a reading far below
  // VC_KPA_MIN means the monitor line is floating, and hiding that helps no one.
  return VC_KPA_MIN
       + (adc::volts(valve) - VC_FB_V_MIN)
         * ((VC_KPA_MAX - VC_KPA_MIN) / (VC_FB_V_MAX - VC_FB_V_MIN));
}

void vcSetValveMv(int valve, int mV) {
  setValve(valve, mV);                    // setValve clamps to the DAC range
}

// On/off solenoids, page 3 of the display. Thin pass-throughs to dv:: so
// ui_display.cpp needs no hardware header -- same boundary the regulators use.
bool vcOnOffGet(int idx) {
  return dv::get(idx);
}

int vcOnOffPin(int idx) {
  return dv::pin(idx);
}

void vcOnOffSet(int idx, bool on) {
  dv::set(idx, on);                       // dv::set ignores a bad index
}

// RS-485 length sensors, page 4 of the display. Pass-throughs to enc:: for the
// same reason as the solenoids: ui_display.cpp needs no hardware header.
int  vcSensorCount(void)     { return enc::count(); }
int  vcSensorId(int i)       { return enc::slaveId(i); }
bool vcSensorOnline(int i)   { return enc::online(i); }
bool vcSensorZeroed(int i)   { return enc::zeroed(i); }
long vcSensorUm(int i)       { return (long)enc::lengthUm(i); }
bool vcSensorScanning(void)  { return enc::scanning(); }
void vcSensorScan(void)      { enc::rescan(); }
void vcSensorZero(void)      { enc::zeroAll(); }

void vcEmergencyStop(void) {
  allValvesOff();
  Serial.println("EMERGENCY STOP - All valves OFF");  // keep the PC app in sync
}

// ============================================================
// Diagnostics
// ============================================================

/**
 * Report every address that ACKs on Wire. A DAC that fails to init is simply
 * one that did not answer, so this says whether it is missing entirely (wiring
 * / power / ground) or answering at an address we are not asking for (jumpers).
 */
void scanI2C() {
  Serial.println("I2C scan on Wire (SDA=D20, SCL=D21):");
  int found = 0;
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      found++;
      Serial.print("  0x");
      Serial.print(addr, HEX);
      if (addr >= DAC_ADDR_BASE && addr < DAC_ADDR_BASE + 8) {
        Serial.print("  (DAC range");
        if (addr < DAC_ADDR_BASE + NUM_DACS) Serial.print(", expected");
        else                                 Serial.print(", NOT used by this build");
        Serial.print(")");
      }
      Serial.println();
    }
  }
  if (found == 0)
    Serial.println("  nothing responded - check SDA=D20/SCL=D21, power, common ground");
  Serial.print("  expected 0x58..0x");
  Serial.println(DAC_ADDR_BASE + NUM_DACS - 1, HEX);
}

// ============================================================
// Serial Command Processing
// ============================================================

/**
 * Process serial commands
 * Format: valve,pressure  e.g. 0,3000 or 5,2100
 * Special commands:
 *   s or S - Emergency stop (all valves off)
 *   ?      - Print status of all valves
 *   p      - Ping
 */
void processSerialCommand() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        String cmdLower = inputBuffer;
        cmdLower.toLowerCase();

        if (cmdLower == "s") {
          allValvesOff();
          Serial.println("EMERGENCY STOP - All valves OFF");
          inputBuffer = "";
          return;
        }

        if (cmdLower == "?") {
          printStatus();
          inputBuffer = "";
          return;
        }

        if (cmdLower == "p") {
          Serial.println("PONG - Arduino connected");
          inputBuffer = "";
          return;
        }

        if (cmdLower == "a") {
          adc::printAll();          // dump all AD7606 channels
          inputBuffer = "";
          return;
        }

        if (cmdLower == "i") {
          scanI2C();                // which addresses actually answer
          inputBuffer = "";
          return;
        }

        // 'c<valve>' or 'c<valve>,<maxMv>' sweeps one regulator and prints
        // commanded vs measured. Checked before the 'e' family and before
        // parseCommand so a bare number is still a setpoint.
        if (cmdLower.length() > 1 && cmdLower[0] == 'c') {
          String rest = cmdLower.substring(1);
          int comma = rest.indexOf(',');
          int valve = (comma < 0 ? rest : rest.substring(0, comma)).toInt();
          int maxMv = 0, cycles = 1;
          if (comma >= 0) {
            String tail = rest.substring(comma + 1);
            int c2 = tail.indexOf(',');
            maxMv  = (c2 < 0 ? tail : tail.substring(0, c2)).toInt();
            if (c2 >= 0) cycles = tail.substring(c2 + 1).toInt();
          }
          cal::start(valve, maxMv, cycles);
          inputBuffer = "";
          return;
        }

        if (cmdLower == "k") {
          calibPrint();
          inputBuffer = "";
          return;
        }

        if (cmdLower == "e") {
          enc::printAll();
          inputBuffer = "";
          return;
        }

        if (cmdLower == "es") {
          enc::rescan();
          inputBuffer = "";
          return;
        }

        // 'es<lo>,<hi>' sweeps a different id range, e.g. es50,80 -- the
        // encoders' ids are wiring, not firmware, so this stays runtime.
        if (cmdLower.startsWith("es") && cmdLower.indexOf(',') > 2) {
          int comma = cmdLower.indexOf(',');
          int lo = cmdLower.substring(2, comma).toInt();
          int hi = cmdLower.substring(comma + 1).toInt();
          enc::rescan(lo, hi);
          inputBuffer = "";
          return;
        }

        // 'ep<n>' moves the RS-485 DE line to pin n (ep-1 = auto-direction).
        if (cmdLower.startsWith("ep") && cmdLower.length() > 2) {
          enc::setDePin(cmdLower.substring(2).toInt());
          inputBuffer = "";
          return;
        }

        if (cmdLower == "el") {
          enc::listen(3000);        // dump the bus unframed for 3s
          inputBuffer = "";
          return;
        }

        if (cmdLower == "et") {
          enc::loopbackTest();      // D18 jumpered to D19 must pass
          inputBuffer = "";
          return;
        }

        if (cmdLower == "ex") {
          enc::setHexDump(!enc::hexDump());
          inputBuffer = "";
          return;
        }

        if (cmdLower == "d" || cmdLower == "d?") {
          dv::printStatus();
          inputBuffer = "";
          return;
        }

        if (cmdLower == "d,off") {
          dv::allOff();
          Serial.println("OK: all on/off valves OFF");
          inputBuffer = "";
          return;
        }

        // Any other line starting with 'd' is an on/off-valve command. Checked
        // before parseCommand() so "d0,1" can never be read as regulator 0.
        if (cmdLower.length() > 1 && cmdLower[0] == 'd') {
          parseDigitalCommand(inputBuffer);
          inputBuffer = "";
          return;
        }

        parseCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
}

/**
 * Parse command in format: valve,value or multiple: v1,val1,v2,val2,...
 * Also supports valve,off
 */
void parseCommand(String cmd) {
  cmd.trim();

  int start = 0;
  int count = 0;

  while (start < (int)cmd.length()) {
    int comma1 = cmd.indexOf(',', start);
    if (comma1 == -1) break;

    int valve = cmd.substring(start, comma1).toInt();

    int comma2 = cmd.indexOf(',', comma1 + 1);
    String pressureStr;
    if (comma2 == -1) {
      pressureStr = cmd.substring(comma1 + 1);
      start = cmd.length();
    } else {
      pressureStr = cmd.substring(comma1 + 1, comma2);
      start = comma2 + 1;
    }
    pressureStr.trim();

    if (pressureStr.equalsIgnoreCase("off")) {
      valveOff(valve);
      Serial.print("OK: V");
      Serial.print(valve);
      Serial.print(" OFF");
    } else {
      int pressure = pressureStr.toInt();
      if (setValve(valve, pressure)) {
        Serial.print("OK: V");
        Serial.print(valve);
        Serial.print("=");
        Serial.print(pressure);
      }
    }
    count++;
    if (start < (int)cmd.length()) Serial.print(" | ");
  }

  if (count > 0) {
    Serial.println();
  } else {
    Serial.println("ERROR: Use valve,value or v1,val1,v2,val2,...");
  }
}

/**
 * Parse an on/off-valve line: dN,V where V is 1/0/on/off. Several pairs may
 * share a line: d0,1,d3,0
 *
 * The 'd' rides on the index token rather than the line as a whole, so a mixed
 * or malformed line still names the valve it is talking about, and a dropped
 * character cannot silently turn "d0,1" into regulator 0 at 1 mV.
 */
void parseDigitalCommand(String cmd) {
  cmd.trim();

  int start = 0;
  int count = 0;

  while (start < (int)cmd.length()) {
    int comma1 = cmd.indexOf(',', start);
    if (comma1 == -1) break;

    String idxTok = cmd.substring(start, comma1);
    idxTok.trim();
    if (idxTok.length() > 0 && (idxTok[0] == 'd' || idxTok[0] == 'D')) {
      idxTok = idxTok.substring(1);
    }
    int valve = idxTok.toInt();

    int comma2 = cmd.indexOf(',', comma1 + 1);
    String valTok;
    if (comma2 == -1) {
      valTok = cmd.substring(comma1 + 1);
      start = cmd.length();
    } else {
      valTok = cmd.substring(comma1 + 1, comma2);
      start = comma2 + 1;
    }
    valTok.trim();

    // Strict: an unrecognised token is rejected, not coerced. toInt() would
    // turn any typo into 0 == OFF, which is safe but silent, and a command
    // that looks accepted while doing nothing is worse than one that errors.
    bool on;
    if (valTok == "1" || valTok.equalsIgnoreCase("on")) {
      on = true;
    } else if (valTok == "0" || valTok.equalsIgnoreCase("off")) {
      on = false;
    } else {
      Serial.print("ERROR: D");
      Serial.print(valve);
      Serial.print(" bad value '");
      Serial.print(valTok);
      Serial.println("' - use 0/1/on/off");
      count++;
      continue;
    }

    if (dv::set(valve, on)) {
      Serial.print("OK: D");
      Serial.print(valve);
      Serial.print("=");
      Serial.print(on ? 1 : 0);
    } else {
      Serial.print("ERROR: Invalid on/off valve index ");
      Serial.print(valve);
    }

    count++;
    if (start < (int)cmd.length()) Serial.print(" | ");
  }

  if (count > 0) {
    Serial.println();
  } else {
    Serial.println("ERROR: Use dN,V with V = 0/1/on/off, e.g. d0,1");
  }
}

/**
 * Print status of all valves
 */
void printStatus() {
  #if INPUT_PRESSURE_MODE
    Serial.println("=== Status (kPa) ===");
  #else
    Serial.println("=== Status (mV) ===");
  #endif
  for (int i = 0; i < NUM_VALVES; i++) {
    Serial.print("V");
    Serial.print(i);
    Serial.print("=");
    Serial.print(currentValue[i]);
    if (i < NUM_VALVES - 1) Serial.print(" | ");
  }
  Serial.println();
}

// ============================================================
// SETUP & LOOP
// ============================================================

void setup() {
  Serial.begin(115200);

  // First, ahead of the 1s serial settle and every bus bring-up: park the six
  // on/off solenoids. Until pinMode() runs, D2..D7 are inputs and the driver
  // board's own pull-ups hold the channels off, so the sooner this claims the
  // pins at the OFF level the shorter the window in which anything else could.
  dv::begin();

  delay(1000);

  Serial.println(F("\n========================================"));
  Serial.println(F("  6-Valve Controller (Compact)"));
  Serial.println(F("========================================"));

  Wire.begin();   // valves 0..5 on the default bus (D20/D21)

  buildMapping();

  if (initValves()) {
    Serial.println("All DACs initialized successfully!");
  } else {
    Serial.println("WARNING: Some DACs failed to initialize");
    scanI2C();      // show what IS on the bus, so the gap is obvious
  }

  #if INPUT_PRESSURE_MODE
    Serial.println("Mode: PRESSURE (kPa)");
    Serial.println("Device range: 0 to 900 kPa (0-10V command)");
    Serial.println("Safety limit: 0 to 900 kPa");
  #else
    Serial.println("Mode: VOLTAGE (mV)");
    Serial.println("Range: 0 to 10000 mV");
  #endif
  Serial.println("Layout: V0-V5 on Wire (DAC 0x58-0x5A)");
  Serial.println("Regulator: cmd 0-10V = 0-900 kPa | monitor 1-5V = 0-900 kPa");
  Serial.print("On/off valves: D0-D5 on pins D2-D7, all OFF (driver active-");
  Serial.print(DV_ACTIVE_LOW ? "LOW" : "HIGH");
  Serial.println(")");
  Serial.println("Commands: valve,value | dN,0|1 | s=stop | ?=status | d?=on/off status");
  Serial.println("          e=encoders | es=rescan | el=listen | et=loopback");
  Serial.println("          ep<n>=DE pin | ex=hex dump");
  Serial.println("          c<v>[,maxMv[,cycles]]=cal sweep, e.g. c0,2200,3");
  Serial.println("          k=show calibration tables");
  Serial.println("          p=ping | a=ADC dump | i=I2C scan");

  // Bring up the touchscreen UI last, so valve state already reflects a clean
  // start. ui::tick() below is non-blocking and never delays serial handling.
  ui::begin();
  Serial.println("Display: GIGA shield UI up (2 reg pages + on/off page, LOCK, E-STOP)");

  // AD7606 analog inputs on SPI. begin() reports each board present/absent.
  adc::begin();
  Serial.print("ADC: AD7606 up (");
  Serial.print(ADC_NUM_BOARDS);
  Serial.print(" board(s), ");
  Serial.print(ADC_NUM_CH);
  Serial.print(" ch @ ");
  Serial.print(ADC_SAMPLE_HZ);
  Serial.println("Hz) - type 'a' for readings");

  // RS-485 encoders on D18/D19. begin() only opens the port and arms the scan;
  // the sweep itself runs in tick(), so setup() never waits on the bus.
  enc::begin();
  Serial.print("Encoders: MAX485 on pins D18/D19 (Serial2), DE D");
  Serial.print(ENC_DE_PIN);
  Serial.print(", scanning ids ");
  Serial.print(ENC_SCAN_LO); Serial.print("-"); Serial.print(ENC_SCAN_HI);
  Serial.println(" - type 'e' for readings");
}

void loop() {
  processSerialCommand();   // serial ALWAYS has priority, every iteration
  adc::tick();              // cooperative, non-blocking AD7606 sampling
  enc::tick();              // cooperative, non-blocking RS-485 Modbus polling
  cal::tick();              // cooperative, non-blocking calibration sweep
  ui::tick();               // cooperative, non-blocking display + touch
}
