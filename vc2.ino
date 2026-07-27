/**
 * vc2.ino
 *
 * 32-Valve Controller (Feedforward) for Arduino Giga R1 - dual I2C bus.
 *
 * Hardware:
 *   Wire  (SDA/SCL,   D20/D21): 8 GP8403 DACs at 0x58..0x5F -> valves 0..15
 *   Wire2 (SDA2/SCL2, D8/D9):   8 GP8403 DACs at 0x58..0x5F -> valves 16..31
 *   DAC output: 0-10V   (2 channels per DAC -> 2 valves per DAC)
 *
 *   NOTE: bank B is on Wire2 (not Wire1) so Wire1 is left free for the GIGA
 *   Display Shield's GT911 touch controller (0x5D on Wire1, which would clash
 *   with a DAC). Wire2 has NO internal pull-ups -- the DAC boards' on-board
 *   pull-ups usually suffice; add ~4.7k to 3V3 on D8/D9 if the bus is flaky.
 *
 * 16 DACs total (8 per bus), 32 regulators total.
 *
 * Valve-to-DAC mapping (2 channels per DAC):
 *   Bus Wire   DAC 0x58 -> V0 ,V1     Bus Wire2  DAC 0x58 -> V16,V17
 *              DAC 0x59 -> V2 ,V3                DAC 0x59 -> V18,V19
 *              DAC 0x5A -> V4 ,V5                DAC 0x5A -> V20,V21
 *              DAC 0x5B -> V6 ,V7                DAC 0x5B -> V22,V23
 *              DAC 0x5C -> V8 ,V9                DAC 0x5C -> V24,V25
 *              DAC 0x5D -> V10,V11               DAC 0x5D -> V26,V27
 *              DAC 0x5E -> V12,V13               DAC 0x5E -> V28,V29
 *              DAC 0x5F -> V14,V15               DAC 0x5F -> V30,V31
 *
 * Commands (Serial @ 115200):
 *   valve,value        Set single valve: 0,3000 or 20,2100
 *   v1,val1,v2,val2,.. Set multiple valves: 0,3000,20,2500
 *   valve,off          Turn off a valve: 20,off
 *   s                  Emergency stop (all 32 valves off)
 *   ?                  Query status of all valves
 *   p                  Ping test
 */

#include <Arduino.h>
#include <Wire.h>
#include "DFRobot_GP8403.h"

#include "valve_core.h"   // read-only accessors exposed to the display UI
#include "ui_display.h"   // on-Giga touchscreen UI (non-blocking)

#define DACS_PER_BUS 8
#define NUM_DACS     (DACS_PER_BUS * 2)   // 16 DACs across both buses
#define NUM_VALVES   (NUM_DACS * 2)       // 32 valves (2 channels per DAC)

static_assert(NUM_VALVES == VC_NUM_VALVES,
              "valve_core.h VC_NUM_VALVES must match NUM_VALVES");

// First contiguous I2C address of the DACs on each bus (0x58..0x5F).
#define DAC_ADDR_BASE 0x58

// ============================================================
// INPUT MODE: Change this flag to switch input format
// ============================================================
// true  = PRESSURE mode: input in kPa, mapped to 0-10V
// false = VOLTAGE mode:  input in mV (0 to 10000), direct output
#define INPUT_PRESSURE_MODE false

// Pressure range in kPa (only used in PRESSURE mode)
// Device mapping: 0-10V corresponds to -100 to 500 kPa
#define PRESSURE_MIN -100
#define PRESSURE_RANGE_MAX 500

// Safety limit: output pressure should not exceed this value
#define PRESSURE_SAFETY_MAX 500

// Voltage range in mV (only used in VOLTAGE mode)
#define VOLTAGE_MIN 0
#define VOLTAGE_MAX 10000

// ============================================================
// DAC instances
// ============================================================
// One object per physical DAC. Bus 0 (Wire) drives valves 0..15,
// bus 1 (Wire2) drives valves 16..31. Addresses run 0x58..0x5F on
// each bus. Wire1 is intentionally NOT used here -- it belongs to the
// display shield's touch controller.

DFRobot_GP8403 dacBus0[DACS_PER_BUS] = {
  DFRobot_GP8403(&Wire, DAC_ADDR_BASE + 0), DFRobot_GP8403(&Wire, DAC_ADDR_BASE + 1),
  DFRobot_GP8403(&Wire, DAC_ADDR_BASE + 2), DFRobot_GP8403(&Wire, DAC_ADDR_BASE + 3),
  DFRobot_GP8403(&Wire, DAC_ADDR_BASE + 4), DFRobot_GP8403(&Wire, DAC_ADDR_BASE + 5),
  DFRobot_GP8403(&Wire, DAC_ADDR_BASE + 6), DFRobot_GP8403(&Wire, DAC_ADDR_BASE + 7)
};

DFRobot_GP8403 dacBus1[DACS_PER_BUS] = {
  DFRobot_GP8403(&Wire2, DAC_ADDR_BASE + 0), DFRobot_GP8403(&Wire2, DAC_ADDR_BASE + 1),
  DFRobot_GP8403(&Wire2, DAC_ADDR_BASE + 2), DFRobot_GP8403(&Wire2, DAC_ADDR_BASE + 3),
  DFRobot_GP8403(&Wire2, DAC_ADDR_BASE + 4), DFRobot_GP8403(&Wire2, DAC_ADDR_BASE + 5),
  DFRobot_GP8403(&Wire2, DAC_ADDR_BASE + 6), DFRobot_GP8403(&Wire2, DAC_ADDR_BASE + 7)
};

// Mapping arrays: valve index -> DAC pointer and channel.
// Filled once in buildMapping(). Even valve indices are channel 0 of a
// DAC, odd indices channel 1, so unique DACs sit at even indices.
DFRobot_GP8403* dacs[NUM_VALVES];
int dacChannels[NUM_VALVES];

// Track current value for each valve (pressure in kPa or voltage in mV)
int currentValue[NUM_VALVES] = {0};

// Serial input buffer, and a flag the display UI sets to make the serial
// handler drain-and-discard incoming commands (standalone mode). Declared here
// so the UI accessors below can reference them.
String inputBuffer = "";
static bool g_ignoreSerial = false;

/**
 * Build the valve -> (DAC, channel) mapping.
 *   valves 0..15  -> dacBus0[0..7], channels 0/1
 *   valves 16..31 -> dacBus1[0..7], channels 0/1
 */
void buildMapping() {
  for (int d = 0; d < DACS_PER_BUS; d++) {
    int v0 = d * 2;                 // bus 0 valves 0..15
    dacs[v0]     = &dacBus0[d]; dacChannels[v0]     = 0;
    dacs[v0 + 1] = &dacBus0[d]; dacChannels[v0 + 1] = 1;

    int v1 = 16 + d * 2;            // bus 1 valves 16..31
    dacs[v1]     = &dacBus1[d]; dacChannels[v1]     = 0;
    dacs[v1 + 1] = &dacBus1[d]; dacChannels[v1 + 1] = 1;
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
    if (dacs[i]->begin() != 0) {
      Serial.print("ERROR: DAC init failed for valves ");
      Serial.print(i);
      Serial.print(" and ");
      Serial.println(i + 1);
      success = false;
    } else {
      dacs[i]->setDACOutRange(DFRobot_GP8403::eOutputRange10V);
      Serial.print("DAC initialized for valves ");
      Serial.print(i);
      Serial.print(" and ");
      Serial.println(i + 1);
    }
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
 * @param valve Valve index (0-31)
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

  dacs[valve]->setDACOutVoltage(mV, dacChannels[valve]);
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
 * Turn off all valves
 */
void allValvesOff() {
  for (int i = 0; i < NUM_VALVES; i++) {
    valveOff(i);
  }
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

float vcValueKpa(int valve) {
  int v = getValveValue(valve);
  #if INPUT_PRESSURE_MODE
    return (float)v;                 // already kPa
  #else
    return v * 0.06f - 100.0f;       // mV -> kPa  (0-10V => -100..500)
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

void vcSetValveMv(int valve, int mV) {
  setValve(valve, mV);                    // setValve clamps to the DAC range
}

void vcEmergencyStop(void) {
  allValvesOff();
  Serial.println("EMERGENCY STOP - All valves OFF");  // keep the PC app in sync
}

void vcSetSerialIgnore(bool ignore) {
  g_ignoreSerial = ignore;
  inputBuffer = "";
  Serial.println(ignore ? "STANDALONE MODE - serial ignored"
                        : "MONITOR MODE - serial active");
}

// ============================================================
// Serial Command Processing
// ============================================================

/**
 * Process serial commands
 * Format: valve,pressure  e.g. 0,3000 or 20,2100
 * Special commands:
 *   s or S - Emergency stop (all valves off)
 *   ?      - Print status of all valves
 *   p      - Ping
 */
void processSerialCommand() {
  while (Serial.available()) {
    char c = Serial.read();

    // Standalone mode: keep the RX buffer drained but ignore the content.
    if (g_ignoreSerial) {
      inputBuffer = "";
      continue;
    }

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
  delay(1000);

  Serial.println(F("\n========================================"));
  Serial.println(F("  32-Valve Controller (Dual-Bus)"));
  Serial.println(F("========================================"));

  Wire.begin();   // bank A: valves 0..15  (D20/D21)
  Wire2.begin();  // bank B: valves 16..31 (D8/D9; Wire1 reserved for touch)

  buildMapping();

  if (initValves()) {
    Serial.println("All DACs initialized successfully!");
  } else {
    Serial.println("WARNING: Some DACs failed to initialize");
  }

  #if INPUT_PRESSURE_MODE
    Serial.println("Mode: PRESSURE (kPa)");
    Serial.println("Device range: -100 to 500 kPa (0-10V)");
    Serial.println("Safety limit: -100 to 500 kPa");
  #else
    Serial.println("Mode: VOLTAGE (mV)");
    Serial.println("Range: 0 to 10000 mV");
  #endif
  Serial.println("Layout: V0-V15 on Wire, V16-V31 on Wire2");
  Serial.println("Commands: valve,value | s=stop | ?=status | p=ping");

  // Bring up the touchscreen UI last, so valve state already reflects a clean
  // start. ui::tick() below is non-blocking and never delays serial handling.
  ui::begin();
  Serial.println("Display: GIGA shield UI up (4 windows, kPa, E-STOP)");
}

void loop() {
  processSerialCommand();   // serial ALWAYS has priority, every iteration
  ui::tick();               // cooperative, non-blocking display + touch
}
