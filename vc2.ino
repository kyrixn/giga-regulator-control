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
 * Analog feedback: one AD7606 board (board 0), channels 0..5 -> valves 0..5.
 *
 * Commands (Serial @ 115200):
 *   valve,value        Set single valve: 0,3000 or 5,2100
 *   v1,val1,v2,val2,.. Set multiple valves: 0,3000,4,2500
 *   valve,off          Turn off a valve: 4,off
 *   s                  Emergency stop (all 6 valves off)
 *   ?                  Query status of all valves
 *   p                  Ping test
 */

#include <Arduino.h>
#include <Wire.h>
#include "DFRobot_GP8403.h"

#include "valve_core.h"   // read-only accessors exposed to the display UI
#include "ui_display.h"   // on-Giga touchscreen UI (non-blocking)
#include "adc_ad7606.h"   // AD7606 analog acquisition on SPI (non-blocking)

#define NUM_DACS     3                    // 0x58..0x5A on Wire
#define NUM_VALVES   (NUM_DACS * 2)       // 6 valves (2 channels per DAC)

static_assert(NUM_VALVES == VC_NUM_VALVES,
              "valve_core.h VC_NUM_VALVES must match NUM_VALVES");

// First contiguous I2C address of the DACs on the bus (0x58..0x5A).
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

// Serial input buffer, and a flag the display UI sets to make the serial
// handler drain-and-discard incoming commands (standalone mode). Declared here
// so the UI accessors below can reference them.
String inputBuffer = "";
static bool g_ignoreSerial = false;

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

// Real-time measured pressure from the AD7606 (channel N -> valve N). The analog
// output is 0-10V with the same mapping the DAC uses: kPa = V*60 - 100.
bool vcHasMeasure(int valve) {
  return valve >= 0 && valve < ADC_NUM_CH;
}

float vcMeasuredKpa(int valve) {
  if (!vcHasMeasure(valve)) return 0.0f;
  return adc::volts(valve) * 60.0f - 100.0f;   // 0V=>-100kPa, 10V=>500kPa
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
 * Format: valve,pressure  e.g. 0,3000 or 5,2100
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

        if (cmdLower == "a") {
          adc::printAll();          // dump all AD7606 channels
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
  Serial.println(F("  6-Valve Controller (Compact)"));
  Serial.println(F("========================================"));

  Wire.begin();   // valves 0..5 on the default bus (D20/D21)

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
  Serial.println("Layout: V0-V5 on Wire (DAC 0x58-0x5A)");
  Serial.println("Commands: valve,value | s=stop | ?=status | p=ping");

  // Bring up the touchscreen UI last, so valve state already reflects a clean
  // start. ui::tick() below is non-blocking and never delays serial handling.
  ui::begin();
  Serial.println("Display: GIGA shield UI up (kPa, E-STOP)");

  // AD7606 analog inputs on SPI. begin() reports each board present/absent.
  adc::begin();
  Serial.print("ADC: AD7606 up (");
  Serial.print(ADC_NUM_BOARDS);
  Serial.print(" board(s), ");
  Serial.print(ADC_NUM_CH);
  Serial.print(" ch @ ");
  Serial.print(ADC_SAMPLE_HZ);
  Serial.println("Hz) - type 'a' for readings");
}

void loop() {
  processSerialCommand();   // serial ALWAYS has priority, every iteration
  adc::tick();              // cooperative, non-blocking AD7606 sampling
  ui::tick();               // cooperative, non-blocking display + touch
}
