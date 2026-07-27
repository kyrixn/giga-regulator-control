/**
 * vc2.ino
 *
 * 16-Valve Controller (Feedforward) for Arduino Giga R1 - dual I2C bus.
 *
 * Hardware:
 *   Wire  (SDA/SCL):    4 GP8403 DACs at 0x58..0x5B  -> valves 0..7
 *   Wire1 (SDA1/SCL1):  4 GP8403 DACs at 0x58..0x5B  -> valves 8..15
 *   DAC output: 0-10V
 *
 * Valve-to-DAC mapping (2 channels per DAC):
 *   V0,V1   = Wire  0x58 ch0,ch1     V8 ,V9   = Wire1 0x58 ch0,ch1
 *   V2,V3   = Wire  0x59 ch0,ch1     V10,V11  = Wire1 0x59 ch0,ch1
 *   V4,V5   = Wire  0x5A ch0,ch1     V12,V13  = Wire1 0x5A ch0,ch1
 *   V6,V7   = Wire  0x5B ch0,ch1     V14,V15  = Wire1 0x5B ch0,ch1
 *
 * Commands (Serial @ 115200):
 *   valve,value        Set single valve: 0,3000 or 9,2100
 *   v1,val1,v2,val2,.. Set multiple valves: 0,3000,9,2500
 *   valve,off          Turn off a valve: 9,off
 *   s                  Emergency stop (all 16 valves off)
 *   ?                  Query status of all valves
 *   p                  Ping test
 */

#include <Arduino.h>
#include <Wire.h>
#include "DFRobot_GP8403.h"

#define NUM_VALVES 16

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

// Wire (valves 0..7)
DFRobot_GP8403 dac0_58(&Wire, 0x58);
DFRobot_GP8403 dac0_59(&Wire, 0x59);
DFRobot_GP8403 dac0_5A(&Wire, 0x5A);
DFRobot_GP8403 dac0_5B(&Wire, 0x5B);

// Wire1 (valves 8..15)
DFRobot_GP8403 dac1_58(&Wire1, 0x58);
DFRobot_GP8403 dac1_59(&Wire1, 0x59);
DFRobot_GP8403 dac1_5A(&Wire1, 0x5A);
DFRobot_GP8403 dac1_5B(&Wire1, 0x5B);

// Mapping arrays: valve index -> DAC pointer and channel
DFRobot_GP8403* dacs[NUM_VALVES] = {
  &dac0_58, &dac0_58,  // Valves 0, 1
  &dac0_59, &dac0_59,  // Valves 2, 3
  &dac0_5A, &dac0_5A,  // Valves 4, 5
  &dac0_5B, &dac0_5B,  // Valves 6, 7
  &dac1_58, &dac1_58,  // Valves 8, 9
  &dac1_59, &dac1_59,  // Valves 10, 11
  &dac1_5A, &dac1_5A,  // Valves 12, 13
  &dac1_5B, &dac1_5B   // Valves 14, 15
};

int dacChannels[NUM_VALVES] = {
  0, 1, 0, 1, 0, 1, 0, 1,
  0, 1, 0, 1, 0, 1, 0, 1
};

// Track current value for each valve (pressure in kPa or voltage in mV)
int currentValue[NUM_VALVES] = {0};

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

  // Initialize each unique DAC (indices 0, 2, 4, ..., 14)
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
 * @param valve Valve index (0-15)
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
// Serial Command Processing
// ============================================================

String inputBuffer = "";

/**
 * Process serial commands
 * Format: valve,pressure  e.g. 0,3000 or 9,2100
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
  Serial.println(F("  16-Valve Controller (Dual-Bus)"));
  Serial.println(F("========================================"));

  Wire.begin();
  Wire1.begin();

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
  Serial.println("Layout: V0-V7 on Wire, V8-V15 on Wire1");
  Serial.println("Commands: valve,value | s=stop | ?=status | p=ping");
}

void loop() {
  processSerialCommand();
}
