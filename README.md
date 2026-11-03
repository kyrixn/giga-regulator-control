# giga-regulator-control

Firmware for the 32-regulator pneumatic station, on an Arduino GIGA R1 with the GIGA Display Shield. Write your own control script against the serial commands below.

| Branch | Use it for |
|---|---|
| `main` | Full system, with up to two BNO055 IMUs |
| `no_imu` | The same firmware without IMU support |
| `compact6R` | Separate 6-regulator hardware build |

## Hardware

- **Outputs:** 16 GP8403 DACs, 0–10 V. V0–V15 are on Wire, V16–V31 on Wire2.
- **Analog inputs:** 4 AD7606 boards, 32 channels. Wiring is in [`pinmapping`](pinmapping).
- **IMUs (optional):** BNO055s on Wire1, at 0x28 (ADR low) and 0x29 (ADR tied to 3V3).

## Build and flash

1. Clone into a folder named `vc2`, as Arduino requires:
   ```
   git clone https://github.com/kyrixn/giga-regulator-control.git vc2
   ```
2. Install the core `arduino:mbed_giga` (tested on 4.6.0) and the libraries `DFRobot_GP8403` (1.0.0), `Arduino_GigaDisplay_GFX` (1.2.0) and `Arduino_GigaDisplayTouch` (1.1.2).
3. Patch `DFRobot_GP8403.h`, because the GIGA core defines no `SCL` or `SDA`. Library updates undo this.
   ```cpp
   int _scl = PIN_WIRE_SCL;   // was SCL
   int _sda = PIN_WIRE_SDA;   // was SDA
   ```
4. Build and flash from the Arduino IDE, or with:
   ```
   arduino-cli compile --fqbn arduino:mbed_giga:giga vc2
   arduino-cli upload --fqbn arduino:mbed_giga:giga -p <port> vc2
   ```

Close anything using the serial port first. Flashing resets the board and all valves vent.

## Serial commands

115200 baud. End each command with a newline.

| Command | Action |
|---|---|
| `valve,value` | Set one valve, e.g. `0,3000` (replies `OK: V0=3000`) |
| `v1,val1,v2,val2,...` | Set several valves |
| `valve,off` | Turn one valve off |
| `s` | Emergency stop: all valves off |
| `?` | Print all valve values |
| `p` | Ping (replies `PONG`) |
| `a` | Print all ADC channels: `ADC seq=N C0=raw(volts) ...` |
| `i1` / `i0` | Start / stop the 45 Hz IMU stream (frame format in `imu_bno055.h`) |
| `i` | Print one reading per IMU |

Values are in mV, 0–10000, where `kPa = mV × 0.06 − 100`. For kPa input, set `INPUT_PRESSURE_MODE` to `true` in `vc2.ino`.

**Set your own pressure limit in your script.** The firmware only clamps to its full range, so 10000 mV commands 500 kPa.

**Serial can be switched off from the touchscreen.** The MODE button toggles between MONITOR and STANDALONE, and STANDALONE ignores every serial command, including `s`. Use the on-screen E-STOP there.

## Example scripts

`vc2_control.py` (terminal) and `vc2_gui.py` (buttons and text boxes) need `pyserial` and `matplotlib`. Both refuse values above `MAX_INPUT_VALUE` in `vc2_control.py`, currently 5000 mV (200 kPa).

```
python vc2_gui.py [port]
```
