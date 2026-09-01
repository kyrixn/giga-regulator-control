#!/usr/bin/env python3
"""
vc2_control.py

6-Valve Controller - Serial Communication Script with Matplotlib Display

Controls Arduino vc2 sketch (compact build) for direct voltage/pressure
control over 3 DACs (0x58..0x5A, 2 channels each = 6 valves):
  - Valves 0..5 on Wire (SDA/SCL)

Arduino mode (set in Arduino code):
  PRESSURE mode: input in kPa (0 to 900)
  VOLTAGE mode:  input in mV (0 to 10000)

Commands:
  Proportional regulators (indices 0..5):
    valve,value          - Set single valve: 0,900 or 5,450
    v1,val1,v2,val2,...  - Set multiple valves: 0,900,4,450
    valve,off            - Turn off a valve: 4,off
    ?                    - Query status of all regulators

  On/off solenoids (number = Giga pin, d2..d7 == pins D2..D7):
    dN,V                 - Set one: d2,1  d2,on  d2,0  d2,off
    dN,V,dM,V,...        - Set several: d2,1,d5,0
    d,off                - Release every on/off valve
    d  or  d?            - Query status of all on/off valves

  s                      - Emergency stop (regulators AND solenoids)

  Info:
    p                    - Ping test
    h                    - Show help
    q                    - Quit program

Usage:
  python vc2_control.py [port]
  python vc2_control.py /dev/ttyACM0
"""

import serial
import serial.tools.list_ports
import sys
import threading
import time
import re
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


NUM_VALVES = 6
ROW_SIZE = 6
NUM_ROWS = (NUM_VALVES + ROW_SIZE - 1) // ROW_SIZE
# Regulator (compact rig): command 0-10V -> 0..900 kPa; monitor 1-5V -> same
# span. Linearity +/-1% F.S. = +/-9 kPa.
KPA_MIN     = 0.0
KPA_MAX     = 900.0
CMD_MV_FS   = 10000     # command mV at KPA_MAX

# SAFETY: chosen in PRESSURE and converted to mV, so revisit it whenever
# KPA_MAX / CMD_MV_FS change. At 0.09 kPa/mV the regulator's full 10000 mV
# would be 900 kPa; this ceiling deliberately stops at ~200 kPa.
MAX_INPUT_VALUE = 2200  # mV or kPa — never send values above this (198 kPa)

# On/off solenoids on the opto-isolated MOSFET board. The user-facing number
# IS the Giga pin: 'd2' drives pin D2 and 'd7' drives pin D7. Internally (and
# in the firmware's dv::) they are still indices 0..5, so ONOFF_PIN0 is the
# protocol offset. Firmware side: DV_PIN_FIRST in valves_onoff.h.
NUM_ONOFF   = 6
ONOFF_PIN0  = 2         # internal index 0 <-> d2/D2 on the wire and screen


def bus_name(valve):
    """Return the I2C bus label for a valve index (single bus in this build)."""
    return 'Wire'


def row_title(v_lo, v_hi):
    """Human-readable title for a row spanning valves v_lo..v_hi."""
    return f'Valves {v_lo}-{v_hi}  ({bus_name(v_lo)})'


class ValveController:
    """6-Valve serial interface with matplotlib display"""

    def __init__(self, port=None, baudrate=115200):
        self.ser = None
        self.baudrate = baudrate
        self.running = False
        self.read_thread = None

        # Valve readings (commanded mV)
        self.valve_data = {}  # {valve_id: mV}

        # On/off solenoid states, {index: bool}. Absent == not yet known; the
        # board offers no readback, so this only ever reflects acks we saw.
        self.onoff_data = {}

        # Display lock
        self.display_lock = threading.Lock()

        # Message queue for terminal output
        self.message_queue = deque(maxlen=20)

        if port:
            self.connect(port)
        else:
            self.auto_connect()

    def auto_connect(self):
        """Try to auto-detect and connect to Arduino"""
        ports = serial.tools.list_ports.comports()

        print("\nAvailable serial ports:")
        for i, port in enumerate(ports):
            print(f"  [{i}] {port.device} - {port.description}")

        if not ports:
            print("[ERR] No serial ports found!")
            return False

        for port in ports:
            if 'ACM' in port.device or 'USB' in port.device or 'Arduino' in port.description:
                try:
                    print(f"\nTrying {port.device}...")
                    self.connect(port.device)
                    return True
                except Exception as e:
                    print(f"  Failed: {e}")

        if ports:
            choice = input("\nEnter port number or path: ").strip()
            try:
                idx = int(choice)
                self.connect(ports[idx].device)
                return True
            except ValueError:
                self.connect(choice)
                return True
            except Exception as e:
                print(f"[ERR] Connection failed: {e}")
                return False

        return False

    def connect(self, port):
        """Connect to specified serial port"""
        self.ser = serial.Serial(port, self.baudrate, timeout=0.1)
        time.sleep(2)  # Wait for Arduino reset
        print(f"[OK] Connected to {port}")

        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()

    def _read_loop(self):
        """Background thread to read serial responses"""
        while self.running:
            try:
                if self.ser and self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self._handle_response(line)
            except Exception as e:
                if self.running:
                    self.message_queue.append(f"[ERR] Read error: {e}")
            time.sleep(0.005)

    def _handle_response(self, line):
        """Process Arduino responses"""
        # Set/off acknowledgments - parse to keep display in sync
        if line.startswith("OK:"):
            # "V<i>=<mV>" -> update display
            for m in re.findall(r'V(\d+)=(-?\d+)', line):
                v = int(m[0])
                mV = int(m[1])
                with self.display_lock:
                    self.valve_data[v] = mV

            # "V<i> OFF" -> remove from display
            for m in re.findall(r'V(\d+)\s+OFF', line):
                v = int(m)
                with self.display_lock:
                    self.valve_data.pop(v, None)

            # "D<pin>=<0|1>" -> on/off solenoid ack. Deliberately a separate
            # letter from V so neither regex can ever match the other's acks.
            # The wire carries pin numbers (D2..D7); onoff_data keys stay 0..5.
            for m in re.findall(r'D(\d+)=([01])', line):
                with self.display_lock:
                    self.onoff_data[int(m[0]) - ONOFF_PIN0] = (m[1] == '1')

            if "all on/off valves OFF" in line:
                with self.display_lock:
                    self.onoff_data = {i: False for i in range(NUM_ONOFF)}

            self.message_queue.append(line)
            return

        # Status query response: "V0=val | V1=val | ... | V15=val"
        if line.startswith("V") and "=" in line and "|" in line:
            matches = re.findall(r'V(\d+)=(-?\d+)', line)
            with self.display_lock:
                self.valve_data.clear()
                for m in matches:
                    v = int(m[0])
                    val = int(m[1])
                    if val != 0:
                        self.valve_data[v] = val
            self.message_queue.append(line)
            return

        # On/off status response: "D2=0 | D3=1 | ... | D7=0" (pin numbers)
        if line.startswith("D") and "=" in line and "|" in line:
            with self.display_lock:
                self.onoff_data = {
                    int(m[0]) - ONOFF_PIN0: (m[1] == '1')
                    for m in re.findall(r'D(\d+)=([01])', line)
                }
            self.message_queue.append(line)
            return

        if line.startswith("EMERGENCY"):
            with self.display_lock:
                self.valve_data.clear()
                self.onoff_data = {i: False for i in range(NUM_ONOFF)}
            self.message_queue.append(f"** {line} **")
            return

        if line.startswith("PONG"):
            self.message_queue.append(line)
            return

        if line.startswith("===") or line.startswith("---"):
            self.message_queue.append(line)
            return

        if "ERROR" in line:
            self.message_queue.append(line)
            return

        # Boot banner / info messages
        if (line.startswith("On/off valves")
                or line.startswith("Display:")
                or line.startswith("ADC:")
                or line.startswith("Mode:")
                or line.startswith("Range:")
                or line.startswith("Device")
                or line.startswith("Layout:")
                or line.startswith("Commands:")
                or line.startswith("Safety")
                or line.startswith("DAC initialized")
                or line.startswith("All DACs")
                or line.startswith("WARNING")):
            self.message_queue.append(line)

    def send(self, command):
        """Send command to Arduino"""
        if not self.ser:
            self.message_queue.append("[ERR] Not connected!")
            return False

        try:
            self.ser.write(f"{command}\n".encode())
            return True
        except Exception as e:
            self.message_queue.append(f"[ERR] Send error: {e}")
            return False

    # ==================== CONTROL COMMANDS ====================

    def _value_within_limit(self, value):
        """Return True if value is allowed to be sent to the hardware."""
        if value > MAX_INPUT_VALUE:
            self.message_queue.append(
                f"[ERR] Value {value} exceeds limit (max {MAX_INPUT_VALUE})"
            )
            return False
        return True

    def set_valve(self, valve, value):
        """Set valve to specified value (pressure in kPa or voltage in mV)"""
        if not self._value_within_limit(value):
            return False
        return self.send(f"{valve},{value}")

    def set_multiple_valves(self, valve_value_pairs):
        """Set multiple valves in a single batched command.

        valve_value_pairs: list of (valve, value) where value is either an int
        (mV / kPa) or the string 'off'.
        """
        for _, val in valve_value_pairs:
            if val == 'off':
                continue
            if not self._value_within_limit(val):
                return False
        cmd = ",".join(f"{v},{val}" for v, val in valve_value_pairs)
        return self.send(cmd)

    def valve_off(self, valve):
        """Turn off a specific valve"""
        with self.display_lock:
            self.valve_data.pop(valve, None)
        return self.send(f"{valve},off")

    def set_onoff(self, pairs):
        """Set one or more on/off solenoids.

        pairs: list of (internal index 0..5, bool). Sent as a single
        'd2,1,d5,0' line (pin-numbered on the wire) so the valves switch in one
        pass of the firmware's command parser rather than staggered across
        several serial round-trips.
        """
        for idx, _ in pairs:
            if not 0 <= idx < NUM_ONOFF:
                self.message_queue.append(
                    f"[ERR] On/off index {idx} out of range (0..{NUM_ONOFF - 1})"
                )
                return False
        cmd = ",".join(f"d{idx + ONOFF_PIN0},{1 if on else 0}" for idx, on in pairs)
        return self.send(cmd)

    def onoff_all_off(self):
        """Release every on/off solenoid (leaves the regulators alone)."""
        with self.display_lock:
            self.onoff_data = {i: False for i in range(NUM_ONOFF)}
        return self.send("d,off")

    def query_onoff(self):
        """Query status of all on/off solenoids"""
        return self.send("d?")

    def emergency_stop(self):
        """Emergency stop - all valves off, regulators and solenoids alike"""
        with self.display_lock:
            self.valve_data.clear()
            self.onoff_data = {i: False for i in range(NUM_ONOFF)}
        return self.send("s")

    def query_status(self):
        """Query status of all valves"""
        return self.send("?")

    def ping(self):
        """Ping test"""
        return self.send("p")

    def close(self):
        """Close connection"""
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=1)
        if self.ser:
            self.ser.close()
            print("\n[OK] Connection closed")


class LiveDisplay:
    """Matplotlib display: 6 regulators per row, plus an on/off solenoid strip"""

    def __init__(self, controller):
        self.controller = controller
        self.fig = None
        self.ani = None
        self.axes = []          # one axis per regulator row, top to bottom
        self.onoff_ax = None    # the on/off solenoid strip along the bottom
        self.status_text = None

        self.colors = {
            'bg': '#1a1a2e',
            'panel': '#16213e',
            'accent': '#0f3460',
            'valve_bar': '#4ecdc4',
            'text': '#eaeaea',
            'text_dim': '#888888',
            'active': '#00ff88',
            'onoff_on': '#00ff88',
            'onoff_off': '#242438'
        }

    def setup(self):
        """Create the matplotlib figure and axes"""
        plt.style.use('dark_background')

        self.fig = plt.figure(figsize=(13, 3.4 + 2.2 * NUM_ROWS),
                              facecolor=self.colors['bg'])
        self.fig.canvas.manager.set_window_title('6-Valve Controller')

        # One row per group of regulators, then a short strip for the on/off
        # solenoids -- they carry one bit each, so they need far less height.
        gs = self.fig.add_gridspec(NUM_ROWS + 1, 1, hspace=0.75,
                                   height_ratios=[3] * NUM_ROWS + [1],
                                   left=0.07, right=0.97, top=0.92, bottom=0.06)

        self.fig.suptitle('6-Valve Controller (Compact)', fontsize=18,
                          color=self.colors['text'], fontweight='bold',
                          fontfamily='monospace')

        self.axes = []
        for r in range(NUM_ROWS):
            v_lo = r * ROW_SIZE
            v_hi = min(v_lo + ROW_SIZE - 1, NUM_VALVES - 1)
            ax = self.fig.add_subplot(gs[r, 0])
            self._style_axes(ax, v_lo, v_hi, row_title(v_lo, v_hi))
            self.axes.append(ax)

        self.onoff_ax = self.fig.add_subplot(gs[NUM_ROWS, 0])
        self._style_onoff_axes(self.onoff_ax)

        self.status_text = self.fig.text(
            0.98, 0.97, '● IDLE', fontsize=10,
            color=self.colors['text_dim'], fontfamily='monospace',
            ha='right', va='top'
        )

    def _style_axes(self, ax, v_lo, v_hi, title):
        ax.set_facecolor(self.colors['panel'])
        ax.set_title(title, color=self.colors['text'], fontsize=12,
                     fontfamily='monospace', loc='left')
        ax.set_xlim(v_lo - 0.5, v_hi + 0.5)
        ax.set_ylim(0, 10500)
        ax.set_xticks(range(v_lo, v_hi + 1))
        ax.set_xlabel('Valve', color=self.colors['text_dim'], fontfamily='monospace')
        ax.set_ylabel('mV', color=self.colors['text_dim'], fontfamily='monospace')
        ax.tick_params(colors=self.colors['text_dim'])
        ax.spines['bottom'].set_color(self.colors['accent'])
        ax.spines['left'].set_color(self.colors['accent'])
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.grid(True, axis='y', alpha=0.2, color=self.colors['text_dim'])

    def _style_onoff_axes(self, ax):
        ax.set_facecolor(self.colors['panel'])
        ax.set_title(
            f'On/Off solenoids  d{ONOFF_PIN0}-d{ONOFF_PIN0 + NUM_ONOFF - 1}  '
            f'(= Giga pins D{ONOFF_PIN0}-D{ONOFF_PIN0 + NUM_ONOFF - 1})',
            color=self.colors['text'], fontsize=12,
            fontfamily='monospace', loc='left')
        ax.set_xlim(-0.5, NUM_ONOFF - 0.5)
        ax.set_ylim(0, 1)
        ax.set_xticks(range(NUM_ONOFF))
        # The number is the pin, so one label says it all.
        ax.set_xticklabels([f'd{ONOFF_PIN0 + i}' for i in range(NUM_ONOFF)])
        ax.set_yticks([])
        ax.tick_params(colors=self.colors['text_dim'])
        for side in ('top', 'right', 'left'):
            ax.spines[side].set_visible(False)
        ax.spines['bottom'].set_color(self.colors['accent'])

    def _redraw_onoff(self, states):
        ax = self.onoff_ax
        ax.clear()
        self._style_onoff_axes(ax)

        for i in range(NUM_ONOFF):
            on = states.get(i, False)
            ax.bar(i, 1.0, 0.7,
                   color=self.colors['onoff_on'] if on else self.colors['onoff_off'],
                   alpha=0.9 if on else 1.0)
            ax.text(i, 0.5, 'ON' if on else 'OFF', ha='center', va='center',
                    fontsize=10, fontweight='bold', fontfamily='monospace',
                    color=self.colors['bg'] if on else self.colors['text_dim'])

    @staticmethod
    def _mv_to_bar(mV):
        """Convert commanded mV to output pressure in bar.

        Command side maps 0-10 V to 0..900 kPa, i.e. kPa = mV * 0.09.
        Then bar = kPa / 100.
        """
        kpa = KPA_MIN + mV * ((KPA_MAX - KPA_MIN) / CMD_MV_FS)
        return kpa / 100.0

    def _redraw_row(self, ax, v_lo, v_hi, title, snapshot):
        ax.clear()
        self._style_axes(ax, v_lo, v_hi, title)

        any_active = False
        for v in range(v_lo, v_hi + 1):
            if v in snapshot:
                mV = snapshot[v]
                ax.bar(v, mV, 0.6, color=self.colors['valve_bar'], alpha=0.85)
                bar = self._mv_to_bar(mV)
                ax.text(v, mV + 200, f'{bar:.2f} bar', ha='center', fontsize=9,
                        color=self.colors['text'])
                any_active = True

        if not any_active:
            ax.text((v_lo + v_hi) / 2.0, 5000, 'No active valves',
                    ha='center', va='center', fontsize=13,
                    color=self.colors['text_dim'], fontfamily='monospace')

    def update(self, frame):
        """Update all display elements"""
        ctrl = self.controller
        with ctrl.display_lock:
            snapshot = dict(ctrl.valve_data)
            onoff = dict(ctrl.onoff_data)

        for r, ax in enumerate(self.axes):
            v_lo = r * ROW_SIZE
            v_hi = min(v_lo + ROW_SIZE - 1, NUM_VALVES - 1)
            self._redraw_row(ax, v_lo, v_hi, row_title(v_lo, v_hi), snapshot)

        self._redraw_onoff(onoff)

        n_on = sum(1 for v in onoff.values() if v)
        if snapshot or n_on:
            self.status_text.set_text(f'● ACTIVE ({len(snapshot)} reg, {n_on} on/off)')
            self.status_text.set_color(self.colors['active'])
        else:
            self.status_text.set_text('● IDLE')
            self.status_text.set_color(self.colors['text_dim'])

        return [self.status_text]

    def start(self):
        """Start the animation"""
        self.setup()
        self.ani = FuncAnimation(self.fig, self.update, interval=50,
                                 blit=False, cache_frame_data=False)
        plt.show(block=False)

    def close(self):
        """Close the display"""
        if self.ani:
            self.ani.event_source.stop()
        plt.close(self.fig)


def print_help():
    print("""
+-------------------------------------------------------------------+
|               6-VALVE CONTROLLER - COMMANDS                       |
+-------------------------------------------------------------------+
|  PROPORTIONAL REGULATORS  (indices 0..5; all on Wire):            |
|    valve,value         Set single valve (e.g. 0,900 or 3,450)     |
|    v1,val1,v2,val2,..  Set multiple    (e.g. 0,900,3,450)         |
|    valve,off           Turn off valve  (e.g. 4,off)               |
|    ?                   Query status of all regulators             |
+-------------------------------------------------------------------+
|  ON/OFF SOLENOIDS  (d2..d7 == Giga pins D2..D7):                  |
|    dN,V                Set one    (d2,1  d2,on  d2,0  d2,off)     |
|    dN,V,dM,V,..        Set several            (e.g. d2,1,d5,0)    |
|    d,off               Release every on/off valve                 |
|    d  or  d?           Query status of all on/off valves          |
|                                                                   |
|    NOTE: the number IS the Giga pin: d2 drives D2, d7 drives D7.  |
+-------------------------------------------------------------------+
|  s                     EMERGENCY STOP (regulators AND solenoids)  |
+-------------------------------------------------------------------+
|  INFO:                                                            |
|    p                   Ping test                                  |
|    h                   Show this help                             |
|    q                   Quit program                               |
+-------------------------------------------------------------------+
|  Mode set in Arduino: PRESSURE (kPa) or VOLTAGE (mV)              |
|  Python limit: values above 2200 are rejected (= 198 kPa)         |
+-------------------------------------------------------------------+
""")


def input_loop(controller):
    """Handle command input in terminal"""
    print("\nType 'h' for help. Commands are entered in this terminal.\n")

    while controller.running:
        try:
            while controller.message_queue:
                msg = controller.message_queue.popleft()
                print(f"  {msg}")

            prompt = "[VALVE]"
            if controller.valve_data:
                prompt += f" [{len(controller.valve_data)} reg]"
            n_on = sum(1 for v in controller.onoff_data.values() if v)
            if n_on:
                prompt += f" [{n_on} on/off]"
            prompt += "> "

            cmd = input(prompt).strip()

            if not cmd:
                continue

            cmd_lower = cmd.lower()

            # ==================== QUIT/STOP ====================

            if cmd_lower == 'q':
                print("Shutting down...")
                controller.emergency_stop()
                time.sleep(0.3)
                controller.running = False
                break

            if cmd_lower == 's':
                controller.emergency_stop()
                print("  ** EMERGENCY STOP **")
                continue

            # ==================== HELP/INFO ====================

            if cmd_lower == 'h':
                print_help()
                continue

            if cmd_lower == '?':
                controller.query_status()
                time.sleep(0.2)
                continue

            if cmd_lower == 'p':
                controller.ping()
                continue

            # ==================== ON/OFF SOLENOIDS ====================
            # Checked before the regulator parser: 'd2,1' must never reach
            # int('d2'). The typed number is the Giga pin (d2..d7); it is
            # shifted to the internal 0..5 index before set_onoff().

            if cmd_lower in ('d', 'd?'):
                controller.query_onoff()
                time.sleep(0.2)
                continue

            if cmd_lower == 'd,off':
                controller.onoff_all_off()
                print("  All on/off valves -> OFF")
                continue

            if cmd_lower.startswith('d') and ',' in cmd_lower:
                parts = [t.strip() for t in cmd_lower.split(',')]
                if len(parts) % 2 != 0:
                    print("[ERR] Invalid format. Use: dN,V or dN,V,dM,V,...")
                    continue
                try:
                    pairs = []
                    for i in range(0, len(parts), 2):
                        tok = parts[i]
                        if tok.startswith('d'):
                            tok = tok[1:]
                        idx = int(tok) - ONOFF_PIN0
                        val = parts[i + 1]
                        if val in ('1', 'on'):
                            on = True
                        elif val in ('0', 'off'):
                            on = False
                        else:
                            raise ValueError(f"bad value {val!r}")
                        pairs.append((idx, on))
                except (ValueError, IndexError) as e:
                    print(f"[ERR] {e}. Use: dN,V with V = 0/1/on/off, e.g. d2,1")
                    continue
                if controller.set_onoff(pairs):
                    print("  " + ", ".join(
                        f"D{i + ONOFF_PIN0}->{'ON' if on else 'OFF'}"
                        for i, on in pairs))
                continue

            # ==================== VALVE CONTROL ====================

            if ',' in cmd:
                parts = cmd.split(',')

                # Single valve: valve,value or valve,off
                if len(parts) == 2:
                    try:
                        valve = int(parts[0])
                        if parts[1].lower() == 'off':
                            controller.valve_off(valve)
                            print(f"  V{valve} -> OFF")
                        else:
                            value = int(parts[1])
                            if controller.set_valve(valve, value):
                                print(f"  V{valve} -> {value}")
                    except ValueError:
                        print("[ERR] Invalid format. Use: valve,value or valve,off")
                    continue

                # Multiple valves: v1,val1,v2,val2,...
                if len(parts) >= 2 and len(parts) % 2 == 0:
                    try:
                        pairs = []
                        for i in range(0, len(parts), 2):
                            v = int(parts[i])
                            val = int(parts[i + 1])
                            pairs.append((v, val))
                        if controller.set_multiple_valves(pairs):
                            print(f"  Setting: {', '.join(f'V{v}={val}' for v, val in pairs)}")
                    except (ValueError, IndexError):
                        print("[ERR] Invalid format. Use: v1,val1,v2,val2,...")
                    continue

                print("[ERR] Invalid format. Use: valve,value or v1,val1,v2,val2,...")
                continue

            print(f"[?] Unknown: {cmd}. Type 'h' for help.")

        except EOFError:
            break
        except KeyboardInterrupt:
            print("\n\n[STOP] Emergency stop triggered!")
            controller.emergency_stop()
            time.sleep(0.3)
            controller.running = False
            break


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else None

    print("=" * 65)
    print("      6-VALVE CONTROLLER (Compact, Feedforward)")
    print("=" * 65)

    controller = ValveController(port)

    if not controller.ser:
        print("[ERR] Failed to connect. Exiting.")
        return

    display = LiveDisplay(controller)
    display.start()

    input_thread = threading.Thread(target=input_loop, args=(controller,), daemon=True)
    input_thread.start()

    try:
        while controller.running:
            plt.pause(0.05)

            while controller.message_queue:
                msg = controller.message_queue.popleft()
                print(f"  {msg}")

    except KeyboardInterrupt:
        print("\n[STOP] Shutting down...")
        controller.emergency_stop()
    finally:
        controller.running = False
        display.close()
        controller.close()


if __name__ == "__main__":
    main()
