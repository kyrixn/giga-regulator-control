#!/usr/bin/env python3
"""
vc2_control.py

16-Valve Controller - Serial Communication Script with Matplotlib Display

Controls Arduino vc2 sketch for direct voltage/pressure control over 16 DACs:
  - Valves  0..7  on Wire   (SDA/SCL)
  - Valves  8..15 on Wire1  (SDA1/SCL1)

Arduino mode (set in Arduino code):
  PRESSURE mode: input in kPa (-100 to 500)
  VOLTAGE mode:  input in mV (0 to 10000)

Commands:
  Valve Control:
    valve,value          - Set single valve: 0,3000 or 9,2100
    v1,val1,v2,val2,...  - Set multiple valves: 0,3000,9,2500
    valve,off            - Turn off a valve: 9,off
    s                    - Emergency stop (all valves off)
    ?                    - Query status of all valves

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


NUM_VALVES = 16
ROW_SIZE = 8
MAX_INPUT_VALUE = 4000  # mV or kPa — never send values above this


class ValveController:
    """16-Valve serial interface with matplotlib display"""

    def __init__(self, port=None, baudrate=115200):
        self.ser = None
        self.baudrate = baudrate
        self.running = False
        self.read_thread = None

        # Valve readings (commanded mV)
        self.valve_data = {}  # {valve_id: mV}

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

        if line.startswith("EMERGENCY"):
            with self.display_lock:
                self.valve_data.clear()
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
        if (line.startswith("Mode:")
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

    def emergency_stop(self):
        """Emergency stop - all valves off"""
        with self.display_lock:
            self.valve_data.clear()
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
    """Matplotlib display: 16 valves in 2 rows (V0-V7 top, V8-V15 bottom)"""

    def __init__(self, controller):
        self.controller = controller
        self.fig = None
        self.ani = None
        self.ax_top = None
        self.ax_bot = None
        self.status_text = None

        self.colors = {
            'bg': '#1a1a2e',
            'panel': '#16213e',
            'accent': '#0f3460',
            'valve_bar': '#4ecdc4',
            'text': '#eaeaea',
            'text_dim': '#888888',
            'active': '#00ff88'
        }

    def setup(self):
        """Create the matplotlib figure and axes"""
        plt.style.use('dark_background')

        self.fig = plt.figure(figsize=(13, 7), facecolor=self.colors['bg'])
        self.fig.canvas.manager.set_window_title('16-Valve Controller')

        gs = self.fig.add_gridspec(2, 1, hspace=0.35,
                                   left=0.07, right=0.97, top=0.90, bottom=0.08)

        self.fig.suptitle('16-Valve Controller (Dual-Bus)', fontsize=18,
                          color=self.colors['text'], fontweight='bold',
                          fontfamily='monospace')

        self.ax_top = self.fig.add_subplot(gs[0, 0])
        self._style_axes(self.ax_top, 0, 7, 'Valves 0-7  (Wire)')

        self.ax_bot = self.fig.add_subplot(gs[1, 0])
        self._style_axes(self.ax_bot, 8, 15, 'Valves 8-15 (Wire1)')

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

    @staticmethod
    def _mv_to_bar(mV):
        """Convert commanded mV to output pressure in bar.

        Device maps 0-10 V to -100..500 kPa, i.e. kPa = (mV/1000)*60 - 100.
        Then bar = kPa / 100. Clamped to >= 0 (no vacuum hardware).
        """
        bar = ((mV / 1000.0) * 60.0 - 100.0) / 100.0
        return max(0.0, bar)

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

        self._redraw_row(self.ax_top, 0, 7, 'Valves 0-7  (Wire)', snapshot)
        self._redraw_row(self.ax_bot, 8, 15, 'Valves 8-15 (Wire1)', snapshot)

        if snapshot:
            self.status_text.set_text(f'● ACTIVE ({len(snapshot)})')
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
|              16-VALVE CONTROLLER - COMMANDS                       |
+-------------------------------------------------------------------+
|  VALVE CONTROL  (indices 0..15;  0-7 = Wire,  8-15 = Wire1):      |
|    valve,value         Set single valve (e.g. 0,3000 or 9,2100)   |
|    v1,val1,v2,val2,..  Set multiple    (e.g. 0,3000,8,2500)       |
|    valve,off           Turn off valve  (e.g. 9,off)               |
|    s                   EMERGENCY STOP (all valves off)            |
|    ?                   Query status of all valves                 |
+-------------------------------------------------------------------+
|  INFO:                                                            |
|    p                   Ping test                                  |
|    h                   Show this help                             |
|    q                   Quit program                               |
+-------------------------------------------------------------------+
|  Mode set in Arduino: PRESSURE (kPa) or VOLTAGE (mV)              |
|  Python limit: values above 4000 are rejected                     |
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
                prompt += f" [{len(controller.valve_data)} active]"
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
    print("      16-VALVE CONTROLLER (Dual-Bus, Feedforward)")
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
