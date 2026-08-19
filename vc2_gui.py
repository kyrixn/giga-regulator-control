#!/usr/bin/env python3
"""
vc2_gui.py

Interactive GUI for the vc2 6-valve controller. Reuses the serial layer
from vc2_control.py (ValveController) and replaces the terminal input
loop with on-figure matplotlib widgets:

  - One TextBox under each bar (V0..V5):
        * Type a number + Enter to set that valve immediately
        * Type "off" (or o/x) + Enter to turn that valve off
        * Clicking away does NOT submit — the typed text stays in the box
          until you press Enter or click APPLY ALL
  - APPLY ALL button: fire every non-empty box in one batched command
  - STOP button: emergency stop (sends 's' to Arduino)
  - ? STATUS button: query all valve states
  - PING button: ping the Arduino

Layout: one row of 6 valves, all on Wire (DAC 0x58-0x5A).

Values above MAX_INPUT_VALUE (2200 mV = 198 kPa) are rejected; see
vc2_control.py. The Arduino may allow higher; Python never sends more.

Usage:
    python vc2_gui.py [port]
    python vc2_gui.py /dev/ttyACM0
"""

import sys
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import TextBox, Button

from vc2_control import (
    ValveController, NUM_VALVES, NUM_ROWS, ROW_SIZE, row_title,
    MAX_INPUT_VALUE,
)


# ============================================================
# Layout (figure-fraction coordinates)
# ============================================================
PLOT_LEFT   = 0.06
PLOT_RIGHT  = 0.85
PLOT_WIDTH  = PLOT_RIGHT - PLOT_LEFT
SLOT_W      = PLOT_WIDTH / ROW_SIZE     # one slot per valve in a row

TB_W        = 0.075                     # textbox width

# Vertical band per row. Bands stack top-to-bottom between these margins.
TOP_MARGIN    = 0.94
BOTTOM_MARGIN = 0.055
BAND_H        = (TOP_MARGIN - BOTTOM_MARGIN) / NUM_ROWS

BTN_X       = 0.87
BTN_W       = 0.11

COLORS = {
    'bg':        '#1a1a2e',
    'panel':     '#16213e',
    'accent':    '#0f3460',
    'accent_h':  '#1a4480',
    'valve_bar': '#4ecdc4',
    'text':      '#eaeaea',
    'text_dim':  '#888888',
    'active':    '#00ff88',
    'stop':      '#cc2a2a',
    'stop_h':    '#ff3333',
    'apply':     '#1f8a3c',
    'apply_h':   '#28a745',
}


def _row_bounds(r):
    """Return (v_lo, v_hi) valve indices for display row r (0 = top)."""
    v_lo = r * ROW_SIZE
    v_hi = min(v_lo + ROW_SIZE - 1, NUM_VALVES - 1)
    return v_lo, v_hi


class InteractiveDisplay:
    """Matplotlib figure with bar plots + per-valve input boxes + STOP button."""

    def __init__(self, controller):
        self.controller = controller
        self.fig = None
        self.ani = None
        self.axes = []             # one bar-plot axis per row, top to bottom
        self.textboxes = []        # list of TextBox, indexed by valve id (0..5)
        self.status_text = None
        self.msg_text = None
        self.recent_msgs = []

        self.btn_stop = None
        self.btn_apply = None
        self.btn_qry = None
        self.btn_ping = None

        # Tracks which textbox (if any) was focused when Enter was pressed.
        # Set by our pre-textbox key_press_event handler. Consumed in submit().
        # If None when submit() fires, it means submit was triggered by
        # focus-loss (click-away), not by Enter -> ignore.
        self._enter_box = None

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    def setup(self):
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(15, 2.4 + 2.1 * NUM_ROWS),
                              facecolor=COLORS['bg'])
        try:
            self.fig.canvas.manager.set_window_title('6-Valve Controller (Interactive)')
        except Exception:
            pass

        # IMPORTANT: connect our key_press_event handler BEFORE creating any
        # TextBox. matplotlib dispatches callbacks in connection order, so
        # this guarantees our handler runs before each TextBox's own _keypress
        # handler -- which is how we know an Enter occurred at the moment
        # submit() fires (vs. click-away, which we want to silently ignore).
        self.fig.canvas.mpl_connect('key_press_event', self._on_key_press)

        self.fig.suptitle(
            '6-Valve Controller  (Interactive)',
            fontsize=17, color=COLORS['text'], fontweight='bold',
            fontfamily='monospace', y=0.985
        )

        # One bar plot + one textbox row per band, stacked top to bottom.
        self.axes = []
        for r in range(NUM_ROWS):
            v_lo, v_hi = _row_bounds(r)
            band_top = TOP_MARGIN - r * BAND_H
            band_bottom = band_top - BAND_H

            plot_h = BAND_H * 0.50
            plot_y = band_bottom + BAND_H * 0.34
            ax = self.fig.add_axes([PLOT_LEFT, plot_y, PLOT_WIDTH, plot_h])
            self._style_axes(ax, v_lo, v_hi, row_title(v_lo, v_hi))
            self.axes.append(ax)

            tb_h = min(0.032, BAND_H * 0.16)
            tb_y = band_bottom + BAND_H * 0.06
            self._make_textbox_row(v_lo, v_hi, tb_y, tb_h)

        self._make_buttons()

        self.status_text = self.fig.text(
            0.98, 0.985, '● IDLE', fontsize=11,
            color=COLORS['text_dim'], fontfamily='monospace',
            ha='right', va='top'
        )

        self.fig.text(
            PLOT_LEFT, 0.012,
            "Enter fires that valve (single).   "
            "Click-away keeps text but doesn't fire.   "
            "APPLY ALL fires every non-empty box in one batch.   "
            f"Max {MAX_INPUT_VALUE}.",
            fontsize=9, color=COLORS['text_dim'], fontfamily='monospace'
        )

        self.msg_text = self.fig.text(
            BTN_X, 0.16, '',
            fontsize=8, color=COLORS['text_dim'], fontfamily='monospace',
            va='top'
        )

    def _make_textbox_row(self, v_lo, v_hi, y, tb_h):
        """Place TextBox widgets, one per valve in the row, aligned to bars."""
        for i in range(v_lo, v_hi + 1):
            cx = PLOT_LEFT + ((i - v_lo) + 0.5) * SLOT_W
            ax_tb = self.fig.add_axes([cx - TB_W / 2, y, TB_W, tb_h])
            tb = TextBox(
                ax_tb, '', initial='',
                color=COLORS['panel'], hovercolor=COLORS['accent']
            )
            tb.text_disp.set_color(COLORS['text'])
            tb.text_disp.set_fontfamily('monospace')
            tb.text_disp.set_fontsize(11)

            # Track which valve this is, and a re-entry guard for set_val('').
            tb._valve_id = i
            tb._suppress = False
            tb.on_submit(self._make_submit(i, tb))
            self.textboxes.append(tb)

    def _make_buttons(self):
        # STOP (big red, top of column)
        ax_stop = self.fig.add_axes([BTN_X, 0.62, BTN_W, 0.30])
        self.btn_stop = Button(
            ax_stop, 'STOP',
            color=COLORS['stop'], hovercolor=COLORS['stop_h']
        )
        self.btn_stop.label.set_fontsize(22)
        self.btn_stop.label.set_fontweight('bold')
        self.btn_stop.label.set_color('white')
        self.btn_stop.on_clicked(self._on_stop)

        # APPLY ALL (medium green, mid column)
        ax_apply = self.fig.add_axes([BTN_X, 0.42, BTN_W, 0.16])
        self.btn_apply = Button(
            ax_apply, 'APPLY ALL',
            color=COLORS['apply'], hovercolor=COLORS['apply_h']
        )
        self.btn_apply.label.set_fontsize(13)
        self.btn_apply.label.set_fontweight('bold')
        self.btn_apply.label.set_color('white')
        self.btn_apply.on_clicked(self._on_apply_all)

        # ? STATUS
        ax_qry = self.fig.add_axes([BTN_X, 0.33, BTN_W, 0.05])
        self.btn_qry = Button(
            ax_qry, '? STATUS',
            color=COLORS['accent'], hovercolor=COLORS['accent_h']
        )
        self.btn_qry.label.set_color('white')
        self.btn_qry.label.set_fontsize(10)
        self.btn_qry.on_clicked(lambda _: self.controller.query_status())

        # PING
        ax_ping = self.fig.add_axes([BTN_X, 0.26, BTN_W, 0.05])
        self.btn_ping = Button(
            ax_ping, 'PING',
            color=COLORS['accent'], hovercolor=COLORS['accent_h']
        )
        self.btn_ping.label.set_color('white')
        self.btn_ping.label.set_fontsize(10)
        self.btn_ping.on_clicked(lambda _: self.controller.ping())

    # ------------------------------------------------------------------
    # Widget callbacks
    # ------------------------------------------------------------------

    def _make_submit(self, valve, tb):
        """Create an on_submit callback bound to (valve, tb).

        Submit fires from matplotlib on BOTH Enter and focus-loss. We only
        act if our pre-textbox key_press_event handler tagged this textbox
        as the one that received Enter. Otherwise this is a click-away and
        we leave the text in the box for APPLY ALL.
        """
        def submit(text):
            if tb._suppress:
                return

            # Only fire if Enter was pressed in THIS textbox.
            if self._enter_box is not tb:
                return
            self._enter_box = None

            t = text.strip().lower()
            if t == '':
                return

            if t in ('off', 'o', 'x'):
                self.controller.valve_off(valve)
            else:
                try:
                    value = int(t)
                    if not self.controller.set_valve(valve, value):
                        return
                except ValueError:
                    self.controller.message_queue.append(
                        f"[ERR] V{valve}: invalid value '{text}'"
                    )
                    return

            tb._suppress = True
            try:
                tb.set_val('')
            finally:
                tb._suppress = False
        return submit

    def _on_key_press(self, event):
        """Tag the focused textbox (if any) when Enter is pressed.

        Runs before each TextBox's own _keypress handler (because we
        connected first in setup()). The submit() callback that fires
        moments later checks self._enter_box to distinguish Enter from
        focus-loss.
        """
        if event.key not in ('enter', 'return'):
            return
        for tb in self.textboxes:
            if tb.capturekeystrokes:
                self._enter_box = tb
                return
        self._enter_box = None

    def _on_apply_all(self, _event):
        """Send every non-empty textbox as one batched command."""
        pairs = []          # list of (valve_id, value-or-'off')
        boxes_to_clear = []
        invalid = []

        for tb in self.textboxes:
            valve = tb._valve_id
            raw = tb.text.strip()
            if raw == '':
                continue

            t = raw.lower()
            if t in ('off', 'o', 'x'):
                pairs.append((valve, 'off'))
                boxes_to_clear.append(tb)
                continue

            try:
                value = int(t)
            except ValueError:
                invalid.append((valve, raw))
                continue

            if value > MAX_INPUT_VALUE:
                invalid.append((valve, raw))
                continue

            pairs.append((valve, value))
            boxes_to_clear.append(tb)

        if invalid:
            for v, txt in invalid:
                self.controller.message_queue.append(
                    f"[ERR] V{v}: invalid value '{txt}' (APPLY ALL aborted)"
                )
            return

        if not pairs:
            self.controller.message_queue.append(
                "[INFO] APPLY ALL: no values to apply"
            )
            return

        if not self.controller.set_multiple_valves(pairs):
            return

        self.controller.message_queue.append(
            f"APPLY ALL: {len(pairs)} valves"
        )

        for tb in boxes_to_clear:
            tb._suppress = True
            try:
                tb.set_val('')
            finally:
                tb._suppress = False

    def _on_stop(self, _event):
        self.controller.emergency_stop()
        self.controller.message_queue.append('** EMERGENCY STOP (button) **')

    # ------------------------------------------------------------------
    # Drawing
    # ------------------------------------------------------------------

    def _style_axes(self, ax, v_lo, v_hi, title):
        ax.set_facecolor(COLORS['panel'])
        ax.set_title(title, color=COLORS['text'], fontsize=11,
                     fontfamily='monospace', loc='left')
        ax.set_xlim(v_lo - 0.5, v_hi + 0.5)
        ax.set_ylim(0, 10500)
        ax.set_xticks(range(v_lo, v_hi + 1))
        ax.set_ylabel('mV', color=COLORS['text_dim'], fontfamily='monospace')
        ax.tick_params(colors=COLORS['text_dim'])
        ax.spines['bottom'].set_color(COLORS['accent'])
        ax.spines['left'].set_color(COLORS['accent'])
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.grid(True, axis='y', alpha=0.2, color=COLORS['text_dim'])

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
                ax.bar(v, mV, 0.6, color=COLORS['valve_bar'], alpha=0.85)
                bar = self._mv_to_bar(mV)
                ax.text(v, mV + 200, f'{bar:.2f} bar', ha='center', fontsize=8,
                        color=COLORS['text'])
                any_active = True

        if not any_active:
            ax.text((v_lo + v_hi) / 2.0, 5000, 'No active valves',
                    ha='center', va='center', fontsize=12,
                    color=COLORS['text_dim'], fontfamily='monospace')

    def update(self, _frame):
        ctrl = self.controller
        with ctrl.display_lock:
            snapshot = dict(ctrl.valve_data)

        for r, ax in enumerate(self.axes):
            v_lo, v_hi = _row_bounds(r)
            self._redraw_row(ax, v_lo, v_hi, row_title(v_lo, v_hi), snapshot)

        if snapshot:
            self.status_text.set_text(f'● ACTIVE ({len(snapshot)})')
            self.status_text.set_color(COLORS['active'])
        else:
            self.status_text.set_text('● IDLE')
            self.status_text.set_color(COLORS['text_dim'])

        # Drain message queue and show the last few lines on the figure.
        while ctrl.message_queue:
            self.recent_msgs.append(ctrl.message_queue.popleft())
        if len(self.recent_msgs) > 6:
            self.recent_msgs = self.recent_msgs[-6:]
        self.msg_text.set_text('\n'.join(self.recent_msgs))

        return []

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self):
        self.setup()
        self.ani = FuncAnimation(
            self.fig, self.update, interval=80,
            blit=False, cache_frame_data=False
        )
        plt.show()  # blocks until the window is closed

    def close(self):
        if self.ani is not None:
            try:
                self.ani.event_source.stop()
            except Exception:
                pass
        if self.fig is not None:
            plt.close(self.fig)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else None

    print("=" * 65)
    print("   6-VALVE CONTROLLER (Interactive GUI)")
    print("=" * 65)

    controller = ValveController(port)
    if not controller.ser:
        print("[ERR] Failed to connect. Exiting.")
        return

    display = InteractiveDisplay(controller)
    try:
        display.start()  # blocks
    except KeyboardInterrupt:
        print("\n[STOP] Shutting down...")
    finally:
        try:
            controller.emergency_stop()
            time.sleep(0.2)
        except Exception:
            pass
        display.close()
        controller.close()


if __name__ == '__main__':
    main()
