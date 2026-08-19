/**
 * ui_display.cpp
 *
 * On-Giga touchscreen UI. Two modes, toggled by the MODE button in the top bar:
 *
 *   MONITOR    - read-only. Mirrors PC/serial state as pressure tiles (kPa).
 *   STANDALONE - touchscreen sliders drive the regulators directly; incoming
 *                serial commands are ignored (see vcSetSerialIgnore). Sliders
 *                map their travel onto VC_SB_MV_MIN..VC_SB_MV_MAX.
 *
 * Design contract (see ui_display.h): ui::tick() is cooperative and
 * non-blocking. In MONITOR it never touches serial or the DAC buses; in
 * STANDALONE serial is ignored, so driving the DACs from touch is safe.
 *
 * Compact build: all 6 regulators live on one page, so the top bar carries only
 * MODE and E-STOP. The space to their left is intentionally blank -- reserved
 * for controls to be added later. The tile / slider grids keep their original
 * 4x2 and 2x4 geometry; the two trailing cells simply stay empty.
 *
 * Libraries: Arduino_GigaDisplay_GFX, Arduino_GigaDisplayTouch (GT911 on Wire1).
 */
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include "ui_display.h"
#include "valve_core.h"

#include "Arduino_GigaDisplay_GFX.h"
#include "Arduino_GigaDisplayTouch.h"

// ============================================================
// Devices
// ============================================================
static GigaDisplay_GFX          gfx;
static Arduino_GigaDisplayTouch  touch;

// ============================================================
// Geometry (landscape 800x480)
// ============================================================
static const int SCR_W = 800;
static const int SCR_H = 480;

static const int PAD      = 6;
static const int TOPBAR_H = 80;
static const int BAR_Y    = PAD;
static const int BAR_H    = TOPBAR_H - 2 * PAD;       // 68

static const int ESTOP_W  = 152;
static const int ESTOP_X  = SCR_W - ESTOP_W - PAD;    // 642
static const int MODE_W   = 70;
static const int MODE_X   = ESTOP_X - MODE_W - PAD;   // 566  (MODE button)
// PAD .. MODE_X is left blank -- room for controls added later.

static const int NCOL     = 4;                        // monitor tile columns
static const int NROW     = 2;                        // monitor tile rows
static const int TILE_TOP = TOPBAR_H + PAD;           // 86
static const int TILES_H  = SCR_H - TILE_TOP - PAD;   // 388

// Standalone slider grid: 2 columns x 4 rows (last 2 cells unused with 6 valves).
static const int SB_COLW  = (SCR_W - 3 * PAD) / 2;    // 391
static const int SB_LX    = PAD;                      // left column x
static const int SB_RX    = PAD + SB_COLW + PAD;      // right column x
static const int SB_ROWH  = (TILES_H - 3 * PAD) / 4;  // 92

// ============================================================
// Colors (RGB565)
// ============================================================
#define RGB565(r, g, b) \
  ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))

static const uint16_t C_BG       = RGB565( 26,  26,  46);
static const uint16_t C_PANEL    = RGB565( 22,  33,  62);
static const uint16_t C_ACCENT   = RGB565( 15,  52,  96);
static const uint16_t C_ACCENT_H = RGB565( 26,  68, 128);
static const uint16_t C_BAR      = RGB565( 78, 205, 196);
static const uint16_t C_BAR_DIM  = RGB565( 40,  74,  72);
static const uint16_t C_ACTIVE   = RGB565(  0, 255, 136);
static const uint16_t C_TEXT     = RGB565(234, 234, 234);
static const uint16_t C_TEXTDIM  = RGB565(136, 136, 136);
static const uint16_t C_STOP     = RGB565(204,  42,  42);
static const uint16_t C_STOP_H   = RGB565(255,  51,  51);
static const uint16_t C_STAND    = RGB565(230, 150,  20);   // standalone accent
static const uint16_t C_WHITE    = 0xFFFF;

// ============================================================
// Touch mapping (portrait native -> landscape display)
// ------------------------------------------------------------
// The GT911 reports coordinates in the panel's native 480x800 portrait frame,
// independent of gfx.setRotation(). If taps land on the wrong spot, flip these.
// ============================================================
#define TOUCH_SWAP_XY 1
#define TOUCH_FLIP_X  0
#define TOUCH_FLIP_Y  1

// Print raw+mapped touch coords to Serial for calibration (adds serial lines;
// leave 0 in normal use).
#define TOUCH_DEBUG 0

// ============================================================
// UI state
// ============================================================
enum UiMode { MODE_MONITOR = 0, MODE_STANDALONE = 1 };
static UiMode mode = MODE_MONITOR;

static bool     cacheValid = false;
static float    cacheKpa[VC_GROUP_SIZE];    // monitor commanded cache
static bool     cacheOn[VC_GROUP_SIZE];
static float    cacheMeasKpa[VC_GROUP_SIZE]; // monitor measured cache (real-time)
static int      cacheSbMv[VC_GROUP_SIZE];   // standalone slider cache (mV)

static int      grabbed = -1;              // slider being dragged, -1 if none
static bool     wasTouched = false;
static uint32_t lastTouchMs = 0;
static uint32_t lastDrawMs  = 0;
static uint32_t estopFlashUntil = 0;

// ============================================================
// Small helpers
// ============================================================
static int clampi(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static int roundKpa(float k) {
  return (int)(k >= 0 ? k + 0.5f : k - 0.5f);
}

static bool inRect(int px, int py, int x, int y, int w, int h) {
  return px >= x && px < x + w && py >= y && py < y + h;
}

// --- monitor tile geometry ---
static void tileRect(int k, int &x, int &y, int &w, int &h) {
  int col = k % NCOL, row = k / NCOL;
  w = (SCR_W - 2 * PAD - (NCOL - 1) * PAD) / NCOL;   // 192
  h = (TILES_H - (NROW - 1) * PAD) / NROW;           // 191
  x = PAD + col * (w + PAD);
  y = TILE_TOP + row * (h + PAD);
}

// --- standalone slider geometry ---
static void sbCell(int k, int &x, int &y, int &w, int &h) {
  int col = k / 4;          // 0 = left (V0-V3), 1 = right (V4, V5)
  int row = k % 4;
  w = SB_COLW;
  h = SB_ROWH;
  x = col ? SB_RX : SB_LX;
  y = TILE_TOP + row * (h + PAD);
}

static void sbTrack(int x, int y, int w, int h,
                    int &tx, int &ty, int &tw, int &th) {
  tx = x + 12;
  tw = w - 24;
  ty = y + 44;
  th = 30;
}

// ============================================================
// Drawing — top bar
// ============================================================
static void drawModeButton() {
  bool sa = (mode == MODE_STANDALONE);
  gfx.fillRect(MODE_X, BAR_Y, MODE_W, BAR_H, sa ? C_STAND : C_ACCENT);
  gfx.drawRect(MODE_X, BAR_Y, MODE_W, BAR_H, C_ACCENT_H);
  gfx.setTextColor(C_WHITE);
  gfx.setTextSize(1);
  gfx.setCursor(MODE_X + 8, BAR_Y + 8);
  gfx.print("MODE");
  gfx.setTextSize(2);
  gfx.setCursor(MODE_X + 8, BAR_Y + 28);
  gfx.print(sa ? "CTRL" : "MON");
}

static void drawEstop(bool pressed) {
  gfx.fillRect(ESTOP_X, BAR_Y, ESTOP_W, BAR_H, pressed ? C_STOP_H : C_STOP);
  gfx.setTextColor(C_WHITE);
  gfx.setTextSize(3);
  gfx.setCursor(ESTOP_X + 16, BAR_Y + BAR_H / 2 - 11);
  gfx.print("E-STOP");
}

static void drawTopBar() {
  // Left of MODE stays empty on purpose (future controls).
  drawModeButton();
  drawEstop(false);
}

// ============================================================
// Drawing — monitor tiles
// ============================================================
// Big real-time measured number. Redrawn on its own (partial) so sensor jitter
// never forces a whole-tile repaint. "--" when this valve has no ADC channel.
static void drawTileMeasured(int k) {
  int x, y, w, h;
  tileRect(k, x, y, w, h);

  gfx.fillRect(x + 8, y + 52, w - 16, 52, C_PANEL);   // clear just this band
  gfx.setTextSize(6);
  gfx.setCursor(x + 14, y + 56);
  if (vcHasMeasure(k)) {
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", roundKpa(vcMeasuredKpa(k)));
    gfx.setTextColor(C_TEXT);
    gfx.print(buf);
  } else {
    gfx.setTextColor(C_TEXTDIM);
    gfx.print("--");
  }
}

// Bottom bar: tracks measured pressure when a sensor is present, else commanded.
static void drawTileBar(int k) {
  int x, y, w, h;
  tileRect(k, x, y, w, h);
  bool meas = vcHasMeasure(k);
  float kpa = meas ? vcMeasuredKpa(k)
                   : (vcValveOn(k) ? vcValueKpa(k) : VC_KPA_MIN);

  int barX = x + 10, barW = w - 20, barY = y + h - 26, barH = 16;
  gfx.fillRect(barX, barY, barW, barH, C_PANEL);      // clear old fill
  gfx.drawRect(barX, barY, barW, barH, C_ACCENT_H);
  float frac = (kpa - VC_KPA_MIN) / (VC_KPA_MAX - VC_KPA_MIN);
  if (frac < 0) frac = 0;
  if (frac > 1) frac = 1;
  int fillW = (int)(frac * (barW - 2));
  if (fillW > 0)
    gfx.fillRect(barX + 1, barY + 1, fillW, barH - 2, meas ? C_BAR : C_BAR_DIM);
}

// Static tile layout: panel, border, valve id, small commanded setpoint, unit.
// The dynamic parts (measured number + bar) are delegated to the helpers above.
static void drawTile(int k) {
  int x, y, w, h;
  tileRect(k, x, y, w, h);
  bool on = vcValveOn(k);

  uint16_t border = on ? C_ACTIVE : C_ACCENT;
  gfx.fillRect(x, y, w, h, C_PANEL);
  gfx.drawRect(x, y, w, h, border);
  gfx.drawRect(x + 1, y + 1, w - 2, h - 2, border);

  // Valve id (top-left).
  gfx.setTextColor(C_TEXT);
  gfx.setTextSize(2);
  gfx.setCursor(x + 10, y + 10);
  gfx.print("V"); gfx.print(k);

  // Commanded setpoint (top-right, small — same size as the id text).
  char cbuf[16];
  if (on) snprintf(cbuf, sizeof(cbuf), "SET %d", roundKpa(vcValueKpa(k)));
  else    snprintf(cbuf, sizeof(cbuf), "SET OFF");
  int cw = (int)strlen(cbuf) * 12;         // size-2 glyph is 12 px wide
  gfx.setTextColor(on ? C_TEXT : C_TEXTDIM);
  gfx.setCursor(x + w - 10 - cw, y + 10);
  gfx.print(cbuf);

  // Unit label under the big measured value (static).
  gfx.setTextColor(C_TEXTDIM);
  gfx.setTextSize(2);
  gfx.setCursor(x + 14, y + 114);
  gfx.print("kPa");

  drawTileMeasured(k);
  drawTileBar(k);
}

static void drawMonitorAll() {
  for (int k = 0; k < VC_GROUP_SIZE; k++) {
    drawTile(k);
    cacheOn[k]      = vcValveOn(k);
    cacheKpa[k]     = vcValueKpa(k);
    cacheMeasKpa[k] = vcMeasuredKpa(k);
  }
}

// ============================================================
// Drawing — standalone sliders
// ============================================================
static void drawSlider(int k) {
  int x, y, w, h;
  sbCell(k, x, y, w, h);
  int outMv = vcValueMv(k);
  bool inBand = (outMv >= VC_SB_MV_MIN && outMv <= VC_SB_MV_MAX);

  gfx.fillRect(x, y, w, h, C_PANEL);
  gfx.drawRect(x, y, w, h, inBand ? C_STAND : C_ACCENT);

  // Label
  gfx.setTextColor(C_TEXT);
  gfx.setTextSize(2);
  gfx.setCursor(x + 10, y + 12);
  gfx.print("V"); gfx.print(k);

  // Readout (kPa) — dim if the valve is off or outside the control band.
  gfx.setTextSize(2);
  gfx.setCursor(x + w - 128, y + 12);
  if (outMv <= 0) {
    gfx.setTextColor(C_TEXTDIM);
    gfx.print("OFF");
  } else {
    char buf[16];
    snprintf(buf, sizeof(buf), "%d kPa", roundKpa(outMv * 0.06f - 100.0f));
    gfx.setTextColor(inBand ? C_TEXT : C_TEXTDIM);
    gfx.print(buf);
  }

  // Track + fill + handle
  int tx, ty, tw, th;
  sbTrack(x, y, w, h, tx, ty, tw, th);
  gfx.drawRect(tx, ty, tw, th, C_ACCENT_H);
  float frac = inBand
      ? (float)(outMv - VC_SB_MV_MIN) / (float)(VC_SB_MV_MAX - VC_SB_MV_MIN)
      : 0.0f;
  int fillW = (int)(frac * (tw - 4));
  if (fillW > 0)
    gfx.fillRect(tx + 2, ty + 2, fillW, th - 4, inBand ? C_BAR : C_BAR_DIM);
  int hx = tx + 2 + fillW;
  gfx.fillRect(hx - 3, ty - 5, 6, th + 10, inBand ? C_WHITE : C_TEXTDIM);
}

static void drawStandaloneAll() {
  // Amber frame around the main area makes the mode unmistakable.
  gfx.drawRect(2, TILE_TOP - 2, SCR_W - 4, TILES_H + 4, C_STAND);
  for (int k = 0; k < VC_GROUP_SIZE; k++) {
    drawSlider(k);
    cacheSbMv[k] = vcValueMv(k);
  }
}

// ============================================================
// Full repaint + dirty redraw
// ============================================================
static void drawAll() {
  gfx.fillScreen(C_BG);
  drawTopBar();
  if (mode == MODE_STANDALONE) drawStandaloneAll();
  else                         drawMonitorAll();
  cacheValid = true;
}

static void renderDirty() {
  if (mode == MODE_STANDALONE) {
    for (int k = 0; k < VC_GROUP_SIZE; k++) {
      if (k == grabbed) continue;              // the drag redraws it live
      int m = vcValueMv(k);
      if (!cacheValid || m != cacheSbMv[k]) {  // e.g. after E-STOP
        drawSlider(k);
        cacheSbMv[k] = m;
      }
    }
  } else {
    for (int k = 0; k < VC_GROUP_SIZE; k++) {
      bool on = vcValveOn(k);
      float kpa = vcValueKpa(k);
      float meas = vcMeasuredKpa(k);
      float dc = kpa - cacheKpa[k];       if (dc < 0) dc = -dc;
      float dm = meas - cacheMeasKpa[k];  if (dm < 0) dm = -dm;
      if (!cacheValid || on != cacheOn[k] || dc >= 1.0f) {
        drawTile(k);                      // full repaint: commanded/state changed
        cacheOn[k]      = on;
        cacheKpa[k]     = kpa;
        cacheMeasKpa[k] = meas;
      } else if (dm >= 1.0f) {
        drawTileMeasured(k);              // partial: just the live number + bar
        drawTileBar(k);
        cacheMeasKpa[k] = meas;
      }
    }
  }
  cacheValid = true;
}

// ============================================================
// Mode switching
// ============================================================
static void toggleMode() {
  mode = (mode == MODE_MONITOR) ? MODE_STANDALONE : MODE_MONITOR;
  vcSetSerialIgnore(mode == MODE_STANDALONE);   // ignore serial while standalone
  grabbed = -1;
  cacheValid = false;
  drawAll();     // entering standalone does NOT drive any output; it only draws
}

// ============================================================
// Touch
// ============================================================
static void mapTouch(int tx, int ty, int &lx, int &ly) {
  int ax = tx, ay = ty;
#if TOUCH_SWAP_XY
  int t = ax; ax = ay; ay = t;
#endif
#if TOUCH_FLIP_X
  ax = (SCR_W - 1) - ax;
#endif
#if TOUCH_FLIP_Y
  ay = (SCR_H - 1) - ay;
#endif
  lx = clampi(ax, 0, SCR_W - 1);
  ly = clampi(ay, 0, SCR_H - 1);
}

// Which slider (if any) contains the point. -1 if none.
static int sliderAt(int lx, int ly) {
  for (int k = 0; k < VC_GROUP_SIZE; k++) {
    int x, y, w, h;
    sbCell(k, x, y, w, h);
    if (inRect(lx, ly, x, y, w, h)) return k;
  }
  return -1;
}

// Set slider k's valve from the finger's x, redraw it live.
static void applyDrag(int k, int lx) {
  int x, y, w, h;
  sbCell(k, x, y, w, h);
  int tx, ty, tw, th;
  sbTrack(x, y, w, h, tx, ty, tw, th);
  float frac = (float)(lx - tx) / (float)tw;
  if (frac < 0) frac = 0;
  if (frac > 1) frac = 1;
  int mV = VC_SB_MV_MIN + (int)(frac * (VC_SB_MV_MAX - VC_SB_MV_MIN) + 0.5f);
  if (mV != cacheSbMv[k]) {
    vcSetValveMv(k, mV);
    cacheSbMv[k] = mV;
    drawSlider(k);
  }
}

// Top-bar taps (E-STOP / MODE). Returns true if handled.
static bool handleTopBarTap(int lx, int ly) {
  if (inRect(lx, ly, ESTOP_X, BAR_Y, ESTOP_W, BAR_H)) {
    vcEmergencyStop();
    drawEstop(true);
    estopFlashUntil = millis() + 160;
    return true;
  }
  if (inRect(lx, ly, MODE_X, BAR_Y, MODE_W, BAR_H)) {
    toggleMode();
    return true;
  }
  return false;   // the rest of the bar is blank
}

static void pollTouch() {
  GDTpoint_t pts[5];
  uint8_t contacts = touch.getTouchPoints(pts);
  bool touched = contacts > 0;

  if (touched) {
    int lx, ly;
    mapTouch(pts[0].x, pts[0].y, lx, ly);
#if TOUCH_DEBUG
    Serial.print("[touch] raw="); Serial.print(pts[0].x); Serial.print(",");
    Serial.print(pts[0].y); Serial.print(" -> "); Serial.print(lx);
    Serial.print(","); Serial.println(ly);
#endif
    if (!wasTouched) {                       // touch down
      if (ly < TOPBAR_H) {
        handleTopBarTap(lx, ly);
      } else if (mode == MODE_STANDALONE) {
        grabbed = sliderAt(lx, ly);
        if (grabbed >= 0) applyDrag(grabbed, lx);
      }
    } else {                                 // touch held (drag)
      if (mode == MODE_STANDALONE && grabbed >= 0) applyDrag(grabbed, lx);
    }
  } else {
    grabbed = -1;                            // released
  }
  wasTouched = touched;
}

// ============================================================
// Public API
// ============================================================
namespace ui {

void begin() {
  gfx.begin();
  gfx.setRotation(1);          // landscape 800x480
  touch.begin();               // GT911 on Wire1
  mode = MODE_MONITOR;
  grabbed = -1;
  drawAll();
}

void tick() {
  uint32_t now = millis();

  if (now - lastTouchMs >= 20) {             // touch poll ~50 Hz
    lastTouchMs = now;
    pollTouch();
  }

  if (estopFlashUntil && now >= estopFlashUntil) {
    estopFlashUntil = 0;
    drawEstop(false);
  }

  if (now - lastDrawMs >= 66) {              // dirty redraw ~15 Hz
    lastDrawMs = now;
    renderDirty();
  }
}

}  // namespace ui
