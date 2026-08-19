/**
 * ui_display.cpp
 *
 * On-Giga touchscreen UI for the compact 6-valve controller.
 *
 * ONE mode: direct control. The old MONITOR/STANDALONE split is gone -- every
 * page is live, and what used to be the MODE button is now LOCK.
 *
 * Layout: 2 pages of 3 regulators, chosen from the tabs in the top bar. Each
 * regulator owns one row, split down the middle:
 *   left  -- valve id, commanded setpoint, live measured pressure, span bar
 *   right -- the slider that commands it
 *
 * LOCK gates TOUCH ONLY. Locked (the power-on state) the sliders ignore the
 * finger, so a brush against the panel cannot pressurise anything; unlocked
 * they drive the DACs directly. Serial is never gated -- a PC command always
 * lands, and the slider follows it on the next redraw because the slider
 * renders vcValueMv(), the real setpoint, rather than a position of its own.
 *
 * Design contract (see ui_display.h): ui::tick() is cooperative and
 * non-blocking -- it returns immediately unless a millis()-gated timer is due,
 * and even then does only a few small draws.
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
static const int LOCK_W   = 110;
static const int LOCK_X   = ESTOP_X - LOCK_W - PAD;   // 526
static const int TABS_X   = PAD;                      // 6
static const int TABS_W   = LOCK_X - PAD - TABS_X;    // 514

static const int NUM_PAGES = VC_NUM_VALVES / VC_GROUP_SIZE;   // 2

// Body: VC_GROUP_SIZE rows, each split into equal left/right halves.
static const int BODY_TOP = TOPBAR_H + PAD;                            // 86
static const int BODY_H   = SCR_H - BODY_TOP - PAD;                    // 388
static const int ROW_W    = SCR_W - 2 * PAD;                           // 788
static const int ROW_H    = (BODY_H - (VC_GROUP_SIZE - 1) * PAD)
                            / VC_GROUP_SIZE;                           // 125
static const int HALF_W   = (ROW_W - PAD) / 2;                         // 391
static const int LEFT_X   = PAD;                                       // 6
static const int RIGHT_X  = PAD + HALF_W + PAD;                        // 403

// ============================================================
// Colors (RGB565)
// ============================================================
#define RGB565(r, g, b) \
  ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))

static const uint16_t C_BG       = RGB565( 26,  26,  46);
static const uint16_t C_PANEL    = RGB565( 22,  33,  62);
static const uint16_t C_ACCENT   = RGB565( 15,  52,  96);
static const uint16_t C_ACCENT_H = RGB565( 26,  68, 128);
static const uint16_t C_TABACT   = RGB565( 30,  80, 140);
static const uint16_t C_BAR      = RGB565( 78, 205, 196);
static const uint16_t C_BAR_DIM  = RGB565( 40,  74,  72);
static const uint16_t C_TEXT     = RGB565(234, 234, 234);
static const uint16_t C_TEXTDIM  = RGB565(136, 136, 136);
static const uint16_t C_STOP     = RGB565(204,  42,  42);
static const uint16_t C_STOP_H   = RGB565(255,  51,  51);
static const uint16_t C_LIVE     = RGB565(230, 150,  20);   // unlocked accent
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

// Redraw hysteresis (kPa). The monitor slope is 225 kPa/V, so the same
// electrical noise swings a lot of kPa; without this the numbers flicker.
// Still far finer than the regulator's +/-9 kPa (VC_KPA_ACCURACY) linearity.
static const float VC_KPA_REDRAW = 2.0f;

// LOCK re-arm delay (ms). handleTopBarTap() fires on the touch-down edge, so a
// momentary GT911 dropout mid-press reads as a second press and toggles LOCK
// straight back. Ignoring repeat presses for this long swallows the bounce.
// Only LOCK needs it: E-STOP and the page tabs are idempotent, a toggle is not.
static const uint32_t LOCK_REARM_MS = 500;

// ============================================================
// UI state
// ============================================================
// Locked at power-on: the panel must never come up able to drive pressure.
static bool     locked  = true;
static int      curPage = 0;

static bool     cacheValid = false;
static int      cacheMv[VC_GROUP_SIZE];       // commanded setpoint (mV)
static float    cacheMeasKpa[VC_GROUP_SIZE];  // measured pressure (kPa)

static int      grabbed = -1;              // slider being dragged, -1 if none
static bool     wasTouched = false;
static uint32_t lastTouchMs = 0;
static uint32_t lastDrawMs  = 0;
static uint32_t estopFlashUntil = 0;
static uint32_t lastLockMs  = 0;           // last accepted LOCK press

// ============================================================
// Small helpers
// ============================================================
static int clampi(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static int roundKpa(float k) {
  return (int)(k >= 0 ? k + 0.5f : k - 0.5f);
}

static bool inRect(int px, int py, int x, int y, int w, int h) {
  return px >= x && px < x + w && py >= y && py < y + h;
}

// Built-in GFX font advances 6*size px per character.
static int textW(const char *s, int size) {
  return (int)strlen(s) * 6 * size;
}

// Which valve row k of the current page shows.
static int valveOf(int k) {
  return curPage * VC_GROUP_SIZE + k;
}

static int rowY(int k) {
  return BODY_TOP + k * (ROW_H + PAD);
}

static void tabRect(int i, int &x, int &y, int &w, int &h) {
  w = (TABS_W - (NUM_PAGES - 1) * PAD) / NUM_PAGES;
  h = BAR_H;
  x = TABS_X + i * (w + PAD);
  y = BAR_Y;
}

// The slider's active track inside the right half of row k.
static void sliderTrack(int k, int &tx, int &ty, int &tw, int &th) {
  int y = rowY(k);
  tx = RIGHT_X + 16;
  tw = HALF_W - 32;
  ty = y + 46;
  th = 36;
}

// ============================================================
// Drawing - top bar
// ============================================================
static void drawTab(int i) {
  int x, y, w, h;
  tabRect(i, x, y, w, h);
  bool active = (i == curPage);
  gfx.fillRect(x, y, w, h, active ? C_TABACT : C_ACCENT);
  gfx.drawRect(x, y, w, h, C_ACCENT_H);

  int lo = i * VC_GROUP_SIZE, hi = lo + VC_GROUP_SIZE - 1;
  gfx.setTextColor(active ? C_WHITE : C_TEXTDIM);
  gfx.setTextSize(3);
  gfx.setCursor(x + 12, y + 22);
  gfx.print("V"); gfx.print(lo); gfx.print("-"); gfx.print(hi);
}

// LOCK gates the sliders' response to touch. Amber = live.
static void drawLockButton() {
  gfx.fillRect(LOCK_X, BAR_Y, LOCK_W, BAR_H, locked ? C_ACCENT : C_LIVE);
  gfx.drawRect(LOCK_X, BAR_Y, LOCK_W, BAR_H, C_ACCENT_H);
  gfx.setTextColor(C_WHITE);
  gfx.setTextSize(1);
  gfx.setCursor(LOCK_X + 10, BAR_Y + 10);
  gfx.print("TOUCH");
  gfx.setTextSize(2);
  gfx.setCursor(LOCK_X + 10, BAR_Y + 32);
  gfx.print(locked ? "LOCKED" : "LIVE");
}

static void drawEstop(bool pressed) {
  gfx.fillRect(ESTOP_X, BAR_Y, ESTOP_W, BAR_H, pressed ? C_STOP_H : C_STOP);
  gfx.setTextColor(C_WHITE);
  gfx.setTextSize(3);
  gfx.setCursor(ESTOP_X + 16, BAR_Y + BAR_H / 2 - 11);
  gfx.print("E-STOP");
}

static void drawTopBar() {
  for (int i = 0; i < NUM_PAGES; i++) drawTab(i);
  drawLockButton();
  drawEstop(false);
}

// ============================================================
// Drawing - left half: id, setpoint, measured pressure
// ============================================================
// Valve id + commanded setpoint. Redrawn on its own whenever the setpoint
// moves, whether that came from the slider or from a serial command.
static void drawRowHeader(int k) {
  int y = rowY(k);
  int v = valveOf(k);
  int mv = vcValueMv(v);

  gfx.fillRect(LEFT_X + 2, y + 2, HALF_W - 4, 32, C_PANEL);

  gfx.setTextColor(C_TEXT);
  gfx.setTextSize(3);
  gfx.setCursor(LEFT_X + 12, y + 8);
  gfx.print("V"); gfx.print(v);

  char buf[20];
  if (mv > 0) snprintf(buf, sizeof(buf), "SET %d kPa", roundKpa(vcMvToKpa(mv)));
  else        snprintf(buf, sizeof(buf), "SET OFF");
  gfx.setTextColor(mv > 0 ? C_TEXT : C_TEXTDIM);
  gfx.setTextSize(2);
  gfx.setCursor(LEFT_X + HALF_W - 12 - textW(buf, 2), y + 12);
  gfx.print(buf);
}

// The big live number. Its own partial redraw so sensor jitter never forces a
// whole-row repaint. "--" when this valve has no ADC channel.
static void drawRowMeasured(int k) {
  int y = rowY(k);
  int v = valveOf(k);

  gfx.fillRect(LEFT_X + 2, y + 38, HALF_W - 4, 50, C_PANEL);

  if (!vcHasMeasure(v)) {
    gfx.setTextSize(6);
    gfx.setTextColor(C_TEXTDIM);
    gfx.setCursor(LEFT_X + 16, y + 40);
    gfx.print("--");
    return;
  }

  char buf[12];
  snprintf(buf, sizeof(buf), "%d", roundKpa(vcMeasuredKpa(v)));
  gfx.setTextSize(6);
  gfx.setTextColor(C_TEXT);
  gfx.setCursor(LEFT_X + 16, y + 40);
  gfx.print(buf);

  gfx.setTextSize(2);
  gfx.setTextColor(C_TEXTDIM);
  gfx.setCursor(LEFT_X + 16 + textW(buf, 6) + 10, y + 72);
  gfx.print("kPa");
}

// Measured pressure as a fraction of the regulator's full span.
static void drawRowBar(int k) {
  int y = rowY(k);
  int v = valveOf(k);
  bool meas = vcHasMeasure(v);
  float kpa = meas ? vcMeasuredKpa(v) : vcValueKpa(v);

  int bx = LEFT_X + 12, bw = HALF_W - 24, by = y + ROW_H - 29, bh = 16;
  gfx.fillRect(bx, by, bw, bh, C_PANEL);
  gfx.drawRect(bx, by, bw, bh, C_ACCENT_H);

  float frac = clampf((kpa - VC_KPA_MIN) / (VC_KPA_MAX - VC_KPA_MIN), 0.0f, 1.0f);
  int fillW = (int)(frac * (bw - 2));
  if (fillW > 0)
    gfx.fillRect(bx + 1, by + 1, fillW, bh - 2, meas ? C_BAR : C_BAR_DIM);
}

static void drawLeftHalf(int k) {
  int y = rowY(k);
  gfx.fillRect(LEFT_X, y, HALF_W, ROW_H, C_PANEL);
  gfx.drawRect(LEFT_X, y, HALF_W, ROW_H, C_ACCENT);
  drawRowHeader(k);
  drawRowMeasured(k);
  drawRowBar(k);
}

// ============================================================
// Drawing - right half: the slider
// ============================================================
static void drawSlider(int k) {
  int y = rowY(k);
  int v = valveOf(k);
  int mv = vcValueMv(v);

  gfx.fillRect(RIGHT_X, y, HALF_W, ROW_H, C_PANEL);
  gfx.drawRect(RIGHT_X, y, HALF_W, ROW_H, locked ? C_ACCENT : C_LIVE);

  gfx.setTextSize(2);
  gfx.setTextColor(locked ? C_TEXTDIM : C_LIVE);
  gfx.setCursor(RIGHT_X + 16, y + 14);
  gfx.print(locked ? "LOCKED" : "DRAG TO SET");

  int tx, ty, tw, th;
  sliderTrack(k, tx, ty, tw, th);
  gfx.drawRect(tx, ty, tw, th, C_ACCENT_H);

  float frac = clampf((float)(mv - VC_SB_MV_MIN)
                      / (float)(VC_SB_MV_MAX - VC_SB_MV_MIN), 0.0f, 1.0f);
  int fillW = (int)(frac * (tw - 4));
  if (fillW > 0)
    gfx.fillRect(tx + 2, ty + 2, fillW, th - 4, locked ? C_BAR_DIM : C_BAR);

  int hx = tx + 2 + fillW;
  gfx.fillRect(hx - 4, ty - 6, 8, th + 12, locked ? C_TEXTDIM : C_WHITE);

  // Band ends, so the travel's meaning is on-screen.
  char buf[16];
  gfx.setTextSize(1);
  gfx.setTextColor(C_TEXTDIM);
  gfx.setCursor(tx, ty + th + 8);
  snprintf(buf, sizeof(buf), "%d", roundKpa(vcMvToKpa(VC_SB_MV_MIN)));
  gfx.print(buf);
  snprintf(buf, sizeof(buf), "%d kPa", roundKpa(vcMvToKpa(VC_SB_MV_MAX)));
  gfx.setCursor(tx + tw - textW(buf, 1), ty + th + 8);
  gfx.print(buf);
}

// ============================================================
// Full repaint + dirty redraw
// ============================================================
static void drawRow(int k) {
  drawLeftHalf(k);
  drawSlider(k);
}

static void drawAll() {
  gfx.fillScreen(C_BG);
  drawTopBar();
  for (int k = 0; k < VC_GROUP_SIZE; k++) {
    drawRow(k);
    int v = valveOf(k);
    cacheMv[k]      = vcValueMv(v);
    cacheMeasKpa[k] = vcMeasuredKpa(v);
  }
  cacheValid = true;
}

static void renderDirty() {
  for (int k = 0; k < VC_GROUP_SIZE; k++) {
    int v = valveOf(k);

    // Setpoint moved. This is the path a serial command takes: it changed the
    // setpoint, so the slider redraws onto the new value -- the PC always wins.
    int mv = vcValueMv(v);
    if (!cacheValid || mv != cacheMv[k]) {
      cacheMv[k] = mv;
      drawRowHeader(k);
      if (k != grabbed) drawSlider(k);   // a live drag already repaints itself
    }

    float meas = vcMeasuredKpa(v);
    float dm = meas - cacheMeasKpa[k];
    if (dm < 0) dm = -dm;
    if (!cacheValid || dm >= VC_KPA_REDRAW) {
      cacheMeasKpa[k] = meas;
      drawRowMeasured(k);
      drawRowBar(k);
    }
  }
  cacheValid = true;
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

// Which row's slider half contains the point. -1 if none.
static int sliderAt(int lx, int ly) {
  for (int k = 0; k < VC_GROUP_SIZE; k++) {
    if (inRect(lx, ly, RIGHT_X, rowY(k), HALF_W, ROW_H)) return k;
  }
  return -1;
}

// Set row k's valve from the finger's x, redraw it live.
static void applyDrag(int k, int lx) {
  int tx, ty, tw, th;
  sliderTrack(k, tx, ty, tw, th);
  float frac = clampf((float)(lx - tx) / (float)tw, 0.0f, 1.0f);
  int mV = VC_SB_MV_MIN + (int)(frac * (VC_SB_MV_MAX - VC_SB_MV_MIN) + 0.5f);

  int v = valveOf(k);
  if (mV != vcValueMv(v)) {
    vcSetValveMv(v, mV);
    cacheMv[k] = mV;
    drawSlider(k);
    drawRowHeader(k);
  }
}

static void setPage(int p) {
  if (p == curPage) return;
  curPage = p;
  grabbed = -1;
  cacheValid = false;
  drawAll();
}

static void toggleLock() {
  locked = !locked;
  grabbed = -1;
  drawLockButton();
  for (int k = 0; k < VC_GROUP_SIZE; k++) drawSlider(k);   // live/dim styling
}

// Top-bar taps (E-STOP / LOCK / page tabs). Returns true if handled.
static bool handleTopBarTap(int lx, int ly) {
  if (inRect(lx, ly, ESTOP_X, BAR_Y, ESTOP_W, BAR_H)) {
    vcEmergencyStop();
    // Re-lock: an E-STOP that leaves the sliders live could be undone by the
    // next stray touch. renderDirty() walks the sliders back to zero.
    if (!locked) { lastLockMs = millis(); toggleLock(); }
    drawEstop(true);
    estopFlashUntil = millis() + 160;
    return true;
  }
  if (inRect(lx, ly, LOCK_X, BAR_Y, LOCK_W, BAR_H)) {
    // Subtraction (not now >= deadline) so this survives millis() rollover.
    uint32_t now = millis();
    if ((uint32_t)(now - lastLockMs) >= LOCK_REARM_MS) {
      lastLockMs = now;
      toggleLock();
    }
    return true;      // swallowed either way; never falls through to a slider
  }
  for (int i = 0; i < NUM_PAGES; i++) {
    int x, y, w, h;
    tabRect(i, x, y, w, h);
    if (inRect(lx, ly, x, y, w, h)) {
      setPage(i);
      return true;
    }
  }
  return false;
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
      } else if (!locked) {                  // LOCK gates the sliders only
        grabbed = sliderAt(lx, ly);
        if (grabbed >= 0) applyDrag(grabbed, lx);
      }
    } else {                                 // touch held (drag)
      if (!locked && grabbed >= 0) applyDrag(grabbed, lx);
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
  locked  = true;
  curPage = 0;
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
