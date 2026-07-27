/**
 * ui_display.cpp
 *
 * Implementation of the on-Giga touchscreen UI. See ui_display.h for the
 * design contract (non-blocking, read-only, serial-priority).
 *
 * Libraries (install via Library Manager):
 *   - Arduino_GigaDisplay_GFX      (framebuffer + Adafruit-GFX drawing API)
 *   - Arduino_GigaDisplayTouch     (GT911 capacitive touch, on Wire1)
 */
#include <Arduino.h>
#include <stdio.h>

#include "ui_display.h"
#include "valve_core.h"

#include "Arduino_GigaDisplay_GFX.h"
#include "Arduino_GigaDisplayTouch.h"

// ============================================================
// Devices
// ============================================================
static GigaDisplay_GFX         gfx;
static Arduino_GigaDisplayTouch touch;

// ============================================================
// Geometry (landscape 800x480)
// ============================================================
static const int SCR_W = 800;
static const int SCR_H = 480;

static const int PAD       = 6;
static const int TOPBAR_H  = 80;
static const int BAR_Y     = PAD;
static const int BAR_H     = TOPBAR_H - 2 * PAD;      // 68

static const int ESTOP_W   = 152;
static const int ESTOP_X   = SCR_W - ESTOP_W - PAD;   // 642
static const int STATUS_W  = 70;
static const int STATUS_X  = ESTOP_X - STATUS_W - PAD;// 566
static const int TABS_X    = PAD;                     // 6
static const int TABS_W    = STATUS_X - PAD - TABS_X; // width available for tabs

static const int NCOL      = 4;
static const int NROW      = 2;
static const int TILE_TOP  = TOPBAR_H + PAD;          // 86
static const int TILES_H   = SCR_H - TILE_TOP - PAD;  // 388

static const int NUM_GROUPS = VC_NUM_VALVES / VC_GROUP_SIZE;  // 4

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
static const uint16_t C_ACTIVE   = RGB565(  0, 255, 136);
static const uint16_t C_TEXT     = RGB565(234, 234, 234);
static const uint16_t C_TEXTDIM  = RGB565(136, 136, 136);
static const uint16_t C_STOP     = RGB565(204,  42,  42);
static const uint16_t C_STOP_H   = RGB565(255,  51,  51);
static const uint16_t C_WHITE    = 0xFFFF;

// ============================================================
// Touch mapping (portrait native -> landscape display)
// ------------------------------------------------------------
// The GT911 reports coordinates in the panel's native 480x800 portrait frame,
// independent of gfx.setRotation(). Map them into our 800x480 landscape frame.
// If taps land on the wrong tile, flip these three flags until they line up.
// ============================================================
#define TOUCH_SWAP_XY 1
#define TOUCH_FLIP_X  0
#define TOUCH_FLIP_Y  1

// Set to 1 to print raw+mapped touch coords to Serial for calibration.
// WARNING: this adds lines to the same serial link the PC app parses -- leave
// it 0 during normal operation.
#define TOUCH_DEBUG 0

// ============================================================
// UI state
// ============================================================
static int      curGroup = 0;
static bool     cacheValid = false;
static float    cacheKpa[VC_GROUP_SIZE];
static bool     cacheOn[VC_GROUP_SIZE];
static int      cacheActive = -1;

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

static void tileRect(int k, int &x, int &y, int &w, int &h) {
  int col = k % NCOL;
  int row = k / NCOL;
  w = (SCR_W - 2 * PAD - (NCOL - 1) * PAD) / NCOL;   // 192
  h = (TILES_H - (NROW - 1) * PAD) / NROW;           // 191
  x = PAD + col * (w + PAD);
  y = TILE_TOP + row * (h + PAD);
}

static void tabRect(int i, int &x, int &y, int &w, int &h) {
  w = (TABS_W - (NUM_GROUPS - 1) * PAD) / NUM_GROUPS;
  h = BAR_H;
  x = TABS_X + i * (w + PAD);
  y = BAR_Y;
}

static bool inRect(int px, int py, int x, int y, int w, int h) {
  return px >= x && px < x + w && py >= y && py < y + h;
}

// ============================================================
// Drawing
// ============================================================
static void drawTab(int i) {
  int x, y, w, h;
  tabRect(i, x, y, w, h);
  bool active = (i == curGroup);
  gfx.fillRect(x, y, w, h, active ? C_TABACT : C_ACCENT);
  gfx.drawRect(x, y, w, h, C_ACCENT_H);

  int lo = i * VC_GROUP_SIZE;
  int hi = lo + VC_GROUP_SIZE - 1;
  gfx.setTextColor(active ? C_WHITE : C_TEXTDIM);
  gfx.setTextSize(2);
  gfx.setCursor(x + 8, y + 10);
  gfx.print("V"); gfx.print(lo); gfx.print("-"); gfx.print(hi);

  gfx.setTextSize(1);
  gfx.setCursor(x + 8, y + 44);
  gfx.print(lo < VC_BANK_SPLIT ? "Wire" : "Wire2");
}

static void drawEstop(bool pressed) {
  gfx.fillRect(ESTOP_X, BAR_Y, ESTOP_W, BAR_H, pressed ? C_STOP_H : C_STOP);
  gfx.setTextColor(C_WHITE);
  gfx.setTextSize(3);
  gfx.setCursor(ESTOP_X + 16, BAR_Y + BAR_H / 2 - 11);
  gfx.print("E-STOP");
}

static void drawStatus(int active) {
  gfx.fillRect(STATUS_X, BAR_Y, STATUS_W, BAR_H, C_PANEL);
  gfx.drawRect(STATUS_X, BAR_Y, STATUS_W, BAR_H, C_ACCENT);
  gfx.setTextColor(C_TEXTDIM);
  gfx.setTextSize(1);
  gfx.setCursor(STATUS_X + 8, BAR_Y + 8);
  gfx.print("ACTIVE");
  gfx.setTextColor(active > 0 ? C_ACTIVE : C_TEXTDIM);
  gfx.setTextSize(3);
  gfx.setCursor(STATUS_X + 8, BAR_Y + 28);
  gfx.print(active);
}

static void drawTile(int k) {
  int x, y, w, h;
  tileRect(k, x, y, w, h);
  int v = curGroup * VC_GROUP_SIZE + k;
  bool on = vcValveOn(v);
  float kpa = vcValueKpa(v);

  uint16_t border = on ? C_ACTIVE : C_ACCENT;
  gfx.fillRect(x, y, w, h, C_PANEL);
  gfx.drawRect(x, y, w, h, border);
  gfx.drawRect(x + 1, y + 1, w - 2, h - 2, border);

  // Valve id
  gfx.setTextColor(C_TEXT);
  gfx.setTextSize(2);
  gfx.setCursor(x + 10, y + 10);
  gfx.print("V"); gfx.print(v);

  // Pressure value
  if (on) {
    char buf[12];
    snprintf(buf, sizeof(buf), "%d", roundKpa(kpa));
    gfx.setTextColor(C_TEXT);
    gfx.setTextSize(4);
    gfx.setCursor(x + 12, y + 60);
    gfx.print(buf);
    gfx.setTextSize(2);
    gfx.setCursor(x + 12, y + 104);
    gfx.print("kPa");
  } else {
    gfx.setTextColor(C_TEXTDIM);
    gfx.setTextSize(4);
    gfx.setCursor(x + 12, y + 70);
    gfx.print("OFF");
  }

  // Fill bar (fraction of the -100..500 kPa span)
  int barX = x + 10, barW = w - 20;
  int barY = y + h - 26, barH = 16;
  gfx.drawRect(barX, barY, barW, barH, C_ACCENT_H);
  float frac = on ? (kpa - VC_KPA_MIN) / (VC_KPA_MAX - VC_KPA_MIN) : 0.0f;
  if (frac < 0) frac = 0;
  if (frac > 1) frac = 1;
  int fillW = (int)(frac * (barW - 2));
  if (fillW > 0) gfx.fillRect(barX + 1, barY + 1, fillW, barH - 2, C_BAR);
}

// Full repaint (initial screen or after switching windows).
static void drawAll() {
  gfx.fillScreen(C_BG);
  for (int i = 0; i < NUM_GROUPS; i++) drawTab(i);
  drawEstop(false);
  cacheActive = vcActiveCount();
  drawStatus(cacheActive);

  for (int k = 0; k < VC_GROUP_SIZE; k++) {
    drawTile(k);
    int v = curGroup * VC_GROUP_SIZE + k;
    cacheOn[k]  = vcValveOn(v);
    cacheKpa[k] = vcValueKpa(v);
  }
  cacheValid = true;
}

// Redraw only tiles whose value/state changed (throttled by tick()).
static void renderDirty() {
  for (int k = 0; k < VC_GROUP_SIZE; k++) {
    int v = curGroup * VC_GROUP_SIZE + k;
    bool on = vcValveOn(v);
    float kpa = vcValueKpa(v);
    float d = kpa - cacheKpa[k];
    if (d < 0) d = -d;
    if (!cacheValid || on != cacheOn[k] || d >= 1.0f) {
      drawTile(k);
      cacheOn[k]  = on;
      cacheKpa[k] = kpa;
    }
  }
  cacheValid = true;

  int active = vcActiveCount();
  if (active != cacheActive) {
    drawStatus(active);
    cacheActive = active;
  }
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

static void handleTap(int lx, int ly) {
  // E-STOP first (safety).
  if (inRect(lx, ly, ESTOP_X, BAR_Y, ESTOP_W, BAR_H)) {
    vcEmergencyStop();
    drawEstop(true);
    estopFlashUntil = millis() + 160;
    return;
  }
  // Window tabs.
  for (int i = 0; i < NUM_GROUPS; i++) {
    int x, y, w, h;
    tabRect(i, x, y, w, h);
    if (inRect(lx, ly, x, y, w, h)) {
      if (i != curGroup) {
        curGroup = i;
        cacheValid = false;
        drawAll();
      }
      return;
    }
  }
}

static void pollTouch() {
  GDTpoint_t pts[5];
  uint8_t contacts = touch.getTouchPoints(pts);
  bool touched = contacts > 0;

  if (touched && !wasTouched) {      // rising edge only (debounced tap)
    int lx, ly;
    mapTouch(pts[0].x, pts[0].y, lx, ly);
#if TOUCH_DEBUG
    Serial.print("[touch] raw="); Serial.print(pts[0].x); Serial.print(",");
    Serial.print(pts[0].y); Serial.print(" -> "); Serial.print(lx);
    Serial.print(","); Serial.println(ly);
#endif
    handleTap(lx, ly);
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
  curGroup = 0;
  drawAll();
}

void tick() {
  uint32_t now = millis();

  // Touch poll ~50 Hz (cheap I2C read on Wire1).
  if (now - lastTouchMs >= 20) {
    lastTouchMs = now;
    pollTouch();
  }

  // Clear the E-STOP press flash.
  if (estopFlashUntil && now >= estopFlashUntil) {
    estopFlashUntil = 0;
    drawEstop(false);
  }

  // Redraw dirty tiles ~15 Hz.
  if (now - lastDrawMs >= 66) {
    lastDrawMs = now;
    renderDirty();
  }
}

}  // namespace ui
