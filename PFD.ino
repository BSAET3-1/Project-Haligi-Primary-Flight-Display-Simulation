/*
 * ESP32 ST7796 Primary Flight Display (PFD)
 * Author: Abrigo, Denmer John E
	   Ante, John Christian A.
	   Apostol, Peter Francis O.
	   Bambico, Vince Allen F. 
	   Caballero, Sherwin M.

 * Board: ESP32-WROOM-32D
 * Display: ST7796 320x480 (SPI)
 * License: Apache License 2.0
 
  TWO MPU6050s:
    - 0x68 supplies PITCH only (roll ignored)
    - 0x69 supplies ROLL only (pitch ignored)
  VSI is mapped from live pitch; Inclinometer is mapped from live roll.
*/

#include <Wire.h>
#include <TFT_eSPI.h>
#include <cmath>

/* ===== I2C PINS/SPEED (adjust if needed) ===== */
#define I2C_SDA 16
#define I2C_SCL 17
#define I2C_HZ  50000   // 50 kHz for stability

/* ====== MPU CONFIG ====== */
#define MPU_ADDR_PITCH 0x68
#define MPU_ADDR_ROLL  0x69
static const float PITCH_SIGN = +1.0f;  // flip to -1.0f if pitch moves opposite
static const float ROLL_SIGN  = +1.0f;  // flip to -1.0f if roll moves opposite

static bool mpuPitchOK=false, mpuRollOK=false;

static bool initMPU(uint8_t addr){
  Wire.beginTransmission(addr);
  if (Wire.endTransmission()!=0) return false;
  // Wake from sleep
  Wire.beginTransmission(addr);
  Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);
  // Optional: set DLPF ~10Hz for cleaner accel
  Wire.beginTransmission(addr);
  Wire.write(0x1A); Wire.write(0x05); Wire.endTransmission(true);
  return true;
}
static bool readAccel(uint8_t addr, int16_t &ax,int16_t &ay,int16_t &az){
  Wire.beginTransmission(addr);
  Wire.write(0x3B); // ACCEL_XOUT_H
  if (Wire.endTransmission(false)!=0) return false;
  Wire.requestFrom((int)addr, 6, (int)true);
  if (Wire.available()!=6) return false;
  ax = (Wire.read()<<8) | Wire.read();
  ay = (Wire.read()<<8) | Wire.read();
  az = (Wire.read()<<8) | Wire.read();
  return true;
}
static float accelPitchDeg(int16_t ax,int16_t ay,int16_t az){
  float fax=ax, fay=ay, faz=az;
  float pitch = atan2f(-fax, sqrtf(fay*fay+faz*faz)) * 57.2957795f; // nose up positive
  return pitch * PITCH_SIGN;
}
static float accelRollDeg(int16_t ax,int16_t ay,int16_t az){
  float fay=ay, faz=az;
  float roll = atan2f(fay, faz) * 57.2957795f;                     // right wing down positive
  return roll * ROLL_SIGN;
}

/* ====== DISPLAY ====== */
TFT_eSPI tft;

// ---------------- Layout (LANDSCAPE) ----------------
static const int16_t SW = 480, SH = 320;
static const int16_t CX = SW/2, CY = SH/2;

static const int16_t TAPE_W = 64;
static const int16_t TOP_H  = 44;

static const int16_t ATT_X  = TAPE_W;
static const int16_t ATT_Y  = TOP_H;
static const int16_t ATT_W  = SW - 2*TAPE_W;
static const int16_t ATT_H  = SH - TOP_H;

static const int16_t REF_Y_OFFSET = TOP_H / 2;

// --------------- Scales -----------------
static const float PITCH_PX_PER_DEG = 5.0f;
static float       ASPD_PX_PER_UNIT = 3.0f;
static const float ALTI_PX_PER_FT   = 0.12f;
static const int   VSI_MAX_FPM      = 2000;

// --------------- Attitude markers ---------------
static const int   HORIZON_THICK    = 3;   // horizon line thickness (pixels)
static const int   ROLL_ARC_R       = 70;  // roll index arc radius (pixels)

// --------------- V-speed bands ---------------
static float VSO = 44;
static float VS1 = 50;
static float VFE = 85;
static float VNO = 129;
static float VNE = 200;

// --------------- Thresholds -----------------
static const float THR_PITCH = 0.30f;
static const float THR_ROLL  = 0.30f;
static const float THR_HDG   = 2.0f;
static const float THR_SPD   = 1.0f;
static const float THR_ALT   = 5.0f;
static const float THR_VSI   = 100.0f;
static const float THR_SLIP  = 0.15f;

// --------------- Rates (ms) -----------------
static const uint16_t PERIOD_ATT  = 33;
static const uint16_t PERIOD_HDG  = 100;
static const uint16_t PERIOD_TAPE = 150;
static const uint16_t PERIOD_VSI  = 200;

// --------------- Colors -----------------
static uint16_t C_BLACK, C_WHITE, C_YELLOW, C_CYAN, C_GREEN, C_SKY, C_GND, C_GREY, C_RED, C_AMBER, C_PURPLE;

// --------------- Data ---------------
static float srcPitch=0, srcRoll=0, srcSpd=100, srcAlt=3500, srcVsi=0;
static float fPitch=0,  fRoll=0,  fSpd=100,  fAlt=3500, fVsi=0;
static float srcSlip=0, fSlip=0; // srcSlip unused now; fSlip follows fRoll

// HEADING STATES
static float fHdg   = 0;
static float bugHdg = 0;
static float viewHdg= 0;

// smoothing
static const float SMOOTH_ATT   = 0.20f;
static const float SMOOTH_TAPES = 0.15f;
static const float SMOOTH_VSI   = 0.25f;
static const float SMOOTH_SLIP  = 0.25f;

// last-drawn
static float prevPitch=1e9, prevRoll=1e9, prevHdgView=1e9, prevSpd=1e9, prevAlt=1e9, prevVsi=1e9, prevSlip=1e9;

// timers
static uint32_t tLastAtt=0, tLastHdg=0, tLastTape=0, tLastVsi=0;

// ---- VSI from pitch scale ----
static const float VSI_PER_DEG = 200.0f;   // ft/min per degree of pitch

// --------- FORWARD DECLARATIONS ----------
void drawVsi(float fpm, bool force=false);
void drawInclinometer(float slipDeg, bool force=false);
void drawAttitude(float pitchDeg, float rollDeg, bool force=false);
void drawHeading(float hdgView, bool force=false);
void drawAirspeed(float spd, bool force=false);
void drawAltitude(float alt, bool force=false);

// utils
inline float   deg2rad(float d){ return d * 0.01745329252f; }
inline int16_t clampi(int16_t v, int16_t lo, int16_t hi){ return v<lo?lo:(v>hi?hi:v); }
inline float   clampf(float v, float lo, float hi){ return v<lo?lo:(v>hi?hi:v); }
inline float   smooth(float cur, float target, float a){ return cur + a*(target - cur); }
inline float   wrapTo360(float a){ while (a >= 360) a -= 360; while (a < 0) a += 360; return a; }
inline float   wrapDiffDeg(float a, float b){ float d = a - b; while (d > 180) d -= 360; while (d < -180) d += 360; return d; }
inline int     norm360i(int x){ x%=360; if(x<0)x+=360; return x; }

// easing + shortest-arc angle lerp
inline float easeInOut01(float x){
  return (x < 0.5f) ? 2.0f*x*x : 1.0f - powf(-2.0f*x + 2.0f, 2.0f)/2.0f;
}
inline float lerpAngleDeg(float a, float b, float t){
  float d = wrapDiffDeg(b, a);
  return wrapTo360(a + d * t);
}

// Airspeed Y mapping (increase upward)
inline int16_t spdToY(float spdCenter, float value){
  return CY - (int16_t)((value - spdCenter) * ASPD_PX_PER_UNIT);
}

// ------------- Cohen–Sutherland clip helpers -------------
enum { INSIDE=0, LEFT=1, RIGHT=2, BOTTOM=4, TOP=8 };
static int outCode(int x, int y, int rx, int ry, int rw, int rh){
  int code = INSIDE;
  if (x < rx)            code |= LEFT;
  else if (x > rx+rw-1)  code |= RIGHT;
  if (y < ry)            code |= TOP;
  else if (y > ry+rh-1)  code |= BOTTOM;
  return code;
}
static bool clipLineToRect(int &x0,int &y0,int &x1,int &y1, int rx,int ry,int rw,int rh){
  int code0 = outCode(x0,y0,rx,ry,rw,rh);
  int code1 = outCode(x1,y1,rx,ry,rw,rh);
  while (true){
    if (!(code0 | code1)) return true;
    else if (code0 & code1) return false;
    int codeOut = code0 ? code0 : code1;
    int x, y;
    if (codeOut & TOP) {
      x = x0 + (x1 - x0) * (ry - y0) / (y1 - y0); y = ry;
    } else if (codeOut & BOTTOM) {
      x = x0 + (x1 - x0) * ((ry+rh-1) - y0) / (y1 - y0); y = ry + rh - 1;
    } else if (codeOut & RIGHT) {
      y = y0 + (y1 - y0) * ((rx+rw-1) - x0) / (x1 - x0); x = rx + rw - 1;
    } else { // LEFT
      y = y0 + (y1 - y0) * (rx - x0) / (x1 - x0); x = rx;
    }
    if (codeOut == code0) { x0 = x; y0 = y; code0 = outCode(x0,y0,rx,ry,rw,rh); }
    else { x1 = x; y1 = y; code1 = outCode(x1,y1,rx,ry,rw,rh); }
  }
}

/* ------------------- Reference symbol & brackets ------------------- */
void drawGarminRefSymbol() {
  const int y  = CY + REF_Y_OFFSET;
  const int noseUp = 4, noseBase = 8, wingOut = 90, wingRise = 6, wingDrop = 8, innerLead = 14;

  tft.fillTriangle(CX, y - noseUp, CX - noseBase, y + noseBase, CX + noseBase, y + noseBase, C_YELLOW);
  tft.drawTriangle(CX, y - noseUp, CX - noseBase, y + noseBase, CX + noseBase, y + noseBase, C_BLACK);

  int Lx1 = CX - innerLead, Ly1 = y + 2;
  int Lx2 = CX - wingOut + 10, Ly2 = y - wingRise;
  int Lx3 = CX - wingOut,      Ly3 = y + wingDrop;
  tft.fillTriangle(Lx1, Ly1, Lx2, Ly2, Lx3, Ly3, C_YELLOW);
  tft.drawTriangle(Lx1, Ly1, Lx2, Ly2, Lx3, Ly3, C_BLACK);

  int Rx1 = CX + innerLead, Ry1 = y + 2;
  int Rx2 = CX + wingOut - 10, Ry2 = y - wingRise;
  int Rx3 = CX + wingOut,      Ry3 = y + wingDrop;
  tft.fillTriangle(Rx1, Ry1, Rx2, Ry2, Rx3, Ry3, C_YELLOW);
  tft.drawTriangle(Rx1, Ry1, Rx2, Ry2, Rx3, Ry3, C_BLACK);

  tft.drawFastHLine(CX - 18, y - 1, 36, C_WHITE);
}
void drawSideBrackets() {
  const int h = 10, w = 22;
  const int y = CY + REF_Y_OFFSET - h/2;
  int xL = ATT_X + 6;
  tft.fillRect(xL, y, w, h, C_YELLOW);
  tft.drawRect(xL, y, w, h, C_BLACK);
  int xR = ATT_X + ATT_W - 6 - w;
  tft.fillRect(xR, y, w, h, C_YELLOW);
  tft.drawRect(xR, y, w, h, C_BLACK);
}

/* ================== INCLINOMETER ================== */
static const int16_t INC_W = 120, INC_H = 16, INC_R = 8;
static const int16_t INC_Y = ATT_Y + ATT_H - 26;
static const float   SLIP_FULL_SCALE_DEG = 8.0f;

void drawInclinometer(float slipDeg, bool force){
  if (!force) if (fabsf(slipDeg - prevSlip) < THR_SLIP) return;
  prevSlip = slipDeg;
  int16_t x0 = CX - INC_W/2;
  int16_t y0 = INC_Y;

  tft.fillRect(x0-2, y0-2, INC_W+4, INC_H+4, C_BLACK);
  tft.fillRect(x0,   y0,   INC_W,   INC_H,   C_GND);
  tft.drawRect(x0,   y0,   INC_W,   INC_H,   C_GREY);

  tft.fillRect(CX-2, y0+2, 4, INC_H-4, C_WHITE);
  tft.fillRect(x0+8, y0+2, 3, INC_H-4, C_WHITE);
  tft.fillRect(x0+INC_W-11, y0+2, 3, INC_H-4, C_WHITE);

  float maxPix = (INC_W/2.0f - INC_R - 3);
  float n = slipDeg / SLIP_FULL_SCALE_DEG; n = n>1?1:(n<-1?-1:n);
  int16_t bx = CX + (int16_t)(n * maxPix);
  int16_t by = y0 + INC_H/2;

  tft.fillCircle(bx, by, INC_R, C_WHITE);
  tft.drawCircle(bx, by, INC_R, C_BLACK);
}

/* --------------- Static frame --------------- */
void drawStaticFrame() {
  C_BLACK  = tft.color565(0,0,0);
  C_WHITE  = tft.color565(255,255,255);
  C_YELLOW = tft.color565(255,255,0);
  C_CYAN   = tft.color565(0,255,255);
  C_GREEN  = tft.color565(0,255,0);
  C_SKY    = tft.color565(70,130,180);
  C_GND    = tft.color565(160,100,50);
  C_GREY   = tft.color565(80,80,80);
  C_RED    = tft.color565(255,0,0);
  C_AMBER  = tft.color565(255,215,0);
  C_PURPLE = tft.color565(180,0,210);

  tft.fillScreen(C_BLACK);
  tft.fillRect(0, 0, TAPE_W, SH, C_BLACK);
  tft.fillRect(SW - TAPE_W, 0, TAPE_W, SH, C_BLACK);
  tft.fillRect(0, 0, SW, TOP_H, C_BLACK);
  tft.drawRect(ATT_X, ATT_Y, ATT_W, ATT_H, C_GREY);

  tft.setTextColor(C_WHITE, C_BLACK);
  tft.setTextSize(1);
  tft.drawString("SPD", 6, 6);
  tft.drawString("ALT", SW - TAPE_W + 6, 6);
  tft.drawString("HDG", CX - 18, 6);
}

/* ============== Attitude core paint (columns) ============== */
static void paintAttitudeColumnsRange(int16_t x_from, int16_t x_to, float pitchDeg, float rollDeg){
  if (x_from < ATT_X) x_from = ATT_X;
  if (x_to   > ATT_X + ATT_W - 1) x_to = ATT_X + ATT_W - 1;
  if (x_from > x_to) return;

  const float rr = deg2rad(rollDeg);
  const float cr = cosf(rr), sr = sinf(rr);
  const float tanr = (fabsf(cr) < 1e-4f) ? (sr>0 ? 1e6f : -1e6f) : (sr/cr);

  const int16_t yTop = ATT_Y;
  const int16_t yBot = ATT_Y + ATT_H - 1;
  const int16_t cy   = ATT_Y + ATT_H/2 + (int16_t)(pitchDeg * PITCH_PX_PER_DEG);

  for (int16_t x = x_from; x <= x_to; ++x) {
    int yh = (int)lroundf(cy + tanr * (x - CX));
    if (yh <= yTop)      tft.drawFastVLine(x, yTop, ATT_H, C_GND);
    else if (yh >= yBot) tft.drawFastVLine(x, yTop, ATT_H, C_SKY);
    else {
      int hSky = yh - yTop;
      if (hSky > 0) tft.drawFastVLine(x, yTop, hSky, C_SKY);
      tft.drawFastVLine(x, yh, yBot - yh + 1, C_GND);
    }
  }
}

/* ================== VSI (overlay) ================== */
void drawVsi(float fpm, bool force){
  if (!force) { if (fabsf(fpm - prevVsi) < THR_VSI) return; }
  prevVsi = fpm;

  const int16_t ALT_X0 = SW - TAPE_W;
  const int16_t VSI_X  = ALT_X0 - 18;
  const float  SCALE   = (ATT_H * 0.35f) / VSI_MAX_FPM;
  const int16_t yMid   = ATT_Y + ATT_H/2;
  const int16_t ySpan  = (int16_t)(VSI_MAX_FPM * SCALE);

  const int16_t ERASE_LEFT  = VSI_X - 14;
  const int16_t ERASE_RIGHT = VSI_X + 2;
  paintAttitudeColumnsRange(ERASE_LEFT, ERASE_RIGHT, fPitch, fRoll);

  tft.drawFastVLine(VSI_X, yMid - ySpan, 2*ySpan, C_WHITE);

  tft.setTextSize(1);
  tft.setTextColor(C_WHITE, C_BLACK);
  for (int v = -VSI_MAX_FPM; v <= VSI_MAX_FPM; v += 500) {
    int16_t y = yMid - (int16_t)(v * SCALE);
    int16_t tickLen = (v % 1000 == 0) ? 8 : 5;
    tft.drawFastHLine(VSI_X - tickLen, y, tickLen, C_WHITE);
    if (v != 0 && (abs(v) % 1000 == 0)) {
      int16_t lx = VSI_X - 12;
      int16_t ly = y - 4;
      tft.drawNumber(abs(v)/1000, lx, ly);
    }
  }
  tft.drawFastHLine(VSI_X - 10, yMid, 20, C_WHITE);

  float n = fpm; n = n>VSI_MAX_FPM?VSI_MAX_FPM:(n<-VSI_MAX_FPM?-VSI_MAX_FPM:n);
  int16_t py = yMid - (int16_t)(n * SCALE);
  int16_t tx1 = VSI_X - 12, ty1 = py;
  int16_t tx2 = VSI_X - 2,  ty2 = py - 5;
  int16_t tx3 = VSI_X - 2,  ty3 = py + 5;
  tft.fillTriangle(tx1, ty1, tx2, ty2, tx3, ty3, C_GREEN);
  tft.drawTriangle(tx1, ty1, tx2, ty2, tx3, ty3, C_BLACK);
}

/* =================== Attitude =================== */
void drawAttitude(float pitchDeg, float rollDeg, bool force) {
  if (!force) {
    if (fabsf(pitchDeg - prevPitch) < THR_PITCH &&
        fabsf(rollDeg  - prevRoll)  < THR_ROLL) {
      drawInclinometer(fSlip, false);
      drawVsi(fVsi, true);
      return;
    }
  }
  prevPitch = pitchDeg; prevRoll = rollDeg;

  paintAttitudeColumnsRange(ATT_X, ATT_X + ATT_W - 1, pitchDeg, rollDeg);

  const float rr = deg2rad(rollDeg);
  const float cr = cosf(rr), sr = sinf(rr);
  int16_t dx = (int16_t)((ATT_W/2 - 3) * cr);
  int16_t dy = (int16_t)((ATT_W/2 - 3) * sr);
  int16_t cy = ATT_Y + ATT_H/2 + (int16_t)(pitchDeg * PITCH_PX_PER_DEG);
  const float nx = -sr, ny = cr;
  for (int k = -(HORIZON_THICK/2); k <= (HORIZON_THICK/2); ++k) {
    int x0 = (CX - dx) + (int)(nx * k);
    int y0 = (cy - dy) + (int)(ny * k);
    int x1 = (CX + dx) + (int)(nx * k);
    int y1 = (cy + dy) + (int)(ny * k);
    int cx0=x0, cy0=y0, cx1=x1, cy1=y1;
    if (clipLineToRect(cx0,cy0,cx1,cy1, ATT_X,ATT_Y,ATT_W,ATT_H)) {
      tft.drawLine(cx0,cy0,cx1,cy1, C_WHITE);
    }
  }

  for (int a = -40; a <= 40; a += 5) {
    float rel = (pitchDeg - (float)a);
    int16_t baseY = (ATT_Y + ATT_H/2) + (int16_t)(rel * PITCH_PX_PER_DEG);
    int L = (a % 10 == 0) ? 48 : 28;

    int16_t xL = CX - L/2, yL = baseY;
    int16_t xR = CX + L/2, yR = baseY;

    float rx1 = (xL - CX), ry1 = (yL - (ATT_Y + ATT_H/2));
    float rx2 = (xR - CX), ry2 = (yR - (ATT_Y + ATT_H/2));
    int16_t ex1 = CX + (int16_t)(cr*rx1 - sr*ry1);
    int16_t ey1 = (ATT_Y + ATT_H/2) + (int16_t)(sr*rx1 + cr*ry1);
    int16_t ex2 = CX + (int16_t)(cr*rx2 - sr*ry2);
    int16_t ey2 = (ATT_Y + ATT_H/2) + (int16_t)(sr*rx2 + cr*ry2);

    int lx0=ex1, ly0=ey1, lx1=ex2, ly1=ey2;
    if (clipLineToRect(lx0,ly0,lx1,ly1, ATT_X,ATT_Y,ATT_W,ATT_H)) {
      tft.drawLine(lx0,ly0,lx1,ly1, C_WHITE);
      if (a % 10 == 0 && a != 0) {
        tft.setTextColor(C_WHITE, C_BLACK);
        tft.setTextSize(1);
        int16_t ty  = ((a>0)? -12:  2);
        int16_t yR  = ly1 + ty;
        int16_t yL  = ly0 + ty;
        int16_t yLo = ATT_Y + 1;
        int16_t yHi = ATT_Y + ATT_H - 10;
        if (yR >= yLo && yR <= yHi) tft.drawNumber(abs(a), lx1 + 6,  yR);
        if (yL >= yLo && yL <= yHi) tft.drawNumber(abs(a), lx0 - 22, yL);
      }
    }
  }

  int16_t arcY = ATT_Y + 22;
  for (int ang=-60; ang<=60; ang+=5) {
    bool major = (ang % 10 == 0);
    bool heavy = (ang % 30 == 0) || (abs(ang)==45) || (abs(ang)==60);
    int h = heavy ? 10 : (major ? 7 : 4);
    float a = deg2rad((float)ang);
    int16_t x0a = CX + (int16_t)(ROLL_ARC_R * sinf(a));
    int16_t y0a = arcY + (int16_t)(ROLL_ARC_R * (1 - cosf(a)));
    tft.fillRect(x0a, y0a, 1, h, C_GREY);
  }

  int16_t px = CX + (int16_t)(ROLL_ARC_R * sinf(deg2rad(rollDeg)));
  int16_t py = arcY + (int16_t)(ROLL_ARC_R * (1 - cosf(deg2rad(rollDeg))));
  tft.drawLine(px-6, py+2, px+6, py+2, C_YELLOW);
  tft.drawLine(px-6, py+2, px,   py-6, C_YELLOW);
  tft.drawLine(px+6, py+2, px,   py-6, C_YELLOW);

  drawGarminRefSymbol();
  drawSideBrackets();

  drawInclinometer(fSlip, true);
  drawVsi(fVsi, true);
}

/* ================== Heading (bug scrolls; tape scrolls at edges) ================== */

// EC11 pins
#define ENC_A   33
#define ENC_B   32
#define ENC_BTN 25

volatile int8_t  encMove = 0;
volatile uint8_t encPrev = 0;
volatile bool encMovedFlag = false;

int   bugStepSlow = 1;
int   bugStepMed  = 5;
int   bugStepFast = 10;
int   pulsesPerDetent = 4;

void IRAM_ATTR encISR() {
  uint8_t a = digitalRead(ENC_A);
  uint8_t b = digitalRead(ENC_B);
  uint8_t curr = (a << 1) | b;
  static const int8_t tbl[16] = {
    0,-1,+1, 0,
   +1, 0, 0,-1,
   -1, 0, 0,+1,
    0,+1,-1, 0
  };
  uint8_t idx = (encPrev << 2) | curr;
  encMove += tbl[idx];
  encPrev = curr;
}

bool readEncButton() {
  static uint32_t tLast = 0;
  static bool     last  = true;
  bool cur = digitalRead(ENC_BTN);
  if (cur != last) { tLast = millis(); last = cur; }
  if (millis() - tLast > 15) return !cur;
  return false;
}

void setupEncoder() {
  pinMode(ENC_A,   INPUT_PULLUP);
  pinMode(ENC_B,   INPUT_PULLUP);
  pinMode(ENC_BTN, INPUT_PULLUP);
  encPrev = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encISR, CHANGE);
}

// keep bug visible by moving view center
void keepBugVisible() {
  const float PX_PER_DEG = 2.0f;
  const float spanDeg = (SW / 2.0f) / PX_PER_DEG;
  const float margin  = 4.0f;
  float delta = wrapDiffDeg(bugHdg, viewHdg);
  if (delta > (spanDeg - margin)) {
    viewHdg = wrapTo360(bugHdg - (spanDeg - margin));
    encMovedFlag = true;
  } else if (delta < -(spanDeg - margin)) {
    viewHdg = wrapTo360(bugHdg + (spanDeg - margin));
    encMovedFlag = true;
  }
}

void serviceEncoder() {
  int8_t delta;
  noInterrupts(); delta = encMove; encMove = 0; interrupts();
  if (delta == 0) return;

  static uint32_t lastTurnUs = 0;
  uint32_t now = micros();
  uint32_t dt  = now - lastTurnUs; if (dt == 0) dt = 1;

  int steps = delta / pulsesPerDetent;
  if (steps == 0) return;

  int stepSize = bugStepSlow;
  if (dt < 120000) stepSize = bugStepMed;
  if (dt <  60000) stepSize = bugStepFast;

  bugHdg = (int)wrapTo360(bugHdg + steps * stepSize);
  keepBugVisible();

  lastTurnUs = now;
  encMovedFlag = true;
}

void handleEncButton() {
  static bool wasPressed = false;
  static uint32_t tPress = 0;

  static bool driftActive   = false;
  static uint32_t driftStartMs= 0;
  static float  driftFrom     = 0;
  static float  driftTo       = 0;
  static const uint16_t DRIFT_MS = 900;

  bool pressed = readEncButton();
  if (pressed && !wasPressed) { tPress = millis(); }
  if (!pressed && wasPressed) {
    uint32_t dur = millis() - tPress;
    driftFrom   = fHdg;
    if (dur < 500) driftTo = bugHdg;
    else {
      int v = (int)roundf(fHdg / 10.0f) * 10;
      driftTo = (float)norm360i(v);
    }
    driftStartMs = millis();
    driftActive  = true;
    encMovedFlag = true;
  }
  wasPressed = pressed;

  // drive drift animation (does not affect attitude)
  if (driftActive) {
    float t = (float)(millis() - driftStartMs) / (float)DRIFT_MS;
    if (t >= 1.0f) {
      fHdg = driftTo;
      viewHdg = fHdg;
      driftActive = false;
    } else {
      float e = easeInOut01(t);
      fHdg   = lerpAngleDeg(driftFrom, driftTo, e);
      viewHdg= fHdg;
    }
    encMovedFlag = true;
  }
}

void drawHeadingBug(float hdgView){
  const float PX_PER_DEG = 2.0f;
  float delta = wrapDiffDeg(bugHdg, hdgView);
  int16_t x = CX + (int16_t)(delta * PX_PER_DEG);
  if (x < 2 || x > SW - 2) return;

  int16_t apexY = TOP_H - 2;
  int16_t baseY = apexY - 10;
  tft.fillTriangle(x, apexY, x-8, baseY, x+8, baseY, C_PURPLE);
  tft.drawTriangle(x, apexY, x-8, baseY, x+8, baseY, C_BLACK);
}

void drawHeading(float hdgView, bool force) {
  if (!force) {
    if (fabsf(wrapDiffDeg(hdgView, prevHdgView)) < THR_HDG && !encMovedFlag) return;
  }
  prevHdgView = hdgView;
  encMovedFlag = false;

  tft.fillRect(0, 0, SW, TOP_H, C_BLACK);

  const float PX_PER_DEG = 2.0f;
  const float spanDeg = (SW / 2.0f) / PX_PER_DEG;
  int startTick = (int)floor((hdgView - spanDeg) / 5.0f) * 5;
  int endTick   = (int)ceil ((hdgView + spanDeg) / 5.0f) * 5;

  for (int tick = startTick; tick <= endTick; tick += 5) {
    float delta = wrapDiffDeg((float)tick, hdgView);
    int16_t x = CX + (int16_t)(delta * PX_PER_DEG);
    if (x < 2 || x > SW - 2) continue;

    int nTick = norm360i(tick);
    bool major30 = (nTick % 30 == 0);
    bool minor10 = (nTick % 10 == 0);
    int16_t h = major30 ? 14 : (minor10 ? 9 : 5);

    tft.fillRect(x, TOP_H-1 - h, 1, h, C_WHITE);
    if (major30) {
      int disp = nTick; if (disp == 360) disp = 0;
      tft.setTextSize(1);
      tft.setTextColor(C_WHITE, C_BLACK);
      tft.drawNumber(disp, x - 8, TOP_H - 1 - h - 12);
    }
  }

  drawHeadingBug(hdgView);

  tft.drawFastHLine(CX-8, TOP_H-2, 17, C_YELLOW);
  tft.drawFastVLine(CX,   TOP_H-18, 16, C_YELLOW);
  tft.setTextSize(2);
  tft.setTextColor(C_WHITE, C_BLACK);
  tft.drawNumber(((int)(fHdg + 0.5f)) % 360, CX - 16, 8);
}

/* ===== Helper: draw color band on airspeed tape ===== */
void drawSpeedBand(float spdCenter, float vLow, float vHigh, uint16_t color) {
  if (vHigh < vLow) { float t = vLow; vLow = vHigh; vHigh = t; }
  int16_t y1 = spdToY(spdCenter, vHigh);
  int16_t y2 = spdToY(spdCenter, vLow);
  if (y2 < y1) { int16_t tt=y1; y1=y2; y2=tt; }

  int16_t yTop = ATT_Y;
  int16_t yBot = ATT_Y + ATT_H;
  if (y2 < yTop || y1 > yBot) return;
  if (y1 < yTop) y1 = yTop;
  if (y2 > yBot) y2 = yBot;

  const int16_t bandX = TAPE_W - 10;
  const int16_t bandW = 6;
  tft.fillRect(bandX, y1, bandW, y2 - y1 + 1, color);
}

/* ================== Airspeed ================== */
void drawAirspeed(float spd, bool force) {
  if (!force) { if (fabsf(spd - prevSpd) < THR_SPD) return; }
  prevSpd = spd;

  tft.fillRect(0, TOP_H, TAPE_W, SH - TOP_H, C_BLACK);

  drawSpeedBand(spd, VSO, VFE, C_WHITE);
  drawSpeedBand(spd, VS1, VNO, C_GREEN);
  drawSpeedBand(spd, VNO, VNE, C_AMBER);

  int16_t yVne = spdToY(spd, VNE);
  if (yVne >= ATT_Y && yVne <= ATT_Y + ATT_H) {
    tft.drawFastHLine(4, yVne-1, TAPE_W-8, C_RED);
    tft.drawFastHLine(4, yVne,   TAPE_W-8, C_RED);
    tft.drawFastHLine(4, yVne+1, TAPE_W-8, C_RED);
  }

  float halfSpanKts = (ATT_H / 2.0f) / ASPD_PX_PER_UNIT;
  int startTick = (int)floor((spd - halfSpanKts) / 10.0f) * 10;
  int endTick   = (int)ceil ((spd + halfSpanKts) / 10.0f) * 10;

  for (int val = startTick; val <= endTick; val += 10) {
    int16_t y = spdToY(spd, (float)val);
    if (y < ATT_Y+2 || y > ATT_Y+ATT_H-2) continue;
    tft.drawFastHLine(TAPE_W-22, y, 18, C_WHITE);
    if (val % 20 == 0) {
      tft.setTextSize(1);
      tft.setTextColor(C_WHITE, C_BLACK);
      tft.drawNumber(val, 6, y-6);
    }
  }

  tft.fillRect(6, CY-14, TAPE_W-12, 28, C_WHITE);
  tft.setTextSize(2);
  tft.setTextColor(C_BLACK, C_WHITE);
  tft.drawNumber((int)(spd+0.5f), 10, CY-10);
}

/* ================== Altitude ================== */
void drawAltitude(float alt, bool force) {
  if (!force) { if (fabsf(alt - prevAlt) < THR_ALT) return; }
  prevAlt = alt;

  int16_t X0 = SW - TAPE_W;
  tft.fillRect(X0, TOP_H, TAPE_W, SH - TOP_H, C_BLACK);

  float halfSpanFt = (ATT_H / 2.0f) / ALTI_PX_PER_FT;
  int startTick = (int)floor((alt - halfSpanFt) / 50.0f) * 50;
  int endTick   = (int)ceil ((alt + halfSpanFt) / 50.0f) * 50;

  for (int val = startTick; val <= endTick; val += 50) {
    int16_t y = CY - (int16_t)((val - alt) * ALTI_PX_PER_FT);
    if (y < ATT_Y+2 || y > ATT_Y+ATT_H-2) continue;
    tft.drawFastHLine(X0+4, y, 16, C_WHITE);
    if (val % 100 == 0) {
      tft.setTextSize(1);
      tft.setTextColor(C_WHITE, C_BLACK);
      tft.drawNumber(val, X0 + TAPE_W - 42, y-6);
    }
  }

  tft.fillRect(X0 + 6, CY-14, TAPE_W-12, 28, C_WHITE);
  tft.setTextSize(2);
  tft.setTextColor(C_BLACK, C_WHITE);
  tft.drawNumber((int)(alt+0.5f), X0 + 10, CY-10);
}

/* -------------------- SETUP ----------------------- */
void setup() {
  Wire.begin(I2C_SDA, I2C_SCL, I2C_HZ);
  Serial.begin(115200);
  delay(200);

  // init two MPUs
  mpuPitchOK = initMPU(MPU_ADDR_PITCH);
  mpuRollOK  = initMPU(MPU_ADDR_ROLL);
  Serial.print("MPU 0x68 (pitch) "); Serial.println(mpuPitchOK ? "OK" : "NOT FOUND");
  Serial.print("MPU 0x69 (roll)  "); Serial.println(mpuRollOK  ? "OK" : "NOT FOUND");

  tft.init();
  tft.setRotation(3);
#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
#endif
  setupEncoder();
  drawStaticFrame();

  // start states
  fHdg = 0; bugHdg = 0; viewHdg = fHdg;

  drawAttitude(fPitch, fRoll, true);
  drawHeading(viewHdg, true);
  drawAirspeed(fSpd, true);
  drawAltitude(fAlt, true);
  drawVsi(fVsi, true);
  drawInclinometer(fSlip, true);

  uint32_t now = millis();
  tLastAtt  = now;
  tLastHdg  = now;
  tLastTape = now;
  tLastVsi  = now;
}

/* --------------------- LOOP ----------------------- */
void loop() {
  uint32_t now = millis();

  // ---------- ATTITUDE from two MPUs ----------
  if (mpuPitchOK) {
    int16_t ax,ay,az;
    if (readAccel(MPU_ADDR_PITCH, ax,ay,az)) srcPitch = accelPitchDeg(ax,ay,az);
  }
  if (mpuRollOK) {
    int16_t ax,ay,az;
    if (readAccel(MPU_ADDR_ROLL, ax,ay,az))  srcRoll  = accelRollDeg(ax,ay,az);
  }

  fPitch = smooth(fPitch, srcPitch, SMOOTH_ATT);
  fRoll  = smooth(fRoll,  srcRoll,  SMOOTH_ATT);

  // ---------- SIMPLE SIMS kept for SPD/ALT only ----------
  float altBase= 3500.0f + 220.0f * sinf(now/4200.0f);
  static float lastAlt = altBase;
  srcVsi   = (altBase - lastAlt) * 60.0f;  // (legacy var; not used for fVsi below)
  lastAlt  = altBase;
  srcAlt   = altBase;
  srcSpd   = 140.0f + 40.0f * sinf(now/3500.0f);

  fSpd   = smooth(fSpd,   srcSpd,   SMOOTH_TAPES);
  fAlt   = smooth(fAlt,   srcAlt,   SMOOTH_TAPES);

  // --- VSI from PITCH (deg → ft/min) ---
  {
    float vsiFromPitch = fPitch * VSI_PER_DEG;
    vsiFromPitch = clampf(vsiFromPitch, -VSI_MAX_FPM, VSI_MAX_FPM);
    fVsi = smooth(fVsi, vsiFromPitch, SMOOTH_VSI);
  }

  // --- Inclinometer from ROLL (ball to low wing) ---
  {
    float slipFromRoll = fRoll; // flip sign if you prefer the opposite
    slipFromRoll = clampf(slipFromRoll, -SLIP_FULL_SCALE_DEG, SLIP_FULL_SCALE_DEG);
    fSlip = smooth(fSlip, slipFromRoll, SMOOTH_SLIP);
  }

  // ---- encoder service (heading only) ----
  serviceEncoder();
  handleEncButton();

  // ---- draws ----
  if (now - tLastAtt  >= PERIOD_ATT)  { tLastAtt  = now; drawAttitude(fPitch, fRoll); }
  if (now - tLastHdg  >= PERIOD_HDG)  { tLastHdg  = now; drawHeading(viewHdg); }
  if (now - tLastTape >= PERIOD_TAPE) { tLastTape = now; drawAirspeed(fSpd); drawAltitude(fAlt); }
  if (now - tLastVsi  >= PERIOD_VSI)  { tLastVsi  = now; drawVsi(fVsi,true); drawInclinometer(fSlip,true); }

  // keep VSI on top
  drawVsi(fVsi, true);

  delay(2);
}
/*
 * ESP32 ST7796 Primary Flight Display (PFD)
 * Author: Abrigo, Denmer John E
	   Ante, John Christian A.
	   Apostol, Peter Francis O.
	   Caballero, Sherwin M.
*/
