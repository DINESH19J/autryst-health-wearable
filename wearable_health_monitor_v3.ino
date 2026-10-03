/*****************************************************************
 *  ESP32 Wearable Health Watch  (v3)
 *  Sensors : AD8232 (ECG), MAX30102 (HR, SpO2, respiration),
 *            MPU6050, GSR v2, TMP117, BMP280
 *  Display : SSD1306 0.96" 6-pin OLED (SPI), watch-style UI
 *  Buttons : NEXT / OK / BACK
 *  NOT a medical device. Educational prototype.
 *
 *  SERIAL_MODE:  0 = CSV log (1 line/second)
 *                1 = filtered ECG for Serial Plotter (115200 baud)
 *                2 = GSR debug for Serial Plotter: raw,fast,tonic,stress
 *****************************************************************/
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MAX30105.h>
#include <heartRate.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_TMP117.h>
#include <Adafruit_Sensor.h>

// ===================== 1. PIN MAP =====================
#define I2C_SDA       21
#define I2C_SCL       22
#define PIN_ECG_OUT   34
#define PIN_ECG_LO_P  32
#define PIN_ECG_LO_N  33
#define PIN_GSR       35
#define BTN_NEXT      25
#define BTN_OK        26
#define BTN_BACK      27
#define PIN_BUZZER    13
#define OLED_SCK      18
#define OLED_MISO     19
#define OLED_MOSI     23
#define OLED_DC       17
#define OLED_RST      16
#define OLED_CS        4

// ===================== 2. USER SETTINGS =====================
const int   SERIAL_MODE      = 0;
const float USER_WEIGHT_KG   = 70.0f;
const int   USER_AGE         = 21;
const float REST_HR_DEFAULT  = 70.0f;
const float SEA_LEVEL_HPA    = 1013.25f;
const float MAINS_HZ         = 50.0f;     // India = 50, USA = 60 (ECG notch filter)
const bool  BTN_ACTIVE_HIGH  = false;     // false = 4-pin tactile buttons to GND
// Calibration (compare with a reference device, then adjust)
const float SPO2_A           = 110.0f;    // SpO2 = A - B * R
const float SPO2_B           = 25.0f;
const float TEMP_OFFSET      = 0.0f;
const float PPG_FINGER_MIN   = 50000.0f;  // IR value above this = finger present
// ECG
const float ECG_MIN_AMP      = 80.0f;     // min R-peak amplitude (filtered ADC counts)
// Respiration (from PPG baseline modulation)
const float RESP_MIN_AMP     = 20.0f;
// GSR
const bool  GSR_INVERTED     = false;     // true if reading DROPS when skin gets sweaty
const float GSR_MIN_RAW      = 150.0f;    // below/above these = electrodes not touching
const float GSR_MAX_RAW      = 3950.0f;
const uint32_t GSR_WARMUP_MS = 30000;     // baseline calibration time
const float GSR_SCR_REL      = 0.010f;    // 1% rise over baseline = one skin-conductance response
const float GSR_SCR_FULL     = 8.0f;      // responses per minute that equal stress 100

// Alerts
const float HR_REST_MAX  = 110.0f;
const float HR_ABS_MAX   = 220.0f - USER_AGE;
const float SPO2_MIN     = 92.0f;
const float STRESS_MAX   = 75.0f;
const float TEMP_MAX     = 38.0f;
const float RESP_LOW     = 8.0f;
const float RESP_HIGH    = 25.0f;
const uint16_t ALERT_HOLD_S = 10;

// ===================== 3. DATA MODEL =====================
enum Activity { ACT_REST, ACT_LIGHT, ACT_MODERATE, ACT_VIGOROUS };
enum Page { P_HOME, P_HEART, P_SPO2, P_ECG, P_RESP, P_TEMP, P_STRESS, P_ACT, P_SCORES, P_COUNT };
const char* ACT_NAMES[4] = {"Resting", "Light", "Moderate", "Vigorous"};

#define A_HR     0x01
#define A_SPO2   0x02
#define A_STRESS 0x04
#define A_TEMP   0x08
#define A_FALL   0x10
#define A_RESP   0x20

const uint8_t BTN_PINS[3] = {BTN_NEXT, BTN_OK, BTN_BACK};
bool     btnLast[3] = {false, false, false};
uint32_t btnTime[3] = {0, 0, 0};
#define B_NEXT 0
#define B_OK   1
#define B_BACK 2

struct Vitals {
  volatile bool     leadsOff = true;
  volatile float    ecgBpm = 0, hrvRmssd = 0, ecgQ = 0;
  volatile uint32_t lastBeat = 0;
  float hr = 0, spo2 = 0, resp = 0;
  bool  finger = false, ppgOk = true;
  uint32_t fingerSince = 0;
  float skinTemp = 0;
  float accDyn = 0;  Activity activity = ACT_REST;
  uint32_t steps = 0;  bool fall = false;
  float relAlt = 0, floors = 0;  bool climbing = false;
  float kcal = 0, gsrRaw = 0, gsrCond = 0, gsrBase = 0, stress = 0;
  float fatigue = 0, fitness = 0, risk = 0, sleepQuality = 0;
  uint16_t alerts = 0;
} v;

#define WAVE_W 128
volatile int16_t ecgWave[WAVE_W];   volatile uint8_t ecgHead = 0;
int16_t respWave[WAVE_W];           uint8_t respHead = 0;
volatile bool ecgFrozen = false;

float restHr = REST_HR_DEFAULT;
bool  sleepMode = false;
Page  page = P_HOME;
uint32_t muteUntil = 0;

// GSR state
bool  gsrContact = false, gsrReady = false;
uint32_t gsrWarmStart = 0;
float gsrFast = 0, gsrTonic = 0, gsrPhasic = 0;
uint32_t scrT[10];  uint8_t scrN = 0, scrI = 0;

// ===================== 4. OBJECTS =====================
Adafruit_SSD1306 oled(128, 64, &SPI, OLED_DC, OLED_RST, OLED_CS);
MAX30105         ppg;
Adafruit_MPU6050 mpu;
Adafruit_BMP280  bmp(&Wire);
Adafruit_TMP117  tmp;

// Simple biquad filter (used for ECG notch + low-pass)
struct Biquad {
  float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
  void setLowpass(float fs, float fc, float q) {
    float w = 2 * PI * fc / fs, c = cosf(w), al = sinf(w) / (2 * q), a0 = 1 + al;
    b0 = (1 - c) / 2 / a0; b1 = (1 - c) / a0; b2 = b0; a1 = -2 * c / a0; a2 = (1 - al) / a0;
  }
  void setNotch(float fs, float f0, float q) {
    float w = 2 * PI * f0 / fs, c = cosf(w), al = sinf(w) / (2 * q), a0 = 1 + al;
    b0 = 1 / a0; b1 = -2 * c / a0; b2 = b0; a1 = -2 * c / a0; a2 = (1 - al) / a0;
  }
  float run(float x) {
    float y = b0 * x + z1;
    z1 = b1 * x - a1 * y + z2;
    z2 = b2 * x - a2 * y;
    return y;
  }
};
Biquad notchF, lowF;

// ===================== 5. HELPERS =====================
bool due(uint32_t &last, uint32_t period) {
  uint32_t n = millis();
  if (n - last >= period) { last = n; return true; }
  return false;
}

float bestHR() {   // ECG when it is clean, otherwise PPG
  if (!v.leadsOff && v.ecgBpm > 0) return v.ecgBpm;
  if (v.finger && v.hr > 0) return v.hr;
  return 0;
}

void beep(uint16_t ms) { digitalWrite(PIN_BUZZER, HIGH); delay(ms); digitalWrite(PIN_BUZZER, LOW); }

int measuringPct() {       // 0..100 after finger placed (12 s settle time)
  if (!v.finger) return 0;
  uint32_t d = millis() - v.fingerSince;
  return d >= 12000 ? 100 : (int)(d / 120);
}

bool beatPulse() { return millis() - v.lastBeat < 160; }

// ===================== 6. INIT =====================
void initOLED() {
  if (!oled.begin(SSD1306_SWITCHCAPVCC)) Serial.println("OLED FAIL");
  oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0); oled.println("Health Watch"); oled.println("Starting...");
  oled.display();
}

void initFail(const char* name) {
  Serial.printf("%s init FAILED\n", name);
  oled.println(name); oled.println("FAILED"); oled.display(); delay(1000);
}

void initSensors() {
  if (!ppg.begin(Wire, I2C_SPEED_FAST)) initFail("MAX30102");
  ppg.setup(0x3F, 4, 2, 400, 411, 4096);   // 100 Hz effective sample rate

  if (!mpu.begin()) initFail("MPU6050");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  if (!bmp.begin(0x76)) initFail("BMP280");
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_63);

  if (!tmp.begin()) initFail("TMP117");
}

// ===================== 7. ECG (runs on core 0) =====================
static uint16_t rr[8];  static uint8_t rrN = 0, rrI = 0;

void pushRR(uint16_t ms) {
  rr[rrI] = ms; rrI = (rrI + 1) % 8; if (rrN < 8) rrN++;
  if (rrN < 5) return;
  uint16_t t[8];
  for (uint8_t i = 0; i < rrN; i++) t[i] = rr[i];
  for (uint8_t i = 1; i < rrN; i++) {          // insertion sort -> median
    uint16_t k = t[i]; int j = i - 1;
    while (j >= 0 && t[j] > k) { t[j + 1] = t[j]; j--; }
    t[j + 1] = k;
  }
  float med = t[rrN / 2];
  uint8_t good = 0;
  for (uint8_t i = 0; i < rrN; i++) if (fabsf((float)rr[i] - med) < 0.25f * med) good++;
  v.ecgQ = 100.0f * good / rrN;                // beat-to-beat consistency
  if (v.ecgQ >= 60.0f) {
    v.ecgBpm = 60000.0f / med;
    uint8_t o = (rrI + 8 - rrN) % 8;  float sq = 0;
    for (uint8_t j = 1; j < rrN; j++) {
      float d = (float)rr[(o + j) % 8] - (float)rr[(o + j - 1) % 8];
      sq += d * d;
    }
    v.hrvRmssd = (v.ecgQ >= 80.0f) ? sqrtf(sq / (rrN - 1)) : 0;
  } else { v.ecgBpm = 0; v.hrvRmssd = 0; }
}

void detectR(float f) {    // R-peak on the FILTERED signal (works for upright or inverted QRS)
  static float peak = 0, prev = 0, prev2 = 0;  static uint32_t lastR = 0;
  float x = fabsf(f);
  peak = (x > peak) ? x : peak * 0.9985f;
  float th = 0.6f * peak;
  uint32_t now = millis();
  float p = fabsf(prev);
  if (p > th && p > fabsf(prev2) && p >= x && peak > ECG_MIN_AMP && now - lastR > 300) {
    if (lastR && now - lastR < 2000) pushRR(now - lastR);
    lastR = now;  v.lastBeat = now;
  }
  prev2 = prev; prev = f;
  if (now - lastR > 3000) { v.ecgBpm = 0; v.hrvRmssd = 0; v.ecgQ = 0; rrN = 0; }
}

void ecgTask(void*) {      // 250 Hz sampling
  TickType_t t = xTaskGetTickCount();
  uint8_t ds = 0;  float pIn = 0, pOut = 0;
  for (;;) {
    vTaskDelayUntil(&t, pdMS_TO_TICKS(4));
    if (digitalRead(PIN_ECG_LO_P) || digitalRead(PIN_ECG_LO_N)) { v.leadsOff = true; pIn = pOut = 0; continue; }
    v.leadsOff = false;
    long sum = 0;
    for (int i = 0; i < 8; i++) sum += analogRead(PIN_ECG_OUT);     // oversample to cut ADC noise
    float x = sum / 8.0f;
    float hp = x - pIn + 0.995f * pOut;  pIn = x; pOut = hp;          // remove DC / baseline wander
    float f = lowF.run(notchF.run(hp));                                // 50 Hz notch + 35 Hz low-pass
    if (SERIAL_MODE == 1) Serial.println((int)f);
    if (!ecgFrozen && ++ds % 2 == 0) {
      ecgWave[ecgHead] = (int16_t)constrain(f, -2000.0f, 2000.0f);
      ecgHead = (ecgHead + 1) % WAVE_W;
    }
    detectR(f);
  }
}

// ===================== 8. RESPIRATION (from PPG) =====================
void updateResp(float r) {  // r = band-passed IR signal (0.05 - 0.6 Hz), called at 100 Hz
  static float amp = 0;  static bool wasLow = false;  static uint32_t lastB = 0;
  static float iv[4] = {0};  static uint8_t ivN = 0, ivI = 0;
  uint32_t now = millis();
  if (v.activity >= ACT_MODERATE) { v.resp = 0; ivN = 0; return; }   // moving: unreliable
  float a = fabsf(r);
  amp = (a > amp) ? a : amp * 0.9995f;
  float h = 0.35f * amp;
  if (amp > RESP_MIN_AMP) {
    if (r < -h) wasLow = true;
    if (r > h && wasLow && now - lastB > 1500) {          // one full breath cycle
      if (lastB && now - lastB < 10000) { iv[ivI] = now - lastB; ivI = (ivI + 1) % 4; if (ivN < 4) ivN++; }
      lastB = now;  wasLow = false;
      if (ivN >= 2) { float s = 0; for (uint8_t i = 0; i < ivN; i++) s += iv[i]; v.resp = 60000.0f * ivN / s; }
    }
  }
  if (now - lastB > 15000) { v.resp = 0; ivN = 0; }
}

// ===================== 9. HEART RATE + SpO2 (MAX30102) =====================
void readHeartSpO2() {
  ppg.check();
  static uint32_t lastBeat = 0;  static float rates[4] = {0};  static uint8_t ri = 0;
  static float irDC = 0, redDC = 0, irSq = 0, redSq = 0, spo2Avg = 0;  static uint8_t n = 0;
  static float rFast = 0, rSlow = 0;  static uint8_t rd = 0;

  while (ppg.available()) {
    float ir = ppg.getFIFOIR(), red = ppg.getFIFORed();
    ppg.nextSample();
    bool f = ir > PPG_FINGER_MIN;
    if (f && !v.finger) v.fingerSince = millis();
    v.finger = f;
    if (!f) {
      v.hr = 0; v.spo2 = 0; v.resp = 0;
      irDC = redDC = irSq = redSq = spo2Avg = 0; n = 0; rFast = rSlow = 0;
      memset(rates, 0, sizeof(rates)); continue;
    }
    // heart rate
    if (checkForBeat((long)ir)) {
      uint32_t now = millis();  uint32_t d = now - lastBeat;  lastBeat = now;
      float bpm = 60000.0f / d;
      if (bpm > 35 && bpm < 200) {
        rates[ri++ % 4] = bpm;
        float s = 0; uint8_t c = 0;
        for (uint8_t i = 0; i < 4; i++) if (rates[i] > 0) { s += rates[i]; c++; }
        v.hr = s / c;
        v.lastBeat = now;
      }
    }
    // SpO2 (ratio of ratios)
    if (irDC == 0) { irDC = ir; redDC = red; }
    irDC += 0.01f * (ir - irDC);  redDC += 0.01f * (red - redDC);
    irSq += sq(ir - irDC);        redSq += sq(red - redDC);
    if (++n >= 100) {
      float R = (sqrtf(redSq / n) / redDC) / (sqrtf(irSq / n) / irDC);
      float s = constrain(SPO2_A - SPO2_B * R, 70.0f, 100.0f);
      spo2Avg = (spo2Avg == 0) ? s : 0.8f * spo2Avg + 0.2f * s;
      v.spo2 = spo2Avg;  n = 0; irSq = redSq = 0;
    }
    // respiration: breathing modulates the IR baseline at 0.1 - 0.5 Hz
    if (rSlow == 0) rFast = rSlow = ir;
    rFast += 0.04f * (ir - rFast);
    rSlow += 0.003f * (ir - rSlow);
    float r = rFast - rSlow;
    updateResp(r);
    if (++rd >= 10) { rd = 0; respWave[respHead] = (int16_t)constrain(r, -1000.0f, 1000.0f); respHead = (respHead + 1) % WAVE_W; }
  }
}

// ===================== 10. MOTION (MPU6050) =====================
void readMotion() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  float mag = sqrtf(a.acceleration.x * a.acceleration.x +
                    a.acceleration.y * a.acceleration.y +
                    a.acceleration.z * a.acceleration.z);
  float dyn = fabsf(mag - 9.81f);
  v.accDyn = 0.9f * v.accDyn + 0.1f * dyn;
  v.activity = v.accDyn < 0.25f ? ACT_REST : v.accDyn < 1.2f ? ACT_LIGHT :
               v.accDyn < 3.0f ? ACT_MODERATE : ACT_VIGOROUS;

  static bool armed = true;  static uint32_t lastStep = 0;
  if (mag > 11.5f && armed && millis() - lastStep > 300) { v.steps++; lastStep = millis(); armed = false; }
  if (mag < 10.3f) armed = true;

  static uint32_t freeFallAt = 0, fallAt = 0;
  if (mag < 3.0f) freeFallAt = millis();
  if (mag > 25.0f && freeFallAt && millis() - freeFallAt < 600) { v.fall = true; fallAt = millis(); freeFallAt = 0; }
  if (v.fall && millis() - fallAt > 15000) v.fall = false;
}

// ===================== 11. ALTITUDE / STAIRS (BMP280) =====================
void readAltitude() {
  static float alt = 0, last = 0, baseAlt = 0;  static bool init = false;
  float a = bmp.readAltitude(SEA_LEVEL_HPA);
  if (!init) { baseAlt = alt = last = a; init = true; }
  alt = 0.8f * alt + 0.2f * a;
  v.relAlt = alt - baseAlt;
  float rate = alt - last;  last = alt;
  v.climbing = (rate > 0.12f && v.activity >= ACT_LIGHT);
  if (v.climbing) v.floors += rate / 3.0f;
}

// ===================== 12. TEMPERATURE (TMP117) =====================
void readTemperature() {
  sensors_event_t temp;
  tmp.getEvent(&temp);
  v.skinTemp = temp.temperature + TEMP_OFFSET;
}

// ===================== 13. STRESS (GSR skin-conductance responses + HR) =====================
void gsrRecalibrate() { gsrFast = 0; gsrReady = false; scrN = 0; }

void readGSR() {           // 10 Hz
  long sum = 0;
  for (int i = 0; i < 16; i++) sum += analogRead(PIN_GSR);
  float raw = sum / 16.0f;
  v.gsrRaw = raw;
  gsrContact = (raw > GSR_MIN_RAW && raw < GSR_MAX_RAW);
  if (!gsrContact) { gsrFast = 0; gsrReady = false; return; }   // fingers off: restart calibration

  float cond = GSR_INVERTED ? (4095.0f - raw) : raw;
  if (gsrFast == 0) { gsrFast = gsrTonic = cond; gsrWarmStart = millis(); scrN = 0; gsrReady = false; }
  gsrFast += 0.2f * (cond - gsrFast);                           // smoothed signal (~0.5 s)
  bool warm = millis() - gsrWarmStart < GSR_WARMUP_MS;
  gsrTonic += (warm ? 0.05f : 0.003f) * (gsrFast - gsrTonic);   // baseline: fast learn, then slow track
  gsrPhasic = gsrFast - gsrTonic;
  v.gsrCond = gsrFast;  v.gsrBase = gsrTonic;

  if (!warm) {
    gsrReady = true;
    static bool inSCR = false;  static uint32_t lastSCR = 0;
    float rel = gsrPhasic / fmaxf(gsrTonic, 300.0f);
    uint32_t now = millis();
    if (!inSCR && rel > GSR_SCR_REL && now - lastSCR > 1500) {   // a new response
      inSCR = true; lastSCR = now; scrT[scrI] = now; scrI = (scrI + 1) % 10; if (scrN < 10) scrN++;
    } else if (inSCR && rel < GSR_SCR_REL * 0.4f) inSCR = false;
  }
  if (SERIAL_MODE == 2) Serial.printf("%.0f,%.0f,%.0f,%.0f\n", raw, gsrFast, gsrTonic, v.stress);
}

float scrRate() {          // skin-conductance responses per minute
  uint32_t now = millis();  uint8_t c = 0;
  for (uint8_t i = 0; i < scrN; i++) if (now - scrT[i] < 60000) c++;
  float win = fminf(60.0f, (now - gsrWarmStart - GSR_WARMUP_MS) / 1000.0f);
  return c * 60.0f / fmaxf(win, 20.0f);
}

int gsrSecondsLeft() {
  uint32_t d = millis() - gsrWarmStart;
  return d >= GSR_WARMUP_MS ? 0 : (int)((GSR_WARMUP_MS - d) / 1000);
}

void fuseStress() {        // 1 Hz
  if (!gsrContact || !gsrReady) { v.stress = 0; return; }
  if (v.activity >= ACT_MODERATE) return;                       // exercise is not stress: hold value
  float g = constrain(scrRate() * 100.0f / GSR_SCR_FULL, 0.0f, 100.0f);
  float s = g;
  if (v.finger && v.ppgOk && v.hr > 0 && v.activity <= ACT_LIGHT) {
    float h = constrain((v.hr - restHr) * 2.0f, 0.0f, 100.0f);
    s = 0.7f * g + 0.3f * h;
  }
  v.stress = 0.8f * v.stress + 0.2f * s;
}

// ===================== 14. SENSOR FUSION =====================
void crossCheckHR() {
  if (!v.leadsOff && v.ecgBpm > 0 && v.finger && v.hr > 0)
    v.ppgOk = fabsf(v.ecgBpm - v.hr) < 12.0f;
  else v.ppgOk = true;
}

void updateCalories() {
  const float MET[4] = {1.0f, 2.5f, 4.5f, 8.0f};
  float met = MET[v.activity] + (v.climbing ? 3.0f : 0.0f);
  float kcalMin = met * 3.5f * USER_WEIGHT_KG / 200.0f;
  float hr = bestHR();
  if (hr > 100) {
    float k = (-55.0969f + 0.6309f * hr + 0.1988f * USER_WEIGHT_KG + 0.2017f * USER_AGE) / 4.184f;
    kcalMin = 0.5f * kcalMin + 0.5f * fmaxf(k, 0.0f);
  }
  v.kcal += kcalMin / 60.0f;
}

void updateFatigue() {
  static float effBase = 0, effNow = 0;  static uint16_t sec = 0;
  float hr = bestHR();
  if (hr <= 0 || v.activity < ACT_MODERATE) return;
  float eff = v.accDyn / (fmaxf(hr - restHr, 0.0f) + 10.0f);
  sec++;
  if (sec < 180) { effBase += (eff - effBase) / sec; effNow = effBase; }
  else {
    effNow = 0.98f * effNow + 0.02f * eff;
    v.fatigue = constrain((1.0f - effNow / fmaxf(effBase, 0.001f)) * 250.0f, 0.0f, 100.0f);
  }
}

void updateSleep() {
  static uint16_t sec = 0, moves = 0;  static uint32_t epochs = 0, quiet = 0;
  if (v.accDyn > 0.15f) moves++;
  if (++sec >= 30) {
    epochs++;  if (moves <= 3) quiet++;
    sec = 0; moves = 0;
    v.sleepQuality = 100.0f * quiet / epochs;
  }
}

// ===================== 15. SCORES + ALERTS =====================
void updateScores() {
  float hr = bestHR();
  if (hr > 40 && v.activity == ACT_REST) restHr = constrain(restHr + 0.01f * (hr - restHr), 45.0f, 100.0f);

  v.fitness = constrain(v.steps / 8000.0f, 0.0f, 1.0f) * 40.0f
            + constrain((90.0f - restHr) / 40.0f, 0.0f, 1.0f) * 30.0f
            + (100.0f - v.fatigue) * 0.30f;

  float r = 0;  bool rest = v.activity <= ACT_LIGHT;
  if (hr > 0 && rest) { if (hr > 120) r += 25; else if (hr > 100) r += 15; else if (hr < 50) r += 10; }
  if (v.finger && v.spo2 > 0 && v.ppgOk) { if (v.spo2 < 90) r += 35; else if (v.spo2 < 92) r += 25; else if (v.spo2 < 95) r += 10; }
  if (v.resp > 0 && rest) { if (v.resp > RESP_HIGH || v.resp < RESP_LOW) r += 10; }
  if (v.skinTemp > 38.0f) r += 20; else if (v.skinTemp > 37.5f) r += 10;
  if (v.stress > 80) r += 15; else if (v.stress > 60) r += 10;
  if (v.hrvRmssd > 0 && v.hrvRmssd < 20) r += 10;
  if (v.fatigue > 60) r += 5;
  v.risk = constrain(r, 0.0f, 100.0f);
}

const char* riskLabel() { return v.risk < 25 ? "LOW" : v.risk < 50 ? "MODERATE" : "HIGH"; }

void checkAlerts() {
  static uint16_t hrC = 0, spC = 0, stC = 0, tC = 0, rC = 0;
  float hr = bestHR();
  bool hrBad = hr > 0 && ((v.activity <= ACT_LIGHT && hr > HR_REST_MAX) || hr > HR_ABS_MAX);
  hrC = hrBad ? min<uint16_t>(hrC + 1, 999) : 0;
  spC = (v.finger && v.ppgOk && v.spo2 > 0 && v.spo2 < SPO2_MIN) ? min<uint16_t>(spC + 1, 999) : 0;
  stC = (v.stress > STRESS_MAX) ? min<uint16_t>(stC + 1, 999) : 0;
  tC  = (v.skinTemp > TEMP_MAX) ? min<uint16_t>(tC + 1, 999) : 0;
  bool rBad = v.resp > 0 && v.activity <= ACT_LIGHT && (v.resp > RESP_HIGH || v.resp < RESP_LOW);
  rC  = rBad ? min<uint16_t>(rC + 1, 999) : 0;

  v.alerts = 0;
  if (hrC >= ALERT_HOLD_S) v.alerts |= A_HR;
  if (spC >= ALERT_HOLD_S) v.alerts |= A_SPO2;
  if (stC >= ALERT_HOLD_S) v.alerts |= A_STRESS;
  if (tC  >= ALERT_HOLD_S) v.alerts |= A_TEMP;
  if (rC  >= 20)           v.alerts |= A_RESP;
  if (v.fall)              v.alerts |= A_FALL;

  if (v.alerts && millis() > muteUntil && !sleepMode) beep(100);
}

const char* alertText() {
  if (v.alerts & A_FALL)   return "FALL DETECTED!";
  if (v.alerts & A_SPO2)   return "LOW SpO2!";
  if (v.alerts & A_HR)     return "HIGH HEART RATE!";
  if (v.alerts & A_RESP)   return "ABNORMAL BREATHING";
  if (v.alerts & A_TEMP)   return "HIGH TEMPERATURE!";
  if (v.alerts & A_STRESS) return "HIGH STRESS!";
  return "";
}

// ===================== 16. BUTTONS =====================
bool pressed(uint8_t i) {
  bool now = BTN_ACTIVE_HIGH ? digitalRead(BTN_PINS[i]) : !digitalRead(BTN_PINS[i]);
  bool ev = now && !btnLast[i] && millis() - btnTime[i] > 50;
  if (now != btnLast[i]) btnTime[i] = millis();
  btnLast[i] = now;
  return ev;
}

void doOkAction() {
  switch (page) {
    case P_HOME:   case P_SCORES: sleepMode = true; break;
    case P_ECG:    ecgFrozen = !ecgFrozen; break;
    case P_ACT:    v.steps = 0; v.kcal = 0; v.floors = 0; break;
    case P_STRESS: gsrRecalibrate(); v.stress = 0; break;
    default: break;
  }
}

void handleButtons() {
  bool n = pressed(B_NEXT), o = pressed(B_OK), b = pressed(B_BACK);
  if (sleepMode) { if (o) sleepMode = false; return; }
  if (n) page = (Page)((page + 1) % P_COUNT);
  if (o) doOkAction();
  if (b) {
    if (v.alerts) { muteUntil = millis() + 60000; v.fall = false; }
    else page = P_HOME;
  }
}

// ===================== 17. ICONS (drawn with shapes) =====================
void drawHeart(int cx, int cy, int r) {
  int q = r / 2; if (q < 1) q = 1;
  oled.fillCircle(cx - q, cy - q / 2, q, SSD1306_WHITE);
  oled.fillCircle(cx + q, cy - q / 2, q, SSD1306_WHITE);
  oled.fillTriangle(cx - r, cy - q / 2 + 1, cx + r, cy - q / 2 + 1, cx, cy + r + q / 2, SSD1306_WHITE);
}

void drawDrop(int cx, int cy, int r) {
  oled.fillCircle(cx, cy, r, SSD1306_WHITE);
  oled.fillTriangle(cx - (r * 87) / 100, cy - r / 2, cx + (r * 87) / 100, cy - r / 2, cx, cy - 2 * r, SSD1306_WHITE);
}

void drawThermo(int x, int y, int pct) {
  oled.drawRoundRect(x + 2, y, 8, 26, 4, SSD1306_WHITE);
  oled.fillCircle(x + 6, y + 30, 6, SSD1306_WHITE);
  int h = constrain(pct, 0, 100) * 20 / 100;
  oled.fillRect(x + 5, y + 24 - h, 2, h + 2, SSD1306_WHITE);
}

void drawLungs(int x, int y, int e) {
  oled.fillRoundRect(x - 14 - e, y + 6, 12 + e, 22, 6, SSD1306_WHITE);
  oled.fillRoundRect(x + 2, y + 6, 12 + e, 22, 6, SSD1306_WHITE);
  oled.drawLine(x, y, x, y + 12, SSD1306_WHITE);
  oled.drawLine(x, y + 12, x - 5, y + 18, SSD1306_WHITE);
  oled.drawLine(x, y + 12, x + 5, y + 18, SSD1306_WHITE);
}

void drawSteps(int x, int y) {
  oled.fillRoundRect(x, y + 6, 6, 10, 3, SSD1306_WHITE);
  oled.fillRoundRect(x + 9, y, 6, 10, 3, SSD1306_WHITE);
}

void drawBolt(int x, int y) {
  oled.fillTriangle(x + 10, y, x + 1, y + 14, x + 9, y + 14, SSD1306_WHITE);
  oled.fillTriangle(x + 5, y + 12, x + 14, y + 12, x + 4, y + 26, SSD1306_WHITE);
}

void drawBar(int x, int y, int w, int h, float pct) {
  oled.drawRect(x, y, w, h, SSD1306_WHITE);
  int f = (int)((w - 4) * constrain(pct, 0.0f, 100.0f) / 100.0f);
  oled.fillRect(x + 2, y + 2, f, h - 4, SSD1306_WHITE);
}

void drawMeasuring(int pct, const char* what) {
  drawHeart(24, 24, (millis() / 400) % 2 ? 14 : 11);
  oled.setTextSize(1);
  oled.setCursor(52, 12); oled.print("Measuring");
  int dots = (millis() / 400) % 4;
  for (int i = 0; i < dots; i++) oled.print(".");
  oled.setCursor(52, 26); oled.print(what);
  drawBar(0, 46, 128, 8, pct);
}

// ===================== 18. SCREENS =====================
void pageHome() {
  float hr = bestHR();
  drawHeart(17, 16, beatPulse() ? 11 : 8);
  oled.setTextSize(3); oled.setCursor(42, 6);
  if (hr > 0) oled.print((int)hr); else oled.print("--");
  oled.setTextSize(1); oled.setCursor(100, 26); oled.print("bpm");
  oled.drawFastHLine(0, 40, 128, SSD1306_WHITE);
  drawDrop(5, 51, 3);
  oled.setCursor(12, 47);
  if (v.spo2 > 0) oled.printf("%.0f%%%s", v.spo2, v.ppgOk ? "" : "?"); else oled.print("--");
  oled.setCursor(48, 47);
  if (v.skinTemp > 0) oled.printf("T%.1f", v.skinTemp); else oled.print("T--");
  oled.setCursor(92, 47);
  if (v.resp > 0) oled.printf("R%.0f", v.resp); else oled.print("R--");
}

void pageHeart() {
  float hr = bestHR();
  if (v.finger && v.leadsOff && measuringPct() < 100) { drawMeasuring(measuringPct(), "Keep finger still"); return; }
  if (hr <= 0) {
    drawHeart(24, 24, 11);
    oled.setCursor(52, 14); oled.print("Place finger");
    oled.setCursor(52, 26); oled.print("or attach ECG");
    return;
  }
  drawHeart(22, 22, beatPulse() ? 15 : 12);
  oled.setTextSize(3); oled.setCursor(52, 6); oled.print((int)hr);
  oled.setTextSize(1); oled.setCursor(52, 32); oled.print("bpm");
  oled.setCursor(0, 44);
  oled.printf("PPG %.0f  ECG %.0f", v.finger ? v.hr : 0.0f, v.leadsOff ? 0.0f : (float)v.ecgBpm);
}

void pageSpo2() {
  if (!v.finger) { drawDrop(22, 28, 10); oled.setCursor(44, 24); oled.print("Place finger"); return; }
  if (measuringPct() < 100 || v.spo2 <= 0) { drawMeasuring(measuringPct(), "Reading SpO2"); return; }
  drawDrop(20, 30, 11);
  oled.setTextSize(3); oled.setCursor(46, 6); oled.printf("%.0f", v.spo2);
  oled.setTextSize(2); oled.print("%");
  oled.setTextSize(1);
  drawBar(0, 40, 128, 8, (v.spo2 - 80.0f) * 5.0f);
  oled.setCursor(0, 50); oled.print(v.ppgOk ? "Signal good" : "Noisy - hold still");
}

void pageEcg() {
  oled.setCursor(0, 0); oled.print("ECG");
  drawHeart(40, 4, 4);
  oled.setCursor(56, 0);
  if (v.leadsOff) { oled.print("Attach electrodes"); return; }
  oled.printf("%.0fbpm Q%.0f%% %s", (float)v.ecgBpm, (float)v.ecgQ, ecgFrozen ? "FRZ" : "");
  int mn = 32767, mx = -32768;
  for (int i = 0; i < WAVE_W; i++) { int s = ecgWave[i]; if (s < mn) mn = s; if (s > mx) mx = s; }
  if (mx - mn < 60) { int m = (mx + mn) / 2; mn = m - 30; mx = m + 30; }
  int py = 0;
  for (int x = 0; x < WAVE_W; x++) {
    int s = ecgWave[(ecgHead + x) % WAVE_W];
    int y = map(s, mn, mx, 52, 14);
    if (x > 0) oled.drawLine(x - 1, py, x, y, SSD1306_WHITE);
    py = y;
  }
}

void pageResp() {
  int e = (respWave[(respHead + WAVE_W - 1) % WAVE_W] > 0) ? 3 : 0;
  drawLungs(22, 4, e);
  oled.setTextSize(3); oled.setCursor(52, 6);
  if (v.resp > 0) oled.printf("%.0f", v.resp); else oled.print("--");
  oled.setTextSize(1); oled.setCursor(52, 32); oled.print("breaths/min");
  if (!v.finger) { oled.setCursor(0, 44); oled.print("Place finger, stay still"); return; }
  int py = 0;
  for (int x = 0; x < WAVE_W; x++) {
    int s = respWave[(respHead + x) % WAVE_W];
    int y = constrain(map(s, -150, 150, 54, 40), 38, 54);
    if (x > 0) oled.drawLine(x - 1, py, x, y, SSD1306_WHITE);
    py = y;
  }
}

void pageTemp() {
  drawThermo(10, 4, (int)((v.skinTemp - 30.0f) * 100.0f / 12.0f));
  oled.setTextSize(3); oled.setCursor(36, 8); oled.printf("%.1f", v.skinTemp);
  oled.setTextSize(2); oled.print("C");
  oled.setTextSize(1); oled.setCursor(36, 36); oled.print("Skin / contact temp");
  oled.setCursor(36, 46); oled.print(v.skinTemp > TEMP_MAX ? "HIGH" : "Normal range");
}

void pageStress() {
  drawBolt(8, 6);
  if (!gsrContact) { oled.setCursor(32, 14); oled.print("Touch GSR pads"); return; }
  if (!gsrReady)   { oled.setCursor(32, 10); oled.print("Calibrating");
                     oled.setCursor(32, 24); oled.printf("%d s - stay calm", gsrSecondsLeft()); return; }
  oled.setTextSize(3); oled.setCursor(36, 6); oled.printf("%.0f", v.stress);
  oled.setTextSize(1); oled.setCursor(36, 32);
  oled.print(v.stress < 30 ? "Calm" : v.stress < 60 ? "Mild stress" : "High stress");
  drawBar(0, 42, 128, 9, v.stress);
}

void pageAct() {
  drawSteps(4, 4);
  oled.setTextSize(2); oled.setCursor(26, 4); oled.printf("%lu", (unsigned long)v.steps);
  oled.setTextSize(1); oled.setCursor(26, 22); oled.print("steps");
  oled.setCursor(0, 32); oled.printf("%.1f kcal  %.1f floors", v.kcal, v.floors);
  oled.setCursor(0, 42); oled.printf("%s  Alt %.1fm", ACT_NAMES[v.activity], v.relAlt);
}

void pageScores() {
  oled.setCursor(0, 0);  oled.printf("Fitness %.0f/100", v.fitness);
  drawBar(0, 10, 128, 8, v.fitness);
  oled.setCursor(0, 22); oled.printf("Health risk %.0f %s", v.risk, riskLabel());
  drawBar(0, 32, 128, 8, v.risk);
  oled.setCursor(0, 44); oled.printf("Sleep %.0f%%  RestHR %.0f", v.sleepQuality, restHr);
}

void pageSleep() {
  oled.fillCircle(20, 30, 12, SSD1306_WHITE);
  oled.fillCircle(26, 26, 11, SSD1306_BLACK);
  oled.setCursor(44, 14); oled.print("SLEEP MODE");
  oled.setCursor(44, 28); oled.printf("Quality %.0f%%", v.sleepQuality);
  oled.setCursor(44, 42); oled.print("OK = wake");
}

const char* okHint() {
  switch (page) {
    case P_HOME:   case P_SCORES: return "sleep";
    case P_ECG:    return ecgFrozen ? "resume" : "freeze";
    case P_ACT:    return "reset";
    case P_STRESS: return "recalibrate";
    default:       return "";
  }
}

void drawScreen() {
  oled.clearDisplay(); oled.setTextColor(SSD1306_WHITE); oled.setTextSize(1); oled.setCursor(0, 0);
  if (sleepMode) { pageSleep(); oled.display(); return; }
  switch (page) {
    case P_HOME:   pageHome();   break;
    case P_HEART:  pageHeart();  break;
    case P_SPO2:   pageSpo2();   break;
    case P_ECG:    pageEcg();    break;
    case P_RESP:   pageResp();   break;
    case P_TEMP:   pageTemp();   break;
    case P_STRESS: pageStress(); break;
    case P_ACT:    pageAct();    break;
    case P_SCORES: pageScores(); break;
    default: break;
  }
  oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
  if (!v.alerts && page != P_HOME) {
    oled.setCursor(104, 56); oled.printf("%d/%d", page + 1, P_COUNT);
    if (okHint()[0]) { oled.setCursor(0, 56); oled.printf("OK:%s", okHint()); }
  }
  if (v.alerts) {
    oled.fillRect(0, 54, 128, 10, SSD1306_WHITE);
    oled.setTextColor(SSD1306_BLACK); oled.setCursor(2, 55); oled.print(alertText());
  }
  oled.display();
}

void splash() {
  for (int i = 0; i < 24; i++) {
    oled.clearDisplay(); oled.setTextColor(SSD1306_WHITE); oled.setTextSize(1);
    int k = i % 8;
    int r = 10 + (k < 4 ? k : 8 - k) * 2;
    drawHeart(64, 24, r);
    oled.setCursor(34, 54); oled.print("HEALTH WATCH");
    oled.display(); delay(90);
  }
}

// ===================== 19. LOGGING =====================
void logCSV() {
  Serial.printf("%lu,%.0f,%.1f,%.0f,%.2f,%.0f,%d,%lu,%.1f,%.0f\n",
                millis(), bestHR(), v.spo2, v.resp, v.skinTemp, v.stress,
                (int)v.activity, (unsigned long)v.steps, v.kcal, v.risk);
}

// ===================== 20. SETUP / LOOP =====================
void setup() {
  Serial.begin(115200);
  uint8_t bm = BTN_ACTIVE_HIGH ? INPUT_PULLDOWN : INPUT_PULLUP;
  pinMode(BTN_NEXT, bm); pinMode(BTN_OK, bm); pinMode(BTN_BACK, bm);
  pinMode(PIN_ECG_LO_P, INPUT);  pinMode(PIN_ECG_LO_N, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);   digitalWrite(PIN_BUZZER, LOW);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_ECG_OUT, ADC_11db);
  analogSetPinAttenuation(PIN_GSR, ADC_11db);

  SPI.begin(OLED_SCK, OLED_MISO, OLED_MOSI);
  initOLED();
  initSensors();
  splash();
  if (SERIAL_MODE == 0) Serial.println("ms,hr,spo2,resp,temp,stress,activity,steps,kcal,risk");

  notchF.setNotch(250.0f, MAINS_HZ, 25.0f);
  lowF.setLowpass(250.0f, 35.0f, 0.707f);
  xTaskCreatePinnedToCore(ecgTask, "ecg", 4096, NULL, 2, NULL, 0);
}

void loop() {
  static uint32_t tMotion = 0, tGsr = 0, tSlow = 0, tDisp = 0;

  readHeartSpO2();
  if (due(tMotion, 40))  readMotion();
  if (due(tGsr, 100))    readGSR();
  if (due(tSlow, 1000)) {
    readTemperature();
    readAltitude();
    crossCheckHR();
    fuseStress();
    updateCalories();
    updateFatigue();
    if (sleepMode) updateSleep();
    updateScores();
    checkAlerts();
    if (SERIAL_MODE == 0) logCSV();
  }
  handleButtons();
  if (due(tDisp, 100)) drawScreen();
}
