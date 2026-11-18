/*
 * ============================================================================
 *  MAX30102  ->  ESP32-C3 SuperMini      (Arduino IDE, no external libraries)
 * ----------------------------------------------------------------------------
 *  Wiring:
 *      MAX30102 VIN  ->  3V3   (never 5 V)
 *      MAX30102 GND  ->  GND
 *      MAX30102 SDA  ->  GPIO4
 *      MAX30102 SCL  ->  GPIO5
 *      MAX30102 INT  ->  not used
 *
 *  Serial Monitor: 115200 baud
 *  (Tools > USB CDC On Boot must be "Enabled" on the SuperMini)
 *
 *  BLE: the same measurements are broadcast to index.html (Web Bluetooth)
 *  and the dashboard can start/stop recording, download /rec.csv and set
 *  the device clock.  See README.md for the browser setup.
 *
 *  Every second the sketch prints
 *      BPM        heart rate           (from the IR channel)
 *      SpO2       blood oxygen         (red/IR ratio-of-ratios)
 *      PI         perfusion index      (AC/DC of the IR, signal strength)
 *      IR/RED AC  live filtered amplitude, useful to judge the signal
 *      IR/RED DC  raw levels, AC peak amplitudes, LED current, sample rate
 *
 *  The I2C register driver, the LED-current auto-gain, the beat detector and
 *  the SpO2 maths are all in this file, so nothing has to be installed
 *  through Library Manager.
 * ============================================================================
 */

#include <Wire.h>
#include <math.h>
#include <LittleFS.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <time.h>

// The ESP32-C3 SuperMini only exposes USB, not the UART pins, so the IDE
// option "USB CDC On Boot" must be set to "Enabled" to see the Serial Monitor.
#if defined(ARDUINO_ARCH_ESP32) && defined(ARDUINO_USB_CDC_ON_BOOT) && !ARDUINO_USB_CDC_ON_BOOT
#warning "Set Tools > USB CDC On Boot = Enabled, otherwise the Serial Monitor stays empty."
#endif

/* ------------------------------- settings -------------------------------- */
static const int      PIN_SDA       = 4;
static const int      PIN_SCL       = 5;
static const uint32_t I2C_CLOCK_HZ  = 400000;
static const uint32_t SERIAL_BAUD   = 115200;

static const uint8_t  MAX30102_ADDR = 0x57;

// LED drive current is adjusted automatically between these limits.
// Register value * 0.2 mA  (0x0A = 2.0 mA, 0x24 = 7.2 mA, 0x3F = 12.6 mA)
static const uint8_t  LED_PA_MIN    = 0x0A;
static const uint8_t  LED_PA_MAX    = 0x3F;
static const uint8_t  LED_PA_START  = 0x24;

/* ------------------------------- registers ------------------------------- */
enum : uint8_t {
  REG_INTR_ENABLE_1 = 0x02,
  REG_INTR_ENABLE_2 = 0x03,
  REG_FIFO_WR_PTR   = 0x04,
  REG_OVF_COUNTER   = 0x05,
  REG_FIFO_RD_PTR   = 0x06,
  REG_FIFO_DATA     = 0x07,
  REG_FIFO_CONFIG   = 0x08,
  REG_MODE_CONFIG   = 0x09,
  REG_SPO2_CONFIG   = 0x0A,
  REG_LED1_PA       = 0x0C,   // RED
  REG_LED2_PA       = 0x0D,   // IR
  REG_PART_ID       = 0xFF
};

/* ----------------------------- tuning knobs ------------------------------ */
static const uint32_t FINGER_ON_IR   = 25000;  // raw IR above this = finger present
static const uint32_t SATURATION_IR  = 250000; // 18-bit full scale is 262143
static const uint32_t SPO2_WINDOW_MS = 4000;   // averaging window for SpO2
static const uint32_t WARMUP_MS      = 3000;
static const uint16_t MIN_BEAT_MS    = 280;    // ~214 bpm
static const uint16_t MAX_BEAT_MS    = 2200;   // 27 bpm
static const uint8_t  FIFO_SAMPLES   = 32;
static const int      SPO2_BUF       = 2048;   // up to ~20 s of samples

// Auto-gain: keep the IR DC level in a comfortable range for the 18-bit ADC.
static const float    GAIN_TARGET    = 90000.0f;
static const float    GAIN_LOW       = 45000.0f;
static const float    GAIN_HIGH      = 170000.0f;

/* ------------------------------- BLE setup ------------------------------- */
// Custom GATT service used by index.html (Web Bluetooth).
static const char* BLE_NAME     = "MAX30102";
static const char* SERVICE_UUID = "7a1f0001-3c2d-4e5f-8a9b-0c1d2e3f4a5b";
static const char* LIVE_UUID    = "7a1f0002-3c2d-4e5f-8a9b-0c1d2e3f4a5b";
static const char* CTRL_UUID    = "7a1f0003-3c2d-4e5f-8a9b-0c1d2e3f4a5b";
static const char* STATUS_UUID  = "7a1f0004-3c2d-4e5f-8a9b-0c1d2e3f4a5b";
static const char* DATA_UUID    = "7a1f0005-3c2d-4e5f-8a9b-0c1d2e3f4a5b";

static const char* REC_PATH     = "/rec.csv";
static const char* CSV_HEADER   = "datetime_gmt+0530,epoch,bpm,spo2,pi\n";
static const uint32_t IST_OFFSET_S = 5 * 3600 + 30 * 60;   // GMT+5:30

/* ------------------------------- I2C layer ------------------------------- */
static bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MAX30102_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(MAX30102_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  if (Wire.requestFrom((uint8_t)MAX30102_ADDR, (uint8_t)1) != 1) return 0xFF;
  return Wire.read();
}

static void i2cScan() {
  Serial.println("I2C scan:");
  uint8_t found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("  device at 0x");
      if (a < 16) Serial.print('0');
      Serial.println(a, HEX);
      found++;
    }
  }
  if (!found) Serial.println("  nothing found - check SDA/SCL/3V3/GND");
}

static uint8_t ledPa = LED_PA_START;

static bool max30102Init() {
  if (readReg(REG_PART_ID) != 0x15) {
    Serial.print("PART_ID = 0x");
    Serial.println(readReg(REG_PART_ID), HEX);
    return false;
  }

  writeReg(REG_MODE_CONFIG, 0x40);            // soft reset
  delay(50);
  while (readReg(REG_MODE_CONFIG) & 0x40) delay(10);

  writeReg(REG_INTR_ENABLE_1, 0x00);          // polled FIFO, no interrupts
  writeReg(REG_INTR_ENABLE_2, 0x00);
  writeReg(REG_FIFO_WR_PTR,   0x00);
  writeReg(REG_OVF_COUNTER,   0x00);
  writeReg(REG_FIFO_RD_PTR,   0x00);

  writeReg(REG_FIFO_CONFIG, 0x5F);            // average 4, rollover on, almost-full 15
  writeReg(REG_MODE_CONFIG, 0x03);            // SpO2 mode: RED + IR
  writeReg(REG_SPO2_CONFIG, 0x2F);            // 4096 nA range, 400 sps, 411 us (18 bit)
  writeReg(REG_LED1_PA,     ledPa);
  writeReg(REG_LED2_PA,     ledPa);

  return true;
}

// Reads every sample waiting in the FIFO. Returns the number of samples.
// Each sample is 6 bytes: RED(3) + IR(3), 18 significant bits each.
static int readFIFO(uint32_t *red, uint32_t *ir, int maxSamples) {
  uint8_t wr = readReg(REG_FIFO_WR_PTR) & 0x1F;
  uint8_t rd = readReg(REG_FIFO_RD_PTR) & 0x1F;
  int n = (wr - rd) & 0x1F;
  if (n == 0) return 0;
  if (n > maxSamples) n = maxSamples;

  int got = 0;
  while (got < n) {
    int chunk = n - got;
    if (chunk > 16) chunk = 16;               // 16 * 6 = 96 bytes (I2C buffer is 128)

    Wire.beginTransmission(MAX30102_ADDR);
    Wire.write(REG_FIFO_DATA);
    if (Wire.endTransmission(false) != 0) break;

    int want = chunk * 6;
    if ((int)Wire.requestFrom((uint8_t)MAX30102_ADDR, (uint8_t)want) != want) break;

    for (int i = 0; i < chunk; i++) {
      uint8_t b[6];
      for (int k = 0; k < 6; k++) b[k] = Wire.read();
      red[got + i] = (((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2]) & 0x3FFFF;
      ir[got + i]  = (((uint32_t)b[3] << 16) | ((uint32_t)b[4] << 8) | b[5]) & 0x3FFFF;
    }
    got += chunk;
  }
  return got;
}

/* --------------------------- signal processing --------------------------- */
static uint32_t bufRed[FIFO_SAMPLES];
static uint32_t bufIR[FIFO_SAMPLES];

static bool     fingerPresent = false;
static uint8_t  pressCnt      = 0;
static uint8_t  releaseCnt    = 0;
static uint32_t warmupUntil   = 0;
static bool     warmupPending = false;
static uint32_t settleUntil   = 0;
static bool     satWindow     = false;   // ADC clipped inside current SpO2 window
static bool     satSeen       = false;   // clipped since the last status line
static uint8_t  gainChanges   = 0;

// filters / beat detector
static bool     firstSample = true;
static float    dcIR        = 0.0f;
static float    dcRed       = 0.0f;
static float    lpIR        = 0.0f, lpRed = 0.0f;     // ~8 Hz low-pass
static float    h1IR        = 0.0f, h1Red = 0.0f;     // 0.6 Hz high-pass, stage 1
static float    h2IR        = 0.0f, h2Red = 0.0f;     // 0.6 Hz high-pass, stage 2
static float    h3IR        = 0.0f, h3Red = 0.0f;     // 0.6 Hz high-pass, stage 3
static float    h4IR        = 0.0f, h4Red = 0.0f;     // 0.6 Hz high-pass, stage 4
static float    bpIR        = 0.0f, bpRed = 0.0f;     // band-passed AC
static float    envIR       = 0.0f, envRed = 0.0f;    // decaying peak envelopes
static bool     inPeak      = false;
static float    peakVal     = 0.0f;
static uint32_t peakMs      = 0;
static float    beatEnv     = 0.0f;   // envelope of accepted beats (4 s decay)

static uint32_t lastBeatMs    = 0;
static uint16_t beatIntervals[5];
static uint8_t  beatCount     = 0;
static uint8_t  beatIdx       = 0;
static float    bpmDisplay    = 0.0f;
static uint8_t  beatsInWin    = 0;

// SpO2 window accumulators
static double   sumR = 0, sumI = 0;
static uint32_t winCount   = 0;
static uint32_t winStartMs = 0;
static float    spo2Value  = 0.0f;
static float    perfusion  = 0.0f;
static bool     spo2Valid  = false;

// band-passed samples of the current window, used for the frequency-domain
// ratio of ratios (immune to breathing/motion baseline wander)
static float    bpBufRed[SPO2_BUF];
static float    bpBufIR[SPO2_BUF];
static int      bpBufN = 0;

// last three SpO2 results -> median, so one noisy window cannot swing the display
static float    spo2Hist[3];
static bool     spo2Have[3];
static uint8_t  spo2Idx    = 0;
static uint8_t  spo2Missed = 0;

static uint32_t lastFifoMs  = 0;
static uint32_t lastGainMs  = 0;
static float    sampleDt    = 0.01f;
static uint32_t lastRawIR   = 0;
static uint32_t lastRawRed  = 0;

static void clearAccumulators(uint32_t now) {
  sumR = sumI = 0;
  winCount = 0;
  beatsInWin = 0;
  winStartMs = now;
  satWindow = false;
  bpBufN = 0;
}

static void resetAll() {
  firstSample = true;
  inPeak = false;
  peakVal = 0;
  peakMs = 0;
  lpIR = lpRed = 0;
  h1IR = h1Red = 0;
  h2IR = h2Red = 0;
  h3IR = h3Red = 0;
  h4IR = h4Red = 0;
  bpIR = bpRed = 0;
  envIR = envRed = 0;
  beatEnv = 0;
  beatCount = 0;
  beatIdx = 0;
  bpmDisplay = 0;
  lastBeatMs = 0;
  spo2Value = 0;
  perfusion = 0;
  spo2Valid = false;
  for (uint8_t i = 0; i < 3; i++) spo2Have[i] = false;
  spo2Idx = 0;
  spo2Missed = 0;
  satSeen = false;
  warmupPending = false;
  clearAccumulators(millis());
}

// Called after a LED-current change: the DC level steps, so restart the chain.
static void restartSignal(uint32_t now) {
  firstSample = true;
  inPeak = false;
  peakVal = 0;
  peakMs = 0;
  lpIR = lpRed = 0;
  h1IR = h1Red = 0;
  h2IR = h2Red = 0;
  h3IR = h3Red = 0;
  h4IR = h4Red = 0;
  bpIR = bpRed = 0;
  envIR = envRed = 0;
  beatEnv = 0;
  beatCount = 0;
  beatIdx = 0;
  bpmDisplay = 0;
  lastBeatMs = 0;
  clearAccumulators(now);
  settleUntil = now + 400;
}

static void addBeatInterval(uint16_t intervalMs) {
  beatIntervals[beatIdx] = intervalMs;
  beatIdx = (beatIdx + 1) % 5;
  if (beatCount < 5) beatCount++;

  // median of the collected intervals = robust against one missed/extra beat
  uint16_t t[5];
  for (uint8_t i = 0; i < beatCount; i++) t[i] = beatIntervals[i];
  for (uint8_t i = 1; i < beatCount; i++) {
    uint16_t key = t[i];
    int8_t j = (int8_t)i - 1;
    while (j >= 0 && t[j] > key) { t[j + 1] = t[j]; j--; }
    t[j + 1] = key;
  }
  float bpm = 60000.0f / (float)t[beatCount / 2];

  if (bpmDisplay < 1.0f) bpmDisplay = bpm;
  else                  bpmDisplay += 0.35f * (bpm - bpmDisplay);
}

static void pushSpo2(float spo2, float pi) {
  spo2Hist[spo2Idx] = spo2;
  spo2Have[spo2Idx] = true;
  spo2Idx = (spo2Idx + 1) % 3;

  float a[3];
  uint8_t n = 0;
  for (uint8_t i = 0; i < 3; i++) if (spo2Have[i]) a[n++] = spo2Hist[i];
  for (uint8_t i = 1; i < n; i++) {
    float key = a[i];
    int8_t j = (int8_t)i - 1;
    while (j >= 0 && a[j] > key) { a[j + 1] = a[j]; j--; }
    a[j + 1] = key;
  }
  spo2Value = a[n / 2];
  perfusion = pi;
  spo2Valid = true;
  spo2Missed = 0;
}

// Magnitude of one frequency component, Hann-windowed. Only the ratio
// between the two channels is used, so the absolute scale does not matter.
static float goertzelMag(const float *x, int n, float f0, float fs) {
  if (n < 32 || f0 <= 0.0f || fs <= 0.0f) return 0.0f;
  float w = 6.2831853f * f0 / fs;
  float cw = cosf(w);
  float sw = sinf(w);
  float coeff = 2.0f * cw;
  float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f;

  for (int i = 0; i < n; i++) {
    float win = 0.5f - 0.5f * cosf(6.2831853f * (float)i / (float)(n - 1));
    s0 = win * x[i] + coeff * s1 - s2;
    s2 = s1;
    s1 = s0;
  }
  float re = s1 - s2 * cw;
  float im = s2 * sw;
  return sqrtf(re * re + im * im);
}

static void computeSpO2(uint32_t now) {
  float fs = (sampleDt > 0.0f) ? (1.0f / sampleDt) : 100.0f;
  float f0 = bpmDisplay / 60.0f;

  bool enough = (beatsInWin >= 2) && (bpBufN >= 64) && !satWindow &&
                (f0 > 0.35f) && (f0 < 3.6f);

  if (enough) {
    double n  = (double)bpBufN;
    double mR = sumR / n;
    double mI = sumI / n;

    if (mR > 1000.0 && mI > 1000.0) {
      float aR = goertzelMag(bpBufRed, bpBufN, f0, fs);
      float aI = goertzelMag(bpBufIR,  bpBufN, f0, fs);

      if (aI > 0.0f) {
        // R = (ACred/DCred) / (ACir/DCir) at the heart-rate frequency
        double R = ((double)aR / mR) / ((double)aI / mI);
        double spo2 = -45.060 * R * R + 30.354 * R + 94.845;

        // PI from the same single-frequency amplitude: broadband noise and
        // baseline wander no longer inflate it.
        double ampI = 4.0 * aI / n;               // Hann-windowed sine amplitude
        float  pi   = (float)(100.0 * (ampI / 1.41421356) / mI);

        if (R > 0.2 && R < 1.6 && spo2 >= 70.0 && spo2 <= 100.0 && pi > 0.1f) {
          pushSpo2((float)spo2, pi);
          clearAccumulators(now);
          return;
        }
      }
    }
  }

  if (++spo2Missed >= 2) {
    spo2Valid = false;
    for (uint8_t i = 0; i < 3; i++) spo2Have[i] = false;
  }
  clearAccumulators(now);
}

// Adaptive detection threshold: 35 % of the amplitude of recent accepted
// beats, never below the noise floor. Wander/noise does not raise it.
static float beatThreshold() {
  float thr = 0.35f * beatEnv;
  return (thr < 60.0f) ? 60.0f : thr;
}

// A beat is accepted only when the peak is followed by a real trough within
// 60...700 ms - that rejects both spikes and slow baseline drift.
static void finishPeak(uint32_t now) {
  if (!inPeak) return;
  inPeak = false;

  float thr = beatThreshold();

  uint32_t since = peakMs - lastBeatMs;
  if (peakVal >= thr && since >= MIN_BEAT_MS) {
    if (lastBeatMs != 0 && since <= MAX_BEAT_MS) addBeatInterval((uint16_t)since);
    lastBeatMs = peakMs;
    if (beatsInWin < 250) beatsInWin++;
    if (peakVal > beatEnv) beatEnv = peakVal;
  }
  peakVal = 0;
}

static void processSample(uint32_t red, uint32_t ir, float dt, uint32_t now) {
  lastRawIR  = ir;
  lastRawRed = red;

  /* --- finger presence, with hysteresis + debounce ----------------------- */
  if (ir > FINGER_ON_IR) {
    if (pressCnt < 40) pressCnt++;
    releaseCnt = 0;
  } else {
    if (releaseCnt < 80) releaseCnt++;
    pressCnt = 0;
  }

  if (!fingerPresent && pressCnt >= 15) {
    fingerPresent = true;
    resetAll();
    dcIR = (float)ir;
    dcRed = (float)red;
    firstSample = false;
    warmupUntil = now + WARMUP_MS;
    warmupPending = true;
    settleUntil = now + 400;
    gainChanges = 0;
    winStartMs = now;
  } else if (fingerPresent && releaseCnt >= 60) {
    fingerPresent = false;
    resetAll();
  }

  if (!fingerPresent) return;
  if ((int32_t)(now - settleUntil) < 0) return;   // ignore the gain-change step

  if (ir >= SATURATION_IR || red >= SATURATION_IR) {
    satWindow = true;
    satSeen   = true;
  }

  if (firstSample) {
    dcIR = (float)ir;
    dcRed = (float)red;
    firstSample = false;
  }

  /* --- DC tracking (tau ~ 1 s) ------------------------------------------- */
  float a = dt;                 // alpha = dt / tau
  if (a > 0.5f) a = 0.5f;
  dcIR  += a * ((float)ir  - dcIR);
  dcRed += a * ((float)red - dcRed);

  float acIR  = (float)ir  - dcIR;
  float acRed = (float)red - dcRed;

  /* --- 0.6 ... 8 Hz band-pass -------------------------------------------- */
  float aLP = 1.0f - expf(-6.2831853f * 8.0f * dt);
  float aHP = 1.0f - expf(-6.2831853f * 0.6f * dt);

  lpIR += aLP * (acIR - lpIR);
  float y1 = lpIR - h1IR;
  h1IR += aHP * y1;
  float y2 = y1 - h2IR;
  h2IR += aHP * y2;
  float y3 = y2 - h3IR;
  h3IR += aHP * y3;
  float y4 = y3 - h4IR;
  h4IR += aHP * y4;
  bpIR = y4;

  lpRed += aLP * (acRed - lpRed);
  y1 = lpRed - h1Red;
  h1Red += aHP * y1;
  y2 = y1 - h2Red;
  h2Red += aHP * y2;
  y3 = y2 - h3Red;
  h3Red += aHP * y3;
  y4 = y3 - h4Red;
  h4Red += aHP * y4;
  bpRed = y4;

  /* --- decaying peak envelopes (~1.5 s) ---------------------------------- */
  float decay = expf(-dt / 1.5f);
  float mIR  = fabsf(bpIR);
  float mRed = fabsf(bpRed);
  envIR  = (mIR  > envIR)  ? mIR  : envIR  * decay;
  envRed = (mRed > envRed) ? mRed : envRed * decay;

  /* --- beat detection ----------------------------------------------------- */
  beatEnv *= expf(-dt / 4.0f);
  float thr = beatThreshold();

  if (bpIR > thr) {
    if (!inPeak) {
      inPeak  = true;
      peakVal = bpIR;
      peakMs  = now;
    } else if (bpIR > peakVal) {
      peakVal = bpIR;
      peakMs  = now;
    }
  } else if (inPeak) {
    if (bpIR < -0.20f * peakVal) {
      uint32_t t = now - peakMs;
      if (t >= 60 && t <= 700) finishPeak(now);
      inPeak  = false;
      peakVal = 0;
    } else if ((uint32_t)(now - peakMs) > 1500) {
      inPeak  = false;            // too slow to be a pulse
      peakVal = 0;
    }
  }

  /* --- SpO2 window -------------------------------------------------------- */
  sumR  += (double)red;
  sumI  += (double)ir;
  winCount++;

  if (bpBufN < SPO2_BUF) {
    bpBufRed[bpBufN] = bpRed;
    bpBufIR[bpBufN]  = bpIR;
    bpBufN++;
  }

  if (((uint32_t)(now - winStartMs) >= SPO2_WINDOW_MS && winCount >= 60) ||
      bpBufN >= SPO2_BUF) {
    computeSpO2(now);
  }
}

/* ------------------------------- auto gain ------------------------------- */
static void maybeAdjustGain(uint32_t now) {
  if (!fingerPresent || firstSample || gainChanges >= 10) return;
  if ((uint32_t)(now - settleUntil) < 300) return;

  uint8_t pa = ledPa;

  if (satWindow || dcIR > GAIN_HIGH) {
    float d = (dcIR < 500.0f) ? 500.0f : dcIR;
    int np = (int)((float)pa * (GAIN_TARGET / d));
    if (np > (int)pa - 2) np = (int)pa - 2;
    if (np < LED_PA_MIN)  np = LED_PA_MIN;
    pa = (uint8_t)np;
  } else if (dcIR < GAIN_LOW) {
    float d = (dcIR < 5000.0f) ? 5000.0f : dcIR;
    int np = (int)((float)pa * (GAIN_TARGET / d));
    if (np < (int)pa + 2) np = (int)pa + 2;
    if (np > LED_PA_MAX)  np = LED_PA_MAX;
    pa = (uint8_t)np;
  }

  if (pa != ledPa) {
    ledPa = pa;
    writeReg(REG_LED1_PA, ledPa);
    writeReg(REG_LED2_PA, ledPa);
    restartSignal(now);
    gainChanges++;
    lastGainMs = now;
  }
}

/* --------------------------- BLE + recording ----------------------------- */
static bool     fsReady       = false;
static bool     recording     = false;
static File     recFile;
static uint32_t recRows       = 0;
static uint32_t recBytes      = 0;
static char     recBuf[1024];
static size_t   recBufLen     = 0;
static uint32_t lastFlushMs   = 0;

// Wall clock, set by the dashboard ("TIME:<epoch seconds>") on connect.
static bool     clockSynced     = false;
static uint32_t clockBaseEpoch  = 0;
static uint32_t clockBaseMs     = 0;

static bool               bleConnected = false;
static uint16_t           bleMtu       = 23;
static BLEServer*         bleServer    = nullptr;
static BLECharacteristic* liveChar     = nullptr;
static BLECharacteristic* ctrlChar     = nullptr;
static BLECharacteristic* statusChar   = nullptr;
static BLECharacteristic* dataChar     = nullptr;

// Commands arrive in the BLE task; the main loop executes them.
static volatile bool     cmdStart   = false;
static volatile bool     cmdStop    = false;
static volatile bool     cmdClear   = false;
static volatile bool     cmdDump    = false;
static volatile bool     cmdTimeSet = false;
static volatile uint32_t cmdEpoch   = 0;

static String            lastCtrl;
static uint32_t          lastCtrlPollMs = 0;

// Latest 1 Hz measurement, mirrored into the status characteristic so the
// dashboard can show values even if it never receives a notification.
//   state: 0 = no finger, 1 = warm-up, 2 = pulse not found yet,
//          3 = pulse found but SpO2 pending, 4 = both valid
static uint8_t devState    = 0;
static uint8_t liveBpm     = 0;
static uint8_t liveSpo2    = 0;
static uint8_t livePiX10   = 0;
static uint8_t warmupLeftS = 0;

static uint32_t nowEpoch() {
  if (!clockSynced) return 0;
  return clockBaseEpoch + (millis() - clockBaseMs) / 1000;
}

static void formatIst(uint32_t epoch, char* out, size_t outLen) {
  if (epoch == 0) {
    snprintf(out, outLen, "0000-00-00 00:00:00");
    return;
  }
  time_t t = (time_t)epoch + (time_t)IST_OFFSET_S;
  struct tm tmv;
  gmtime_r(&t, &tmv);
  strftime(out, outLen, "%Y-%m-%d %H:%M:%S", &tmv);
}

static void flushRec() {
  if (recFile && recBufLen > 0) {
    recFile.write((const uint8_t*)recBuf, recBufLen);
    recFile.flush();
    recBufLen = 0;
  }
  lastFlushMs = millis();
}

static void appendCsvRow(float bpm, float spo2, float pi) {
  if (!recording || !recFile) return;

  uint32_t ep = nowEpoch();
  char dt[24];
  formatIst(ep, dt, sizeof(dt));

  int room = (int)sizeof(recBuf) - (int)recBufLen;
  if (room < 64) {
    flushRec();
    room = (int)sizeof(recBuf);
  }

  int len = snprintf(recBuf + recBufLen, room, "%s,%lu,%d,%d,%.2f\n",
                     dt, (unsigned long)ep,
                     (int)(bpm + 0.5f), (int)(spo2 + 0.5f), pi);
  if (len <= 0 || len >= room) return;   // should not happen

  recBufLen += len;
  recRows++;
  recBytes += len;
}

// Counts data rows (the header is not counted) so the dashboard shows the
// real total when a recording is continued after a stop.
static uint32_t countCsvRows() {
  File f = LittleFS.open(REC_PATH, "r");
  if (!f) return 0;
  uint32_t rows = 0;
  bool firstLine = true;
  int c;
  while ((c = f.read()) >= 0) {
    if (c == '\n') {
      if (firstLine) firstLine = false;
      else rows++;
    }
  }
  f.close();
  return rows;
}

static void startRecording() {
  if (!fsReady) {
    Serial.println("Record: LittleFS is not available");
    return;
  }
  if (recording) return;

  recFile = LittleFS.open(REC_PATH, FILE_APPEND);
  if (!recFile) {
    Serial.printf("Record: could not open %s\n", REC_PATH);
    return;
  }
  if (recFile.size() == 0) {
    recFile.print(CSV_HEADER);
    recFile.flush();
  }

  recRows    = countCsvRows();
  recBytes   = recFile.size();
  recBufLen  = 0;
  recording  = true;
  lastFlushMs = millis();
  Serial.printf("Record: started (%lu rows in file)\n", (unsigned long)recRows);
}

static void stopRecording() {
  if (!recording) return;
  recording = false;
  flushRec();
  if (recFile) recFile.close();
  Serial.printf("Record: stopped (%lu rows)\n", (unsigned long)recRows);
}

static void clearRecording() {
  stopRecording();
  recBufLen = 0;
  recRows  = 0;
  recBytes = 0;
  if (fsReady && LittleFS.exists(REC_PATH)) LittleFS.remove(REC_PATH);
  Serial.println("Record: data cleared");
}

static void bleNotifyBytes(BLECharacteristic* ch, const uint8_t* data, size_t len) {
  if (!bleConnected || ch == nullptr || len == 0) return;
  ch->setValue(data, len);
  ch->notify();
}

static void bleNotifyText(BLECharacteristic* ch, const char* text) {
  bleNotifyBytes(ch, (const uint8_t*)text, strlen(text));
}

// Streams /rec.csv to the dashboard: "B,<bytes>", then raw chunks, then "E".
static void dumpCsv() {
  if (!bleConnected) return;
  stopRecording();                    // flush and freeze the file

  if (bleServer != nullptr) {
    uint16_t m = bleServer->getPeerMTU(bleServer->getConnId());
    if (m >= 23) bleMtu = m;
  }

  File f = LittleFS.open(REC_PATH, "r");
  if (!f) {
    bleNotifyText(dataChar, "B,0");
    bleNotifyText(dataChar, "E");
    return;
  }

  uint32_t total = f.size();
  char hdr[24];
  snprintf(hdr, sizeof(hdr), "B,%lu", (unsigned long)total);
  bleNotifyText(dataChar, hdr);

  size_t chunk = (bleMtu > 26) ? (size_t)(bleMtu - 3) : 20;
  if (chunk > 240) chunk = 240;

  uint8_t buf[240];
  uint32_t sent = 0;
  while (f.available() && bleConnected) {
    int n = f.read(buf, chunk);
    if (n <= 0) break;
    bleNotifyBytes(dataChar, buf, (size_t)n);
    sent += n;
    delay(10);                        // let the BLE stack drain its queue
  }
  f.close();
  bleNotifyText(dataChar, "E");
  Serial.printf("CSV: sent %lu of %lu bytes\n",
                (unsigned long)sent, (unsigned long)total);
}

static void putU32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    bleConnected = true;
    // Skip whatever command string was left in the characteristic from the
    // previous session, so a stale "START" is not executed again.
    if (ctrlChar != nullptr) {
      lastCtrl = ctrlChar->getValue();
      lastCtrl.trim();
    }
    bleMtu = s->getPeerMTU(s->getConnId());
    if (bleMtu < 23) bleMtu = 23;
    Serial.printf("BLE: connected (MTU %u)\n", bleMtu);
  }

  void onDisconnect(BLEServer* s) override {
    bleConnected = false;
    Serial.println("BLE: disconnected");
    BLEDevice::startAdvertising();
  }
};

// Commands are polled instead of using the callback that needs Bluedroid
// internals; a new value is acted on once.
static void blePollControl(uint32_t now) {
  if (!bleConnected || ctrlChar == nullptr) return;
  if ((uint32_t)(now - lastCtrlPollMs) < 100) return;
  lastCtrlPollMs = now;

  String v = ctrlChar->getValue();
  v.trim();

  if (v.length() == 0 || v == lastCtrl) return;
  lastCtrl = v;

  if (v == "START") {
    cmdStart = true;
  } else if (v == "STOP") {
    cmdStop = true;
  } else if (v == "CLEAR") {
    cmdClear = true;
  } else if (v == "DUMP") {
    cmdDump = true;
  } else if (v.startsWith("TIME:")) {
    cmdEpoch   = strtoul(v.c_str() + 5, nullptr, 10);
    cmdTimeSet = true;
  }
}

// 20-byte binary status (version 3):
//   [0]    flags: 1=recording, 2=clock synced, 4=file system ok,
//                 8=finger present, 16=warm-up finished
//   [1..4] rows, [5..8] bytes, [9..12] epoch, [13] sample rate in Hz
//   [14] bpm, [15] spo2, [16] perfusion index x10,
//   [17] state (0..4), [18] warm-up seconds left, [19] version = 3
static void updateStatusValue() {
  if (statusChar == nullptr) return;
  uint8_t b[20];
  b[0] = (recording ? 1 : 0) | (clockSynced ? 2 : 0) | (fsReady ? 4 : 0) |
         (fingerPresent ? 8 : 0) |
         ((fingerPresent && (int32_t)(millis() - warmupUntil) >= 0) ? 16 : 0);
  putU32(b + 1, recRows);
  putU32(b + 5, recBytes);
  putU32(b + 9, nowEpoch());
  float fs = (sampleDt > 0.0f) ? (1.0f / sampleDt) : 0.0f;
  b[13] = (uint8_t)(fs + 0.5f);
  b[14] = liveBpm;
  b[15] = liveSpo2;
  b[16] = livePiX10;
  b[17] = devState;
  b[18] = warmupLeftS;
  b[19] = 3;
  statusChar->setValue(b, sizeof(b));
}

// Logs when the client enables notifications, so the Serial Monitor explains
// why the dashboard might be falling back to status polling.
class LiveCallbacks : public BLECharacteristicCallbacks {
  void onStatus(BLECharacteristic* c, Status s, uint32_t code) override {
    (void)c;
    (void)code;
    static int last = -1;
    if ((int)s == last) return;
    last = (int)s;
    if (s == SUCCESS_NOTIFY) {
      Serial.println("BLE: live notifications active");
    } else if (s == ERROR_NOTIFY_DISABLED) {
      Serial.println("BLE: live notify not subscribed (dashboard uses status polling)");
    } else if (s == ERROR_NO_CLIENT) {
      Serial.println("BLE: live notify skipped (no client)");
    } else if (s == ERROR_GATT) {
      Serial.println("BLE: live notify send failed");
    }
  }
};

static void bleBegin() {
  BLEDevice::init(BLE_NAME);
  BLEDevice::setMTU(247);

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());

  BLEService* svc = bleServer->createService(SERVICE_UUID);

  liveChar = svc->createCharacteristic(
    LIVE_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  liveChar->addDescriptor(new BLE2902());
  liveChar->setCallbacks(new LiveCallbacks());

  ctrlChar = svc->createCharacteristic(
    CTRL_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);

  statusChar = svc->createCharacteristic(STATUS_UUID, BLECharacteristic::PROPERTY_READ);
  updateStatusValue();

  dataChar = svc->createCharacteristic(DATA_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  dataChar->addDescriptor(new BLE2902());

  svc->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setName(BLE_NAME);
  adv->setScanResponse(true);
  adv->start();

  Serial.printf("BLE: advertising as \"%s\"\n", BLE_NAME);
}

static void bleProcessCommands(uint32_t now) {
  bool changed = false;
  if (cmdTimeSet) {
    cmdTimeSet = false;
    clockBaseEpoch = cmdEpoch;
    clockBaseMs    = millis();
    clockSynced    = true;
    changed        = true;
    Serial.printf("BLE: clock set to %lu\n", (unsigned long)cmdEpoch);
  }
  if (cmdStart) { cmdStart = false; startRecording(); changed = true; }
  if (cmdStop)  { cmdStop  = false; stopRecording();  changed = true; }
  if (cmdClear) { cmdClear = false; clearRecording(); changed = true; }
  if (cmdDump)  { cmdDump  = false; dumpCsv();        changed = true; }
  if (changed) updateStatusValue();
}

/* --------------------------------- setup --------------------------------- */
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);

  Serial.println();
  Serial.println("MAX30102 pulse oximeter - ESP32-C3 SuperMini");
  Serial.printf("I2C: SDA=GPIO%d  SCL=GPIO%d  %lu Hz\n",
                PIN_SDA, PIN_SCL, (unsigned long)I2C_CLOCK_HZ);

  Wire.begin(PIN_SDA, PIN_SCL, I2C_CLOCK_HZ);

  if (!max30102Init()) {
    Serial.println("MAX30102 not found at 0x57.");
    i2cScan();
    Serial.println("Fix the wiring and press the reset button.");
    while (true) delay(1000);
  }

  // LittleFS holds the CSV recording (/rec.csv).
  fsReady = LittleFS.begin(true);
  if (fsReady) {
    Serial.printf("LittleFS: %lu of %lu bytes free\n",
                  (unsigned long)(LittleFS.totalBytes() - LittleFS.usedBytes()),
                  (unsigned long)LittleFS.totalBytes());
  } else {
    Serial.println("LittleFS: mount failed - recording disabled");
  }

  bleBegin();

  Serial.println("Sensor OK. Put your fingertip gently on the sensor,");
  Serial.println("keep still, wait a few seconds...\n");
  Serial.println("  time  BPM  SpO2    PI     IR DC   RED DC   IR AC  RED AC   LED    rate");

  winStartMs = millis();
  lastFifoMs = 0;
  lastGainMs = millis();
}

/* ---------------------------------- loop --------------------------------- */
void loop() {
  uint32_t now = millis();

  int n = readFIFO(bufRed, bufIR, FIFO_SAMPLES);
  if (n > 0) {
    float dt;
    if (lastFifoMs == 0) {
      dt = 0.01f;
    } else {
      dt = (float)(now - lastFifoMs) / (1000.0f * (float)n);
      if (dt < 0.002f) dt = 0.002f;
      if (dt > 0.2f)   dt = 0.2f;
    }
    lastFifoMs = now;
    sampleDt = dt;

    for (int i = 0; i < n; i++) {
      processSample(bufRed[i], bufIR[i], dt, now);
    }
  } else {
    delay(2);
  }

  // start the SpO2 averaging window only after the warm-up transient
  if (warmupPending && fingerPresent && (int32_t)(now - warmupUntil) >= 0) {
    warmupPending = false;
    clearAccumulators(now);
  }

  // keep the LED current in the useful range (checks at most every 1.5 s)
  if ((uint32_t)(now - lastGainMs) >= 1500) {
    lastGainMs = now;
    maybeAdjustGain(now);
  }

  // execute commands from the dashboard (a CSV dump can take a few seconds)
  blePollControl(now);
  bleProcessCommands(now);
  now = millis();

  if (recording && (uint32_t)(now - lastFlushMs) >= 30000) flushRec();

  /* ------------------------------ reporting ------------------------------ */
  static uint32_t lastPrintMs = 0;
  if ((uint32_t)(now - lastPrintMs) < 1000) return;
  lastPrintMs = now;

  // forget a heart rate that has gone stale
  if (fingerPresent && beatCount > 0 && (now - lastBeatMs) > 5000) {
    beatCount = 0;
    beatIdx = 0;
    bpmDisplay = 0;
  }

  bool measuring = fingerPresent && (int32_t)(now - warmupUntil) >= 0;
  bool bpmOk     = measuring && bpmDisplay > 0.0f;
  bool spo2Ok    = measuring && spo2Valid;

  /* --- device state shown on the dashboard -------------------------------- */
  if (!fingerPresent)  devState = 0;   // finger off
  else if (!measuring) devState = 1;   // warming up
  else if (!bpmOk)     devState = 2;   // pulse not found yet
  else if (!spo2Ok)    devState = 3;   // pulse ok, SpO2 pending
  else                 devState = 4;   // both valid

  float bpmNow = bpmOk ? bpmDisplay : 0.0f;
  if (bpmNow < 0.0f)   bpmNow = 0.0f;
  if (bpmNow > 255.0f) bpmNow = 255.0f;
  liveBpm  = (uint8_t)(bpmNow + 0.5f);
  liveSpo2 = spo2Ok ? (uint8_t)(spo2Value + 0.5f)  : 0;

  float piNow = spo2Ok ? perfusion : 0.0f;
  if (piNow < 0.0f)  piNow = 0.0f;
  if (piNow > 25.0f) piNow = 25.0f;
  livePiX10 = (uint8_t)(piNow * 10.0f + 0.5f);

  if (fingerPresent && !measuring) {
    int32_t left = (int32_t)(warmupUntil - now);
    left = (left + 999) / 1000;
    if (left < 0)   left = 0;
    if (left > 255) left = 255;
    warmupLeftS = (uint8_t)left;
  } else {
    warmupLeftS = 0;
  }

  /* --- serial monitor ----------------------------------------------------- */
  if (!fingerPresent) {
    Serial.printf("%5lus   no finger (IR=%lu, RED=%lu)\n",
                  (unsigned long)(now / 1000),
                  (unsigned long)lastRawIR,
                  (unsigned long)lastRawRed);
  } else if (!measuring) {
    Serial.printf("%5lus  measuring, hold still ...\n", (unsigned long)(now / 1000));
  } else {
    char bpmTxt[8];
    char spo2Txt[9];
    if (bpmOk) snprintf(bpmTxt, sizeof(bpmTxt), "%3.0f", bpmDisplay);
    else       snprintf(bpmTxt, sizeof(bpmTxt), " --");
    if (spo2Ok) snprintf(spo2Txt, sizeof(spo2Txt), "%3.0f%%", spo2Value);
    else        snprintf(spo2Txt, sizeof(spo2Txt), "  --");

    Serial.printf("%5lus  %s  %s  %4.2f%%  %6lu  %6lu  %5.0f  %5.0f  0x%02X  %5.1f Hz%s\n",
                  (unsigned long)(now / 1000),
                  bpmTxt,
                  spo2Txt,
                  spo2Ok ? perfusion : 0.0f,
                  (unsigned long)dcIR,
                  (unsigned long)dcRed,
                  envIR,
                  envRed,
                  ledPa,
                  1.0f / sampleDt,
                  satSeen ? "  !SAT" : "");
  }
  satSeen = false;

  /* --- BLE live value: "<epoch>,<bpm>,<spo2>,<state>" ---------------------- */
  // Max 20 bytes so it still fits a 23-byte ATT MTU.
  char live[24];
  snprintf(live, sizeof(live), "%lu,%u,%u,%u",
           (unsigned long)nowEpoch(), liveBpm, liveSpo2, devState);
  bleNotifyText(liveChar, live);

  /* --- CSV recording: one row per second while a finger is present -------- */
  if (measuring) {
    appendCsvRow(bpmOk ? bpmDisplay : 0.0f,
                 spo2Ok ? spo2Value : 0.0f,
                 spo2Ok ? perfusion : 0.0f);
  }

  updateStatusValue();
}
