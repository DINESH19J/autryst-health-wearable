# ESP32 Wearable Health Watch

A multi-sensor wearable built on the **ESP32** that combines **ECG, PPG (heart rate, SpO2, respiration), motion, barometric pressure, skin conductance and temperature** to produce rule-based health insights with no machine learning: stress, fatigue, activity intensity, stair climbing, calorie burn, sleep quality, a fitness score and a health-risk index. Results appear on a watch-style OLED interface with live icons and a beating-heart animation.

> ⚠️ **Disclaimer:** Educational / research prototype. **Not a medical device** and not intended for diagnosis or treatment. Values are screening indicators only.

---

## Table of Contents
1. [Features](#1-features)
2. [Hardware](#2-hardware)
3. [Pin Connections](#3-pin-connections)
4. [Software Setup](#4-software-setup)
5. [User Interface and Controls](#5-user-interface-and-controls)
6. [How It Works](#6-how-it-works)
7. [Sensor-Fusion Applications](#7-sensor-fusion-applications)
8. [Accuracy and Calibration](#8-accuracy-and-calibration)
9. [Troubleshooting](#9-troubleshooting)
10. [Reducing Wires / Wireless Options](#10-reducing-wires--wireless-options)
11. [Roadmap](#11-roadmap)
12. [Repository Layout](#12-repository-layout)

---

## 1. Features
- **ECG** at 250 Hz with band-pass + 50/60 Hz notch filtering, R-peak detection, HR, HRV (RMSSD) and a signal-quality score
- **Heart rate and SpO2** from MAX30102, with ECG-vs-PPG cross-validation
- **Respiration rate** derived from the PPG baseline (no extra sensor)
- **Stress** from GSR skin-conductance responses, gated by motion and combined with HR
- **Activity intensity, steps, stair climbing (floors)** from MPU6050 + BMP280
- **Calorie estimate** (MET + heart-rate blend), **fatigue detection**, **fall detection**
- **Sleep quality estimate** (motion-based actigraphy)
- **Fitness score** and **health-risk index** (simple point rules)
- **Smart alerts** with buzzer: high HR, low SpO2, abnormal breathing, high temperature, high stress, fall
- **Watch-style OLED UI**: pulsing heart, "Measuring…" progress, icons, 9 pages
- **Serial Plotter modes** for live ECG and GSR debugging

## 2. Hardware

| Part | Purpose | Interface |
|---|---|---|
| ESP32 Dev Module | Main controller | |
| AD8232 ECG module + Ag/AgCl electrodes | ECG | Analog |
| MAX30102 | Heart rate, SpO2, respiration | I2C |
| MPU6050 (GY-521) | Motion, steps, falls | I2C |
| BMP280 | Pressure / relative altitude | I2C |
| TMP117 | Body (skin) temperature | I2C |
| GSR v2.0 (3-pin) | Skin conductance | Analog |
| 0.96" 6-pin OLED (SSD1306, **SPI**) | Display | SPI |
| 3 × 4-pin tactile buttons | NEXT / OK / BACK | Digital |
| Active buzzer (optional) | Alerts | Digital |
| 100 nF ceramic capacitor (marked 104) | ECG output filtering | |
| Li-ion battery + charger/protection | Portable power | |

## 3. Pin Connections

Power everything from **3V3** and a common **GND**. Pin labels vary between boards, so follow the printed labels.

### 3.1 I2C bus (4 sensors): SDA = GPIO21, SCL = GPIO22

| Sensor | Address | Connections |
|---|---|---|
| MAX30102 | 0x57 | VIN→3V3, GND→GND, SCL→22, SDA→21 (INT/IRD/RD open) |
| MPU6050 | 0x68 | VCC→3V3, GND→GND, SCL→22, SDA→21, **AD0→GND** |
| TMP117 | 0x48 | VIN→3V3, GND→GND, SCL→22, SDA→21 (ADD0→GND if present; ALERT/INT open) |
| BMP280 (6-pin) | 0x76 | VCC→3V3, GND→GND, SCL→22, SDA→21, **CSB→3V3**, **SDO→GND** |

If your TMP117 shows a different address in an I2C scanner (0x49–0x4B), use `tmp.begin(0x49)` etc. in `initSensors()`.

### 3.2 OLED 0.96" 6-pin (SPI)

| OLED label | ESP32 |
|---|---|
| GND | GND |
| VCC | 3V3 |
| D0 (SCK) | GPIO18 |
| D1 (MOSI) | GPIO23 |
| RES | GPIO16 |
| DC | GPIO17 |
| CS (if present) | GPIO4 (set `OLED_CS` to -1 if the module has no CS pin) |

### 3.3 Analog sensors

| Signal | ESP32 | Notes |
|---|---|---|
| AD8232 OUTPUT | GPIO34 (ADC1) | 100 nF capacitor between OUTPUT and GND |
| AD8232 LO+ / LO− | GPIO32 / GPIO33 | Leads-off detection |
| AD8232 3.3V / GND | 3V3 / GND | SDN open |
| GSR SIG | GPIO35 (ADC1) | Power GSR from **3V3** |
| GSR VCC / GND | 3V3 / GND | |

Only ADC1 pins are used so analog readings keep working when WiFi is enabled.

### 3.4 Buttons and buzzer
Each 4-pin tactile button: one leg to the GPIO, the **diagonal** leg to GND. No resistor is needed (internal pull-up).

| Button | GPIO |
|---|---|
| NEXT | 25 |
| OK | 26 |
| BACK | 27 |
| Buzzer + (active buzzer) | 13 (− to GND) |

### 3.5 GPIO summary

| GPIO | Use | GPIO | Use |
|---|---|---|---|
| 21 / 22 | I2C SDA / SCL | 34 | ECG output |
| 18 / 23 | OLED SCK / MOSI | 32 / 33 | ECG LO+ / LO− |
| 16 / 17 / 4 | OLED RES / DC / CS | 35 | GSR |
| 25 / 26 / 27 | NEXT / OK / BACK | 13 | Buzzer |

GPIO 5 and 19 are kept free for a future SD card on the shared SPI bus.

### 3.6 ECG electrode placement and wiring tips
- **RA** (red): right upper chest below the collarbone; **LA** (yellow): left upper chest; **RL** (green): lower right ribs.
- Use pre-gelled Ag/AgCl snap electrodes on clean, dry skin.
- **Run from a battery** while electrodes are attached. Avoid mains-connected supplies.
- Keep ECG leads short and away from the buzzer and WiFi antenna.
- The four I2C boards each carry pull-ups. If the bus is unstable, remove pull-ups from 1–2 boards.

## 4. Software Setup

1. Install the **esp32 by Espressif Systems** board package (Boards Manager).
2. Select **Tools → Board → ESP32 Arduino → ESP32 Dev Module** and your COM port.
3. Install libraries (Library Manager):
   - Adafruit SSD1306, Adafruit GFX Library
   - SparkFun MAX3010x Pulse and Proximity Sensor Library
   - Adafruit MPU6050, Adafruit BMP280 Library, Adafruit TMP117
   - Adafruit Unified Sensor, Adafruit BusIO (install dependencies when prompted)
4. Place `wearable_health_monitor_v3.ino` in a folder named exactly `wearable_health_monitor_v3`.
5. Click **Verify**, then **Upload** (hold BOOT if upload does not start).

### Configuration (top of the sketch)

| Setting | Meaning |
|---|---|
| `SERIAL_MODE` | 0 = CSV log, 1 = filtered ECG for Serial Plotter, 2 = GSR debug |
| `MAINS_HZ` | 50 (India/EU) or 60 (USA), for the ECG notch filter |
| `USER_WEIGHT_KG`, `USER_AGE` | Calorie estimate and maximum heart rate |
| `SPO2_A`, `SPO2_B` | SpO2 calibration (`SpO2 = A − B·R`) |
| `TEMP_OFFSET` | Temperature calibration |
| `ECG_MIN_AMP` | Minimum R-peak amplitude |
| `GSR_INVERTED` | Set `true` if the GSR reading drops when skin gets sweaty |
| `BTN_ACTIVE_HIGH` | `false` for tactile buttons wired to GND |

**Serial Plotter:** set `SERIAL_MODE = 1`, upload, then open **Tools → Serial Plotter** at **115200** baud to see the live filtered ECG waveform.

## 5. User Interface and Controls

| Button | Action |
|---|---|
| **NEXT** | Next page |
| **OK** | Context action (see below) |
| **BACK** | Mutes an active alert for 60 s (and clears fall alert); otherwise returns to Home |

| Page | Content | OK action |
|---|---|---|
| 1 Home | Pulsing heart, HR, SpO2, temperature, respiration | Sleep mode |
| 2 Heart | HR (PPG and ECG), "Measuring…" progress | none |
| 3 SpO2 | Drop icon, SpO2 value, signal quality | none |
| 4 ECG | Live waveform, BPM, quality % | Freeze / resume |
| 5 Respiration | Lungs icon, breaths/min, waveform | none |
| 6 Temperature | Thermometer icon and value | none |
| 7 Stress | Stress level, calibration countdown | Recalibrate GSR |
| 8 Activity | Steps, kcal, floors, activity level, altitude | Reset counters |
| 9 Scores | Fitness, risk index, sleep quality, resting HR | Sleep mode |

In sleep mode the display shows only the sleep quality, alerts are muted, and OK wakes the device.

## 6. How It Works

### System architecture
```
 AD8232 ──(analog)──┐
 GSR ────(analog)───┤        ┌── Core 0: ECG task @250 Hz (filter, R-peak, RR, HRV, quality)
                    ├─ ESP32 ┤
 MAX30102 ─┐        │        └── Core 1: loop(): PPG, motion, GSR, fusion, alerts, scores, UI
 MPU6050  ─┤ I2C ───┘                       │
 BMP280   ─┤                                └→ Serial (CSV / plotter), future BLE / WiFi / SD
 TMP117   ─┘
 OLED ──(SPI)── ESP32
```

### Measurement methods
- **ECG:** raw ADC (8× oversampled) → DC blocker → 50/60 Hz notch → 35 Hz low-pass → adaptive R-peak detector. HR is the median of recent RR intervals; HRV is RMSSD. A quality score (% of beats consistent with the median) hides unreliable values.
- **Heart rate / SpO2:** beat detection on the IR channel; SpO2 from the red/IR ratio-of-ratios (`110 − 25R` default, user-calibratable). SpO2 is flagged unreliable if PPG and ECG heart rates disagree by more than 12 bpm.
- **Respiration:** band-pass of the IR baseline (about 0.05–0.6 Hz); one breath per trough-to-peak cycle; rate is averaged over recent intervals. Disabled during moderate or vigorous movement.
- **Stress:** GSR is smoothed, a baseline (tonic level) is learned for 30 s, then skin-conductance responses (≥1 % rise over baseline) are counted per minute. Stress = 70 % response rate (8/min = 100) + 30 % heart-rate elevation over resting HR (only when PPG is reliable and the user is at rest). Electrodes off ⇒ automatic recalibration. Stress is paused during exercise.
- **Activity:** accelerometer magnitude RMS → Resting / Light / Moderate / Vigorous; steps by threshold crossing.
- **Stairs and floors:** BMP280 relative altitude rising faster than 0.12 m/s while moving ⇒ climbing (+3 MET), 3 m per floor.
- **Calories:** MET from activity (+ stairs), blended with the Keytel heart-rate formula above 100 bpm.
- **Fatigue:** efficiency = motion intensity ÷ (HR − resting HR); a sustained drop versus a 3-minute baseline ⇒ fatigue %.
- **Fall detection:** free-fall (< 3 m/s²) followed by impact (> 25 m/s²) within 0.6 s.
- **Sleep quality:** 30 s epochs; quiet epochs ÷ total epochs.

### Alerts (sustained before triggering)

| Alert | Condition |
|---|---|
| High HR | > 110 bpm at rest/light activity, or > (220 − age), for 10 s |
| Low SpO2 | < 92 % with a reliable signal, for 10 s |
| Abnormal breathing | < 8 or > 25 breaths/min at rest, for 20 s |
| High temperature | > 38.0 °C for 10 s |
| High stress | Stress > 75 for 10 s |
| Fall | Free-fall + impact |

### Fitness score and health-risk index
- **Fitness (0–100)** = steps/8000 × 40 + resting-HR score × 30 + (100 − fatigue) × 0.30
- **Risk (0–100)** adds points: resting HR > 100 / > 120 / < 50 (15 / 25 / 10), SpO2 < 95 / < 92 / < 90 (10 / 25 / 35), abnormal breathing (10), temperature > 37.5 / > 38.0 (10 / 20), stress > 60 / > 80 (10 / 15), HRV RMSSD < 20 ms (10), fatigue > 60 % (5). Level: **< 25 LOW**, **25–49 MODERATE**, **≥ 50 HIGH**.

## 7. Sensor-Fusion Applications

| Fusion | Application | Status |
|---|---|---|
| MPU6050 + BMP280 | Activity intensity, stair detection, accurate calories | ✅ Implemented |
| MPU6050 + HR + BMP280 | Fatigue detection | ✅ Implemented |
| GSR + HR + motion | Stress / anxiety detection that ignores exercise | ✅ Implemented |
| PPG-HR + ECG-HR | Motion-artifact rejection and signal confidence | ✅ Implemented |
| ECG + SpO2 + Resp | Early cardiovascular / respiratory risk flag | ✅ Implemented (risk index) |
| MPU6050 + HR + SpO2 | Fall detection with vitals | ✅ Fall detected; vitals check is an extension |
| Temp + resting HR + HRV | Illness early warning | 🔧 Extension (needs personal baseline) |
| Temp + motion + HR + GSR | Heat-stress warning | 🔧 Extension |
| ECG R-peak + PPG foot | Pulse transit time → blood-pressure trend | 🔧 Extension (needs calibration) |
| ECG RR intervals | Arrhythmia screening (RR irregularity) | 🔧 Extension |
| SpO2 + motion at night | Nocturnal desaturation screening | 🔧 Extension |
| BMP280 + SpO2 + HR | Altitude-sickness monitor | 🔧 Extension |
| Motion + HR | HR recovery (1 min) for cardio fitness | 🔧 Extension |

## 8. Accuracy and Calibration

This is a low-cost prototype. Expected performance against clinical devices:

| Measurement | Realistic accuracy |
|---|---|
| ECG heart rate (good electrode contact) | about 1–3 bpm |
| Fingertip PPG heart rate (at rest) | about 3–5 bpm |
| SpO2 (uncalibrated) | about ±3–4 %; unreliable with movement, cold fingers or poor contact |
| Respiration (PPG-derived, at rest) | about ±2–3 breaths/min |
| TMP117 sensor | ±0.1 °C, but skin temperature is **not** core temperature |
| Calories | about ±20–30 % |
| Steps | about ±10 % |
| Altitude | relative only; drifts with weather |
| GSR / stress / risk | relative, heuristic scores; not clinically validated |

**Calibrate and validate:**
1. Compare SpO2 and HR against a certified pulse oximeter at rest (10–20 readings) and adjust `SPO2_A` / `SPO2_B`.
2. Compare temperature against a clinical thermometer and set `TEMP_OFFSET`.
3. Compare ECG BPM with a chest-strap monitor.
4. Report mean absolute error per sensor in your project documentation.

## 9. Troubleshooting

| Problem | Fix |
|---|---|
| `Wire.begin(int,int)`, `ADC_11db`, `xTaskCreatePinnedToCore` errors | Wrong board selected. Choose **ESP32 Dev Module** |
| Sensor missing / "FAILED" on boot | Run an I2C scanner; expect 0x57, 0x68, 0x48, 0x76. Check SDA/SCL, AD0, BMP280 CSB |
| OLED blank | Check D0/D1/RES/DC wiring; try `OLED_CS` = −1 if no CS pin |
| ECG shows spikes only when touching or tapping leads | Poor electrode contact. Use gel electrodes, clean skin, battery power, short leads |
| ECG BPM shows `--` | Signal too noisy (quality < 60 %) or amplitude below `ECG_MIN_AMP` |
| Stress shows "Calibrating" or "Touch GSR pads" | Keep both fingers on the pads; baseline needs 30 s of stable contact |
| Stress seems inverted | Toggle `GSR_INVERTED` after checking behaviour in `SERIAL_MODE = 2` |
| Buttons do nothing / act pressed | Check `BTN_ACTIVE_HIGH`; use diagonal legs on 4-pin buttons |
| `'Btn' was not declared` | You are compiling an old sketch version. Use v3 |

## 10. Reducing Wires / Wireless Options
- **Watch-style ECG:** a stainless-steel pad on the back of the case (left wrist, LA), a pad on the side touched by a right finger (RA), and another back pad for RL. The signal is weaker but needs no leads.
- **GSR:** finger straps with 10 cm leads, or pads on the underside of the wrist.
- **Chest-patch node:** AD8232 + small ESP32 on the chest with 10–15 cm snap leads, streaming ECG to the main unit over **ESP-NOW** or BLE.
- Keep any remaining leads short and twisted/shielded, and away from other wiring.

## 11. Roadmap
- **BLE** (NimBLE) GATT service for a phone app; **WiFi + MQTT** to Node-RED / InfluxDB / Grafana
- **Local web dashboard** (ESP32 web server + WebSocket + Chart.js)
- **SD-card logging** (CS = GPIO5, shared SPI bus) with an RTC for timestamps
- Emergency alerts (Telegram / SMS webhook) on fall or critical vitals
- Personal baselines (resting HR, HRV, temperature) for illness early warning
- Arrhythmia screening, pulse transit time trend, nocturnal SpO2 dip counting
- Battery monitoring and deep-sleep power optimisation; 3D-printed enclosure

## 12. Repository Layout
```
/esp32-health-watch
 ├── README.md
 ├── firmware/wearable_health_monitor_v3/wearable_health_monitor_v3.ino
 ├── docs/            (wiring diagram, architecture, screenshots, demo photos)
 ├── hardware/        (BOM, enclosure files)
 ├── dashboard/       (Node-RED flow / Grafana JSON / web UI)
 └── data/            (sample CSV logs and validation results)
```



