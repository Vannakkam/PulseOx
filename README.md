# MAX30102 + ESP32-C3 SuperMini pulse oximeter

Arduino IDE firmware that reads a MAX30102 heart-rate / SpO2 sensor on an
ESP32-C3 SuperMini and

* prints **pulse rate (BPM)**, **SpO2**, the **perfusion index** and live
  signal levels to the Serial Monitor once per second, and
* broadcasts the same measurements over **BLE**, with a Web Bluetooth
  dashboard (`index.html`) that can pair, sync the clock, start/stop a CSV
  recording, download it, and refresh the live view.

No libraries to install — the I2C driver, LED auto-gain, beat detector, SpO2
calculation, BLE service and CSV recorder are all inside
`Max30102_SMC3.ino` (BLE and LittleFS are part of the ESP32 core).

## Files

| File | Purpose |
|------|---------|
| `Max30102_SMC3.ino` | Firmware: sensor, DSP, BLE server, LittleFS CSV recorder |
| `index.html` | Dashboard (open in Chrome/Edge, Web Bluetooth) |
| `README.md` | This document |

## Wiring

| MAX30102 module | ESP32-C3 SuperMini |
|-----------------|--------------------|
| VIN / VCC       | **3V3** (never 5 V) |
| GND             | GND                |
| SDA             | GPIO4              |
| SCL             | GPIO5              |
| INT             | not connected      |

The module already has I2C pull-ups, so no extra resistors are needed.
Keep the sensor out of direct sunlight and away from bright lamps.

## Arduino IDE setup

1. Open `Max30102_SMC3.ino` (the folder name matches the sketch name).
2. Boards Manager: install **esp32 by Espressif** (tested with 3.3.10).
3. Tools → Board → **ESP32C3 Dev Module**.
4. Tools → **USB CDC On Boot → Enabled**.
   Required on the SuperMini: its USB port is the only serial connection, and
   with CDC disabled the Serial Monitor stays empty.
5. Tools → Partition Scheme → **Default 4MB with spiffs**.
   The default gives 1.31 MB for the app (this sketch uses about 0.72 MB) and
   a 1.4 MB LittleFS area for recordings.
6. Tools → Port → the SuperMini's USB port, then **Upload**.
7. Open the Serial Monitor at **115200 baud**.

If uploading fails, hold **BOOT**, tap **RESET**, release **BOOT**, upload again.

The firmware starts advertising as **`MAX30102`** as soon as it boots. No
pairing in the operating system is needed — the dashboard connects directly.

## Serial output

```
MAX30102 pulse oximeter - ESP32-C3 SuperMini
I2C: SDA=GPIO4  SCL=GPIO5  400000 Hz
LittleFS: 1433600 of 1441792 bytes free
BLE: advertising as "MAX30102"
Sensor OK. Put your fingertip gently on the sensor,
keep still, wait a few seconds...

  time  BPM  SpO2    PI     IR DC   RED DC   IR AC  RED AC   LED    rate
     3s   no finger (IR=812, RED=655)
     6s  measuring, hold still ...
     9s   73    --  1.14%   92411    41380    1120     430  0x24  100.0 Hz
    14s   72    97%  1.21%   92402    41283    1154     442  0x24  100.0 Hz
```

* **BPM** – heart rate from the IR channel (median of the last 5 beat intervals).
* **SpO2** – ratio-of-ratios at the measured pulse frequency, median of the
  last 3 windows. `--` until a stable pulse exists and the signal is good.
* **PI** – perfusion index (`AC/DC` of IR, in %), measured at the pulse
  frequency. The best quick quality check: below ~0.2 % the SpO2 number is
  unreliable.
* **IR/RED DC** – raw 18-bit levels (0…262143).
* **IR/RED AC** – peak amplitudes of the filtered signal. If IR AC is below
  ~100 counts the pulse is too weak to detect: press a little firmer or clean
  the sensor window.
* **LED** – current LED drive value, adjusted automatically (`0x24` = 7.2 mA).
* **rate** – measured FIFO sample rate (about 100 Hz here).
* **!SAT** – the ADC clipped; the auto-gain immediately reduces the LED current.

Allow 5–10 s after placing the finger: BPM needs a few beats, SpO2 needs a
4 s window and a valid pulse.

When the dashboard connects, the Serial Monitor also prints
`BLE: connected (MTU …)` and then either `BLE: live notifications active`
or `BLE: live notify not subscribed (dashboard uses status polling)`.

## BLE dashboard (`index.html`)

### Requirements

* **Chrome or Edge** on a PC/Android phone. Firefox and Safari do not implement
  Web Bluetooth. On Android, Bluetooth and Location must be switched on.
* Web Bluetooth only works in a **secure context**: `http://localhost` or an
  HTTPS page. Opening the file with `file://` usually fails.

Serve the folder and open it through localhost:

```bash
cd Max30102_SMC3
python3 -m http.server 8000
# then open http://localhost:8000 in Chrome/Edge
```

### Using it

1. Click **Pair / Connect** and pick **MAX30102** in the browser dialog.
2. The dashboard immediately sends the PC/phone time to the ESP32 and shows
   the **device clock** in GMT+5:30. The big clock is always the current local
   GMT+5:30 time.
3. **● Start recording** begins a CSV log on the ESP32 (LittleFS). One row per
   second is written while a finger is detected, so recording continues even if
   the browser is closed or the device goes out of range.
4. **■ Stop recording** flushes and closes the file.
5. **⬇ Download CSV** streams `/rec.csv` from the device and saves it with a
   GMT+5:30 timestamp in the filename. Recording is stopped automatically first
   so the file cannot change during the transfer.
6. **⟳ Refresh data** re-syncs the clock and re-reads the device status.
7. **Clear device log** deletes the stored CSV on the ESP32.

The live cards show BPM and SpO2; the chart keeps the last 3 minutes and the
table lists the last 12 measurements. A banner above the cards reports the
sensor state — **Finger off**, **Warming up**, **Detecting pulse**,
**Measuring SpO₂** or **Measuring** — with a hint and the warm-up countdown,
so it is clear whether the setup is working. If the page is not connected,
**Download CSV** exports the samples this page has collected instead.

### CSV format

```
datetime_gmt+0530,epoch,bpm,spo2,pi
2026-11-18 21:14:05,1795000445,72,98,1.21
```

* `datetime_gmt+0530` – already converted to GMT+5:30.
* `epoch` – Unix time in seconds (UTC), useful for spreadsheets.
* `bpm`, `spo2`, `pi` – `0` when that value was not valid in that second.

The default 1.4 MB LittleFS partition holds roughly **8 hours** of one row per
second. The buffer is flushed to flash every 30 s (and on stop) to keep flash
wear low.

## BLE protocol

Service UUID `7a1f0001-3c2d-4e5f-8a9b-0c1d2e3f4a5b`.

| Characteristic | UUID suffix | Properties | Format |
|----------------|-------------|------------|--------|
| Live | `…0002…` | read, notify | text `epoch,bpm,spo2,state`, one line per second; `0` = invalid |
| Control | `…0003…` | write | `TIME:<unixSeconds>`, `START`, `STOP`, `CLEAR`, `DUMP` |
| Status | `…0004…` | read | 20 bytes (see below) |
| Data | `…0005…` | notify | `B,<bytes>` then raw CSV chunks then `E` |

Status bytes (little-endian): `[0]` flags, `[1..4]` rows, `[5..8]` bytes,
`[9..12]` epoch, `[13]` sample rate, `[14]` bpm, `[15]` spo2, `[16]` PI×10,
`[17]` state, `[18]` warm-up seconds left, `[19]` version = 3.

Status flags: bit 0 = recording, bit 1 = clock synced, bit 2 = LittleFS ready,
bit 3 = finger present, bit 4 = warm-up finished.

State values: `0` finger off, `1` warming up, `2` detecting pulse,
`3` pulse found / SpO₂ pending, `4` both values valid.

The dashboard subscribes to Live notifications for 1 Hz updates. It also polls
Status every 1.5 s, so the readings and the state banner keep working even if a
browser does not deliver notifications.

The dashboard negotiates the ATT MTU with the device; CSV chunks are sized to
the negotiated MTU (up to 240 bytes) so the transfer is quick on both Android
and desktop.

## How it works

* Sensor: SpO2 mode (RED + IR), 400 sps, 4× averaging, 18-bit, polled FIFO.
* **Auto-gain**: the LED current is adjusted so the IR DC level stays between
  45 000 and 170 000 counts. This fixes the two classic failures — clipping on
  strong signals (which kills the pulse and the SpO2 ratio) and a signal that
  is too weak to find beats.
* **Filtering**: 0.6–8 Hz band-pass on both channels (4-pole high-pass, so
  breathing wander is removed hard).
* **Pulse**: one beat per positive peak that is followed by a real trough
  within 60–700 ms (spikes and slow drift are both rejected). The threshold is
  35 % of the amplitude of recent accepted beats, with a 60-count noise floor,
  plus a 280 ms refractory. Beat times come from the peak sample, not from when
  the loop noticed it.
* **SpO2**: over the last 4 s the amplitude of the red and IR signals is
  measured at the actual heart-rate frequency (Hann-windowed Goertzel). Using
  one frequency makes the ratio immune to breathing/motion baseline wander,
  which is the usual cause of SpO2 reading 75–85 % on hobby setups.
  `R = (Ared/DCred) / (Air/DCir)`, then Maxim's curve
  `SpO2 = -45.06·R² + 30.354·R + 94.845`. The perfusion index is taken from
  the same IR amplitude, so noise and wander cannot inflate it.
* Results are only accepted when at least 2 beats are present, the perfusion
  index is above 0.1 % and R is in a plausible range; otherwise the display
  keeps the previous value for one window and then shows `--`.
* **Clock**: the dashboard sends `TIME:<epoch>` on connect; the firmware keeps
  `epoch + millis()` and timestamps CSV rows in GMT+5:30. After a reboot the
  clock is unset until the next connection.

## Tuning

| What | Where | Notes |
|------|-------|-------|
| Finger threshold | `FINGER_ON_IR` | 25000. Compare with the `IR=` value printed with no finger. |
| SpO2 averaging | `SPO2_WINDOW_MS` | 4000 ms. Longer = steadier, slower. |
| Warm-up | `WARMUP_MS` | 3000 ms ignored after a finger is placed. |
| LED limits | `LED_PA_MIN` / `LED_PA_MAX` | Auto-gain range, `0x0A`…`0x3F` = 2.0…12.6 mA. |
| BLE name | `BLE_NAME` | Advertising name, default `MAX30102`. |
| CSV path | `REC_PATH` | `/rec.csv` in LittleFS. |

## Troubleshooting

### Serial / sensor

| Symptom | Likely cause |
|----------|--------------|
| `PART_ID = 0xFF` / `not found` | Wiring, 3V3 supply, SDA/SCL swapped. The sketch runs an I2C scan and lists what answers. |
| `no finger` while a finger is on it | IR stays below `FINGER_ON_IR`; check the printed `IR=` and the sensor window. |
| BPM stays `--` | Look at **IR AC**. Below ~100 counts the pulse is too weak: press firmer, cover the sensor from light, clean the glass. |
| SpO2 stays `--` | It needs a valid BPM first (at least 2 beats and PI > 0.1 %). |
| SpO2 jumps around | Movement. Rest the finger, keep still. Median of 3 windows smooths a little. |
| `!SAT` every second | Auto-gain should clear it within a few seconds; if not, the sensor is being over-driven by ambient IR (direct sun/incandescent lamp). |
| `LittleFS: mount failed` | The partition scheme has no LittleFS/SPIFFS area. Select **Default 4MB with spiffs** and re-upload. Serial measurement still works; recording does not. |

### BLE / dashboard

| Symptom | Likely cause |
|----------|--------------|
| `MAX30102` does not appear | Firmware not running, or the board rebooted. Check the Serial Monitor for `BLE: advertising as "MAX30102"`. |
| "Web Bluetooth is not available" | Use Chrome/Edge, not Firefox/Safari. On Android enable Bluetooth and Location. |
| Connect button does nothing / secure-context error | Open the page via `http://localhost:8000` (or HTTPS), not `file://`. |
| `Could not read device status` | The GATT connection dropped; reconnect. Keep the phone/PC within a few metres. |
| Cards stay `--` while the Serial Monitor shows values | Reload the page after uploading the new firmware. Check the Serial Monitor: `BLE: live notifications active` means notifications work; `not subscribed` means the page is using the automatic status-polling fallback (values still update every 1.5 s). |
| Download times out | Move closer, stop other BLE activity, retry. The transfer is paced so the ESP32 BLE queue cannot overflow; a full 1.4 MB log can take a minute or two. |
| Downloaded CSV is incomplete | Some notifications were lost (distance/interference). The page saves what arrived and warns; reconnect and download again. |
| Downloaded CSV is empty | Rows are only written while a finger is detected. Place the finger, wait for BPM/SpO2, then start recording. |
| Device clock shows "not synced" | Click **Sync device clock** (or reconnect). The ESP32 has no battery-backed RTC. |
| Recording stops when the browser closes | It does not: rows are stored on the ESP32. Reconnect to download them. |

## Notes on accuracy

This is a hobby-grade measurement, not a medical device. It uses Maxim's
generic calibration curve and a single LED current, so expect a few percent
error on SpO2 and a few BPM on heart rate even with a good signal. It is fine
for a project demo, not for diagnosis.
