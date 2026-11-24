# indoorV10-fw-birdy

ESP-IDF firmware for the indoorV10 indoor air monitor (ESP32-C5) with a
2.13" e-paper panel, a capacitive touch front, and an 11-LED WS2812 bar.

## Features

- **Sensors**: temperature and humidity (SHT40), pressure (SPL07), VOC/NOx (SGP41),
  particulates (Sensirion SPS30 or Plantower PMS5003, detected at boot) and CO2
  (Senseair S88 or S12, detected at boot).
- **E-paper dashboard**: PM2.5 and CO2 as headline figures, climate, gas and
  pressure groups, a top bar with sensor status, the clock (SNTP), and a
  "NO WIFI" flag. Notifications clear themselves after 15 s. Partial refreshes,
  with a periodic full refresh to clear ghosting.
- **Touch front**: three supported controllers, chosen at build time:
  - CAP1166: three buttons and a slider. The slider sets LED bar brightness.
  - CAP1133: three buttons, each with its own LED.
  - CAP1203: three buttons.

  Pad LEDs are lit while idle and go dark under a finger. They switch off after
  an idle timeout, and the next tap only wakes them. Quick presses during a
  refresh are folded into one repaint.
- **LED bar**: shows PM2.5 or CO2 (selectable in the menu) using the AirGradient
  ONE's bands and colours, either as a bar that grows with the reading or as the
  whole bar in the band's colour. The bar turns purple when Wi-Fi is down.
- **Wi-Fi**: connects as a station. If it can't, it offers a setup portal: an
  open SoftAP with a form at `http://192.168.4.1`, which closes after 5 minutes.
- **Cloud upload**: posts measurements on a fixed interval. It can be switched
  off from the menu.
- **Menu** (left/right move, middle selects):
  - LED ring on/off
  - LED shows (PM2.5 / CO2)
  - Frontlight level
  - Pad LED timeout
  - Theme (light/dark)
  - Temperature unit
  - Upload
  - Refresh rate (5 s to 1 min)
  - Full refresh
  - Touch test: a guided per-pad check showing signal, threshold and peak
  - About

  Menu settings are not saved across reboots.

## Build options

| Where | Option | Values |
|---|---|---|
| `components/sensors/include/touch.h` | `TOUCH_IC` | `TOUCH_IC_CAP1166`, `TOUCH_IC_CAP1133`, `TOUCH_IC_CAP1203` |
| `components/sensors/include/touch.h` | `TOUCH_GAIN` | analog gain `1`, `2`, `4`, `8` (CAP1133/CAP1166 only) |
| `main/main.c` | `LED_BAR_STYLE` | `LED_BAR_STYLE_LEVEL` (bar graph) or `LED_BAR_STYLE_FULL` |
| `main/main.c` | `LED_BAR_FROM_FAR_END` | `0` grow from the LED nearest the MCU, `1` from the far end |
| `idf.py menuconfig` | `DISPLAY_FRONTLIGHT` | frontlit panel or bare panel |
| `idf.py menuconfig` | `CLOCK_TZ`, `CLOCK_NTP_SERVER` | POSIX time zone (default `<+07>-7`), NTP host |
| `idf.py menuconfig` | `WIFI_STA_SSID`, `WIFI_STA_PASSWORD` | default network |
| `idf.py menuconfig` | `CLOUD_*` | host, serial, post interval, TLS |

## Build and flash

Built with ESP-IDF v6.0, target `esp32c5`:

```sh
idf.py set-target esp32c5   # once
idf.py build
idf.py -p <port> flash monitor
```

## Branches

- `main`: current firmware, rev-C board pin map.
- `rev-B-ui`: the same UI on the rev-B board pin map (I2C on IO25/IO26, EPD CS
  on IO7, Touch_INT/RST on IO6/IO2). The tag `rev-B` marks the last firmware
  written for that board before the UI work.
- `co2-calibration`: work-in-progress branch for CO2 calibration.
