# ESP32 C6 RD-03D Live Radar

This project turns an ESP32 C6 and an Ai Thinker RD-03D radar module into a WiFi based live radar viewer. The ESP32 C6 reads target data from the RD-03D over UART, connects to a local WiFi network and serves a browser dashboard showing a 9 meter, 120 degree forward radar sector.

![RD-03D browser radar dashboard](assets/Screenshot 2026-05-15 000924.png)

## Hardware

- ESP32 C6 development board
- Ai Thinker RD-03D radar module
- Jumper wires
- 5V power source suitable for the ESP32 C6 and RD-03D
- USB cable for programming

## Wiring

| RD-03D | ESP32 C6 |
| --- | --- |
| VCC | 5V |
| GND | GND |
| TX | GPIO7 |
| RX | GPIO6 |

## Arduino IDE board settings used during testing

- Board: ESP32C6 Dev Module
- Upload Speed: 115200
- USB CDC On Boot: Enabled
- Serial Monitor: 115200 baud
- Radar UART: 256000 baud on GPIO7 RX and GPIO6 TX

## Required Arduino libraries

Install these before compiling:

- ESP32 board package by Espressif Systems
- ESP Async WebServer, usually shown as `ESPAsyncWebServer` or `ESP Async WebServer`
- AsyncTCP, required by ESP Async WebServer on ESP32 boards
- ArduinoJson

The following are included with the ESP32 board package and do not usually need separate installation:

- WiFi
- Arduino core / Arduino.h

This web dashboard version does not require TFT_eSPI because the display is served in a browser.

## USB driver

The `drivers/CH343SER.ZIP` file is included for Windows systems that need the CH343 USB serial driver. Install it only if the ESP32 C6 board does not appear as a usable COM port.

## WiFi credentials

The published sketch is sanitized and does not include private WiFi credentials.

For local use:

1. Open the `RD03D_ESP32C6_Web_Radar` sketch folder.
2. Copy `secrets.example.h`.
3. Rename the copy to `secrets.h`.
4. Edit `secrets.h` with your WiFi network name and password.
5. Upload the sketch.

`secrets.h` is ignored by git and should not be committed.

## Current features

- 9 meter radar range
- 120 degree forward sector display instead of a 360 degree display
- RD-03D UART decoding on GPIO7 RX and GPIO6 TX
- Up to three simultaneous RD-03D target slots
- Filtered confirmed-target radar display
- Pedestrian, vehicle and unknown moving classification
- Live target plotting after detection qualification
- Longer live movement trails
- Detection history table
- Saved long-path trails
- Selectable saved trails that redraw on the radar screen
- Centered and stable browser UI layout
- Manual radar reset button
- Automatic frame stall recovery
- Automatic frozen-content recovery
- Logging on and off control
- Clear Log button that clears all detection history stored in device RAM
- CSV export of detection history
- More aggressive foliage, breeze and micro movement filtering

## Display behavior

The radar display shows filtered confirmed targets only. Raw one-frame detections are not plotted immediately.

Pedestrian and vehicle targets appear after the classification thresholds are met. Unknown moving targets appear only after repeated measurable movement reaches the unknown target criteria.

## Logging behavior

Logging stores completed movement events only when the tracked path length is at least 2 meters.

Events with a continuous path of 4 meters or more save their trail points in RAM. Selecting one of those log items redraws the saved trail on the radar display. Saved trail items remain available until the ESP32 is rebooted or Clear Log is used.

Clear Log clears the entire stored detection history, including pedestrian, vehicle and unknown entries and any saved trails.

## Log capacity and RAM behavior

Log storage is fixed size in RAM and does not grow indefinitely while logging is left on.

The sketch stores up to 48 log items. When the log is full, new entries replace the oldest non-pinned log item. Pinned trail items are preserved. If all 48 entries are pinned trail items, new log entries are dropped until the log is cleared or the ESP32 is rebooted.

Each pinned trail item can store up to 80 trail points. The ESP32 continues running with the fixed log array, although the browser response can become larger as saved trail history accumulates.

## Filtering

The sketch reduces false detections caused by bushes, foliage and breeze driven movement by using speed, distance jitter, lateral jitter, stationary clearing and repeated micro movement rejection.

The tuning constants are near the top of the sketch and can be adjusted for the final mounting location.

## Use

After upload, open Serial Monitor at 115200 baud. Once connected to WiFi, the ESP32 C6 prints the local web address, for example:

```text
Open on any device: http://192.168.1.11
```

Open that address from a phone, tablet or computer on the same WiFi network.

## Repository layout

```text
.
├── RD03D_ESP32C6_Web_Radar/
│   ├── RD03D_ESP32C6_Web_Radar.ino
│   └── secrets.example.h
├── assets/
│   └── Screenshot 2026-05-15 000924.png
├── drivers/
│   └── CH343SER.ZIP
├── .gitignore
├── LICENSE
├── README.md
└── UPDATE_NOTES.md
```

## Notes

- This version intentionally omits RF transceiver code.
- The sketch supports up to three RD-03D targets.
- The IP address may change unless your router assigns a DHCP reservation to the ESP32 C6.
- If no radar data appears, confirm the RD-03D TX wire goes to GPIO7 and the RD-03D RX wire goes to GPIO6.
