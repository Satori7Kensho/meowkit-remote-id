# Drone Scanner hardware test checklist

Use this after flashing a new test build. The goal is to catch regressions as features are added.

## 1. Firmware / launcher sanity

- MeowKit boots normally.
- Existing launcher/apps still appear.
- **Drone Scanner** appears in the launcher.
- Drone Scanner opens without rebooting or freezing.
- Hold B exits Drone Scanner.
- Wi-Fi reconnects after leaving Drone Scanner if it was connected before opening.
- Existing BLE-related MeowKit apps still work after Drone Scanner exits.

## 2. Scanner status / diagnostics

- Status page reaches `LISTENING`.
- Diagnostics page shows Wi-Fi channel changing over time.
- BLE scanner reports `ACTIVE` when available.
- Dropped-frame counter is not increasing uncontrollably.
- App remains responsive while scanning.

## 3. Nearby drone details

For a real Remote ID transmitter, verify every field that is actually broadcast:

- UAS / Remote ID
- transport: Wi-Fi Beacon, Wi-Fi NAN, BLE, or BLE 5
- RSSI
- latitude / longitude
- speed
- heading
- **Height** with reference:
  - `AGL` when Remote ID says height is over ground
  - `ATO` when Remote ID says height is over takeoff
- **Geo altitude** labelled `HAE` when present
- barometric altitude when geo altitude is unavailable

The app must not call WGS84 HAE altitude "MSL".

## 4. Home-relative information

After setting Home:

- Home persists after reboot.
- Nearby view shows `HOME <distance>m <cardinal> BRG <degrees>`.
- Main status shows the closest tracked drone.
- Closest-drone line includes height/altitude when available.
- Consecutive location samples produce `CLOSING`, `DEPARTING`, or `STEADY`.
- `OVERHEAD VICINITY` appears only for small horizontal Home distance.
- Approximate direct range appears only when a usable Remote ID height is present.

## 5. Drone Radar

- Home is centered.
- N / E / S / W labels are correct.
- Distance rings use stable scales.
- D1 / D2 / D3 labels correspond to active tracks.
- Each marker has a heading tick pointing in the broadcast course direction.
- Home alert radius is visible as a distinct ring.
- Targets inside the alert radius use the alert color.

## 6. Alerts

- A newly seen drone produces the normal green visual alert.
- A drone entering the configured Home zone produces the stronger orange visual alert.
- Normal MeowKit LED settings are restored after the alert and after exiting the app.

## 7. Logging and PC bridge

With SD available:

- `/drone_scanner.csv` is created.
- rows contain location, vertical fields, speed, heading, and operator location when broadcast.

With USB connected:

- newline-delimited JSON appears on serial.
- vertical fields keep their meaning (`geo_altitude_m`, `baro_altitude_m`, `height_m`, `height_reference`).
- `tools/gev_bridge.py` serves current tracks on localhost.

## 8. Regression rule

If a new build loses any previously working item above, treat it as a regression
and fix it before adding more features.
