# MeowKit Drone Scanner roadmap

**App name:** Drone Scanner  
**Visualization mode:** Drone Radar

This fork adds a passive **Remote ID / Drone Scanner** application to MeowKit.

## Goal

Receive nearby standards-compliant drone Remote ID broadcasts on the MeowKit's
ESP32-S3, decode them locally, and present useful telemetry without requiring
internet access.

The project is receiver-only. It does not transmit to, control, interfere with,
or disable aircraft.

## Milestones

1. **App shell** — launcher integration, clean open/close lifecycle.
2. **Wi-Fi Remote ID** — ASTM/OpenDroneID Wi-Fi Beacon and Wi-Fi NAN capture.
3. **Decode telemetry** — UAS ID, position, altitude, speed, heading, signal.
4. **Bluetooth Remote ID** — legacy BLE + BLE 5 extended advertising.
5. **Multi-drone table** — active tracks, last-seen expiry, transport + RSSI.
6. **MeowKit UI** — nearby list and per-drone detail view.
7. **Drone Radar view** — Home-relative radar with cardinal directions, stable distance rings, per-drone labels, and heading ticks.
8. **Home awareness** — saved receiver/Home coordinate, horizontal range, bearing/cardinal direction, approximate direct range using RID height, and closing/departing trend from successive positions.
9. **Home-zone alerts** — configurable 50 m–5 km radius, with a distinct visual alert when a detected track enters the configured zone.
10. **SD logging** — encounter snapshots saved to CSV when an SD card is available.
11. **Diagnostics** — Wi-Fi channel, BLE state, RID message counts, unique tracks, and dropped-frame counters.
12. **PC bridge** — newline-delimited USB JSON plus a localhost Windows bridge for downstream tools.
13. **God's Eye View integration** — local Remote ID detections exposed as a distinct Local Drones layer.

## Architecture

```text
ESP32-S3 Wi-Fi / BLE
        |
        v
RemoteIdReceiver
        |
        v
OpenDroneID decoder
        |
        v
RemoteIdTrack table
        |
        +--> MeowKit Drone Scanner UI
        |
        +--> optional SD / USB / Wi-Fi JSON output
```

The radio backend is kept separate from the UI so scanner shutdown can restore
shared Wi-Fi/BLE resources cleanly when the user leaves the app.

## Third-party components

The intended standards decoder is
[OpenDroneID Core C](https://github.com/opendroneid/opendroneid-core-c), which is
licensed under Apache-2.0. Existing ESP32 Remote ID scanner implementations are
used as implementation references where their licenses permit it.


## Home-relative presentation

When Home is configured, the Nearby view prioritizes practical spatial context:

```text
HOME 420m  SE  BRG 137
Direct range ~437m (RID height)
CLOSING
```

The radar uses a stable human-friendly scale, marks Home at the centre, labels
tracks as D1, D2, etc., and draws a short heading tick from each track marker.
A distinct ring marks the configured Home alert radius.

"Direct range" is intentionally approximate because the Remote ID Height field
may use a reference other than the Home ground elevation. Horizontal Home range
and bearing are the primary distance/direction values.
