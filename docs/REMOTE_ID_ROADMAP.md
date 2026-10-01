# MeowKit Drone Scanner roadmap

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
7. **Relative/radar view** — position display when a receiver reference location
   is available.
8. **Optional SD logging** — user-controlled encounter logging.
9. **Optional alerts** — LED/sound on newly observed Remote ID transmitters.
10. **Optional PC bridge** — JSON output for tools such as God's Eye View.

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
