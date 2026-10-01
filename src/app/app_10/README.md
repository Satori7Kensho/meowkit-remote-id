# Drone Scanner app

Native MeowKit app for passive ASTM/FAA Remote ID reception.

## Current implementation

- launcher/UI shell
- receiver lifecycle abstraction
- active-track data model
- vendored OpenDroneID Core C decoder (Apache-2.0)
- adapter from decoded OpenDroneID data to MeowKit track records

The radio backend is intentionally staged separately. Wi-Fi/BLE capture will be
enabled only after lifecycle and decoder integration build cleanly.

## Naming

- **Drone Scanner** = complete app
- **Drone Radar** = planned live visualization screen
