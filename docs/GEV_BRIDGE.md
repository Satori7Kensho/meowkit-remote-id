# God's Eye View bridge

Drone Scanner emits newline-delimited JSON over MeowKit USB serial. The included
Windows-friendly bridge turns that feed into a localhost-only HTTP endpoint.

This is the transport layer for the planned **Local Drones** integration in
God's Eye View. GEV itself still needs a small custom provider/layer before it
will display this endpoint.

## Install

```powershell
py -m pip install -r tools/requirements-gev-bridge.txt
```

## Run

```powershell
py tools/gev_bridge.py --port COM6
```

Replace `COM6` if Windows assigns the MeowKit a different port.

The bridge then serves:

- `http://127.0.0.1:8765/drones.json`
- `http://127.0.0.1:8765/health`

It binds to loopback only and does not send Remote ID data to the internet.

## Data lifecycle

The bridge keeps tracks in memory and drops a drone after 120 seconds without a
new observation. It writes no files. Persistent encounter logging, when enabled
by the firmware and an SD card is available, remains on the MeowKit SD card.
