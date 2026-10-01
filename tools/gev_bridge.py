#!/usr/bin/env python3
"""
MeowKit Drone Scanner -> local HTTP bridge.

Reads newline-delimited JSON emitted by the MeowKit over USB serial and exposes
only the latest Remote ID tracks on localhost. Intended for future integration
with a local God's Eye View "Local Drones" provider.

No internet connection is used and the HTTP server binds to 127.0.0.1 only.
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial
from serial.tools import list_ports


TRACK_TTL_SECONDS = 120.0


class State:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.tracks: dict[str, dict] = {}
        self.serial_port = ""
        self.last_packet_monotonic = 0.0

    @staticmethod
    def key_for(msg: dict) -> str:
        uas_id = str(msg.get("uas_id") or "").strip()
        if uas_id:
            return f"uas:{uas_id}"
        # Current firmware normally supplies an ID by the time a track is useful.
        # Keep an anonymous fallback so the bridge remains robust.
        return f"anon:{msg.get('operator_id','')}:{msg.get('transport','')}"

    def ingest(self, msg: dict) -> None:
        if msg.get("type") != "meowkit_remote_id":
            return

        now = time.monotonic()
        item = dict(msg)
        item["_received_monotonic"] = now

        with self.lock:
            self.tracks[self.key_for(item)] = item
            self.last_packet_monotonic = now

    def snapshot(self) -> dict:
        now = time.monotonic()
        with self.lock:
            stale = [
                key for key, value in self.tracks.items()
                if now - float(value.get("_received_monotonic", now)) > TRACK_TTL_SECONDS
            ]
            for key in stale:
                self.tracks.pop(key, None)

            drones = []
            for value in self.tracks.values():
                public = {
                    k: v for k, v in value.items()
                    if not k.startswith("_")
                }
                public["age_s"] = round(
                    now - float(value.get("_received_monotonic", now)), 2
                )
                drones.append(public)

            age = None
            if self.last_packet_monotonic:
                age = round(now - self.last_packet_monotonic, 2)

            return {
                "source": "meowkit-drone-scanner",
                "serial_port": self.serial_port,
                "last_packet_age_s": age,
                "count": len(drones),
                "drones": drones,
            }


STATE = State()


def find_port(explicit: str | None) -> str:
    if explicit:
        return explicit

    candidates = list(list_ports.comports())
    usb = [
        p.device for p in candidates
        if "USB" in (p.description or "").upper()
        or "ESP" in (p.description or "").upper()
    ]
    if len(usb) == 1:
        return usb[0]

    if not candidates:
        raise RuntimeError("No serial ports found.")

    names = ", ".join(f"{p.device} ({p.description})" for p in candidates)
    raise RuntimeError(
        "Could not choose the MeowKit serial port automatically. "
        f"Available ports: {names}. Use --port COM6 (or your current COM port)."
    )


def serial_worker(port: str, baud: int) -> None:
    while True:
        try:
            with serial.Serial(port, baudrate=baud, timeout=1) as ser:
                STATE.serial_port = port
                print(f"[bridge] Listening to {port} @ {baud}")
                while True:
                    raw = ser.readline()
                    if not raw:
                        continue
                    try:
                        line = raw.decode("utf-8", errors="strict").strip()
                        msg = json.loads(line)
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        # Normal firmware debug logs share the serial port.
                        continue
                    STATE.ingest(msg)
        except serial.SerialException as exc:
            print(f"[bridge] Serial error: {exc}; retrying in 2 s")
            time.sleep(2)


class Handler(BaseHTTPRequestHandler):
    def _json(self, status: int, obj: object) -> None:
        payload = json.dumps(obj, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "http://localhost")
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self) -> None:
        if self.path in ("/", "/drones", "/drones.json"):
            self._json(200, STATE.snapshot())
            return
        if self.path == "/health":
            snap = STATE.snapshot()
            self._json(200, {
                "ok": True,
                "source": snap["source"],
                "serial_port": snap["serial_port"],
                "count": snap["count"],
                "last_packet_age_s": snap["last_packet_age_s"],
            })
            return
        self._json(404, {"error": "not_found"})

    def log_message(self, fmt: str, *args: object) -> None:
        # Keep terminal readable; serial/RID status is more useful.
        return


def main() -> None:
    ap = argparse.ArgumentParser(description="MeowKit Drone Scanner local bridge")
    ap.add_argument("--port", help="Serial port, e.g. COM6. Auto-detect if omitted.")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--http-port", type=int, default=8765)
    args = ap.parse_args()

    port = find_port(args.port)

    thread = threading.Thread(
        target=serial_worker, args=(port, args.baud), daemon=True
    )
    thread.start()

    server = ThreadingHTTPServer(("127.0.0.1", args.http_port), Handler)
    print(f"[bridge] Local API: http://127.0.0.1:{args.http_port}/drones.json")
    print("[bridge] Ctrl+C to stop")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
