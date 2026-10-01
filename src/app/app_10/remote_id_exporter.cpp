/**
 * @file remote_id_exporter.cpp
 */
#include "remote_id_exporter.h"

#include <Arduino.h>
#include <SD_MMC.h>
#include <FS.h>

namespace MOONCAKE::APPS
{

static constexpr const char* LOG_PATH = "/drone_scanner.csv";

const char* RemoteIdExporter::_transport(RemoteIdTransport t)
{
    switch (t) {
    case RemoteIdTransport::WiFiBeacon:  return "wifi_beacon";
    case RemoteIdTransport::WiFiNaN:     return "wifi_nan";
    case RemoteIdTransport::BleLegacy:   return "ble_legacy";
    case RemoteIdTransport::BleExtended: return "ble_extended";
    default:                             return "unknown";
    }
}

void RemoteIdExporter::begin(DEVICES* device)
{
    _device = device;
    _sdReady = false;
    _mountedByUs = false;

    if (!_device) return;

    if (_device->sd.isReady()) {
        _sdReady = true;
    } else if (_device->sd.begin(false)) {
        _sdReady = true;
        _mountedByUs = true;
    }

    if (_sdReady && !SD_MMC.exists(LOG_PATH)) {
        File f = SD_MMC.open(LOG_PATH, FILE_WRITE);
        if (f) {
            f.println("uptime_ms,event,uas_id,operator_id,transport,rssi,latitude,longitude,altitude_m,height_m,speed_mps,heading_deg,operator_latitude,operator_longitude");
            f.close();
        }
    }
}

void RemoteIdExporter::end()
{
    if (_device && _mountedByUs)
        _device->sd.end();

    _device = nullptr;
    _sdReady = false;
    _mountedByUs = false;
}

void RemoteIdExporter::_printEscapedJson(const char* value)
{
    if (!value) return;

    for (const char* p = value; *p; ++p) {
        switch (*p) {
        case '\\': Serial.print("\\\\"); break;
        case '"':  Serial.print("\\\""); break;
        case '\n': Serial.print("\\n"); break;
        case '\r': Serial.print("\\r"); break;
        case '\t': Serial.print("\\t"); break;
        default:
            if (static_cast<unsigned char>(*p) >= 0x20)
                Serial.print(*p);
            break;
        }
    }
}

void RemoteIdExporter::emitJson(const RemoteIdTrack& t, bool firstSeen)
{
    Serial.print("{\"type\":\"meowkit_remote_id\",\"event\":\"");
    Serial.print(firstSeen ? "first_seen" : "update");
    Serial.print("\",\"uptime_ms\":");
    Serial.print(t.lastSeenMs);

    Serial.print(",\"uas_id\":\"");
    _printEscapedJson(t.uasId);
    Serial.print("\",\"operator_id\":\"");
    _printEscapedJson(t.operatorId);

    Serial.print("\",\"transport\":\"");
    Serial.print(_transport(t.transport));
    Serial.print("\",\"rssi\":");
    Serial.print(static_cast<int>(t.rssi));

    Serial.print(",\"has_location\":");
    Serial.print(t.hasLocation ? "true" : "false");

    if (t.hasLocation) {
        Serial.print(",\"latitude\":"); Serial.print(t.latitude, 7);
        Serial.print(",\"longitude\":"); Serial.print(t.longitude, 7);
        Serial.print(",\"altitude_m\":"); Serial.print(t.altitudeMslM, 1);
        Serial.print(",\"height_m\":"); Serial.print(t.heightAglM, 1);
        Serial.print(",\"speed_mps\":"); Serial.print(t.speedMps, 2);
        Serial.print(",\"heading_deg\":"); Serial.print(t.headingDeg, 1);
    }

    Serial.print(",\"has_operator_location\":");
    Serial.print(t.hasOperatorLocation ? "true" : "false");

    if (t.hasOperatorLocation) {
        Serial.print(",\"operator_latitude\":"); Serial.print(t.operatorLatitude, 7);
        Serial.print(",\"operator_longitude\":"); Serial.print(t.operatorLongitude, 7);
    }

    Serial.println("}");
}

void RemoteIdExporter::logCsv(const RemoteIdTrack& t, bool firstSeen)
{
    if (!_sdReady) return;

    File f = SD_MMC.open(LOG_PATH, FILE_APPEND);
    if (!f) return;

    f.print(t.lastSeenMs); f.print(',');
    f.print(firstSeen ? "first_seen" : "update"); f.print(',');
    f.print(t.uasId); f.print(',');
    f.print(t.operatorId); f.print(',');
    f.print(_transport(t.transport)); f.print(',');
    f.print(static_cast<int>(t.rssi)); f.print(',');

    if (t.hasLocation) {
        f.print(t.latitude, 7); f.print(',');
        f.print(t.longitude, 7); f.print(',');
        f.print(t.altitudeMslM, 1); f.print(',');
        f.print(t.heightAglM, 1); f.print(',');
        f.print(t.speedMps, 2); f.print(',');
        f.print(t.headingDeg, 1); f.print(',');
    } else {
        f.print(",,,,,,");
    }

    if (t.hasOperatorLocation) {
        f.print(t.operatorLatitude, 7); f.print(',');
        f.print(t.operatorLongitude, 7);
    } else {
        f.print(',');
    }

    f.println();
    f.close();
}

} // namespace MOONCAKE::APPS
