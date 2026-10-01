/**
 * @file remote_id_types.h
 * @brief Data model shared by the MeowKit Drone Scanner UI and receiver backend.
 */
#pragma once

#include <cstddef>
#include <cstdint>

namespace MOONCAKE::APPS
{

static constexpr std::size_t REMOTE_ID_MAX_TRACKS = 16;
static constexpr std::size_t REMOTE_ID_ID_LEN     = 21; // ASTM/OpenDroneID max ID length + NUL

enum class RemoteIdTransport : uint8_t {
    Unknown = 0,
    WiFiBeacon,
    WiFiNaN,
    BleLegacy,
    BleExtended
};

enum class RemoteIdHeightReference : uint8_t {
    Unknown = 0,
    Takeoff,
    Ground
};

struct RemoteIdTrack {
    bool     active = false;
    uint32_t lastSeenMs = 0;
    uint32_t lastExportMs = 0;
    uint32_t lastLogMs = 0;

    uint8_t  mac[6] = {0};
    int8_t   rssi = 0;

    char uasId[REMOTE_ID_ID_LEN]      = {0};
    char operatorId[REMOTE_ID_ID_LEN] = {0};

    double latitude  = 0.0;
    double longitude = 0.0;

    // Previous valid position retained for Home-relative closing/departing
    // calculations. Remote ID broadcasts arrive frequently enough that two
    // successive location samples are useful without predicting a flight path.
    double previousLatitude = 0.0;
    double previousLongitude = 0.0;
    uint32_t locationSeenMs = 0;
    uint32_t previousLocationSeenMs = 0;
    bool hasPreviousLocation = false;

    double operatorLatitude  = 0.0;
    double operatorLongitude = 0.0;

    // Remote ID exposes several vertical fields. Keep them distinct so the UI
    // never labels WGS84 ellipsoid altitude as MSL or assumes Height is always AGL.
    float geoAltitudeM  = -1000.0f; // WGS84 HAE
    float baroAltitudeM = -1000.0f; // pressure altitude, 1013.24 mb reference
    float heightM       = -1000.0f; // above takeoff or ground per heightReference
    RemoteIdHeightReference heightReference = RemoteIdHeightReference::Unknown;

    float speedMps     = 0.0f;
    float headingDeg   = 0.0f;

    bool hasLocation         = false;
    bool hasGeoAltitude      = false;
    bool hasBaroAltitude     = false;
    bool hasHeight           = false;
    bool hasOperatorLocation = false;
    bool insideHomeZone      = false;
    RemoteIdTransport transport = RemoteIdTransport::Unknown;
};

} // namespace MOONCAKE::APPS
