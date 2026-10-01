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

struct RemoteIdTrack {
    bool     active = false;
    uint32_t lastSeenMs = 0;

    uint8_t  mac[6] = {0};
    int8_t   rssi = 0;

    char uasId[REMOTE_ID_ID_LEN]      = {0};
    char operatorId[REMOTE_ID_ID_LEN] = {0};

    double latitude  = 0.0;
    double longitude = 0.0;
    double operatorLatitude  = 0.0;
    double operatorLongitude = 0.0;

    float altitudeMslM = 0.0f;
    float heightAglM   = 0.0f;
    float speedMps     = 0.0f;
    float headingDeg   = 0.0f;

    bool hasLocation         = false;
    bool hasOperatorLocation = false;
    RemoteIdTransport transport = RemoteIdTransport::Unknown;
};

} // namespace MOONCAKE::APPS
