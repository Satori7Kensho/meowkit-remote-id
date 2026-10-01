/**
 * @file remote_id_decoder.cpp
 * @brief OpenDroneID decoder adapter.
 */
#include "remote_id_decoder.h"
#include <cstring>

namespace MOONCAKE::APPS
{

static void copy_id(char* dst, size_t dstLen, const char* src)
{
    if (!dst || !dstLen) return;
    dst[0] = '\0';
    if (!src) return;
    std::strncpy(dst, src, dstLen - 1);
    dst[dstLen - 1] = '\0';
}

bool RemoteIdDecoder::fromUasData(RemoteIdTrack& out,
                                  const ODID_UAS_Data& uas,
                                  const uint8_t mac[6],
                                  int8_t rssi,
                                  RemoteIdTransport transport,
                                  uint32_t seenMs)
{
    RemoteIdTrack next{};
    next.active = true;
    next.lastSeenMs = seenMs;
    next.rssi = rssi;
    next.transport = transport;
    if (mac) std::memcpy(next.mac, mac, sizeof(next.mac));

    bool useful = false;

    for (int i = 0; i < ODID_BASIC_ID_MAX_MESSAGES; ++i) {
        if (uas.BasicIDValid[i]) {
            copy_id(next.uasId, sizeof(next.uasId), uas.BasicID[i].UASID);
            useful = true;
            break;
        }
    }

    if (uas.OperatorIDValid) {
        copy_id(next.operatorId, sizeof(next.operatorId), uas.OperatorID.OperatorId);
        useful = true;
    }

    if (uas.LocationValid) {
        next.latitude     = uas.Location.Latitude;
        next.longitude    = uas.Location.Longitude;
        next.altitudeMslM = uas.Location.AltitudeGeo;
        next.heightAglM   = uas.Location.Height;
        next.speedMps     = uas.Location.SpeedHorizontal;
        next.headingDeg   = uas.Location.Direction;

        const bool latOk = next.latitude >= -90.0 && next.latitude <= 90.0;
        const bool lonOk = next.longitude >= -180.0 && next.longitude <= 180.0;
        next.hasLocation = latOk && lonOk && !(next.latitude == 0.0 && next.longitude == 0.0);
        useful = true;
    }

    if (uas.SystemValid) {
        next.operatorLatitude  = uas.System.OperatorLatitude;
        next.operatorLongitude = uas.System.OperatorLongitude;

        const bool latOk = next.operatorLatitude >= -90.0 && next.operatorLatitude <= 90.0;
        const bool lonOk = next.operatorLongitude >= -180.0 && next.operatorLongitude <= 180.0;
        next.hasOperatorLocation =
            latOk && lonOk &&
            !(next.operatorLatitude == 0.0 && next.operatorLongitude == 0.0);
        useful = true;
    }

    if (!useful) return false;
    out = next;
    return true;
}

bool RemoteIdDecoder::decodeMessagePack(RemoteIdTrack& out,
                                        const uint8_t* bytes,
                                        size_t length,
                                        const uint8_t mac[6],
                                        int8_t rssi,
                                        RemoteIdTransport transport,
                                        uint32_t seenMs)
{
    if (!bytes || length == 0) return false;

    ODID_UAS_Data uas{};
    odid_initUasData(&uas);

    if (odid_message_process_pack(&uas, bytes, length) < 0)
        return false;

    return fromUasData(out, uas, mac, rssi, transport, seenMs);
}

} // namespace MOONCAKE::APPS
