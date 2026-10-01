/**
 * @file remote_id_decoder.h
 * @brief Adapter from OpenDroneID decoded structures to Drone Scanner tracks.
 */
#pragma once

#include "remote_id_types.h"
extern "C" {
#include <opendroneid.h>
}

namespace MOONCAKE::APPS
{

class RemoteIdDecoder {
public:
    static bool fromUasData(RemoteIdTrack& out,
                            const ODID_UAS_Data& uas,
                            const uint8_t mac[6],
                            int8_t rssi,
                            RemoteIdTransport transport,
                            uint32_t seenMs);

    static bool decodeMessagePack(RemoteIdTrack& out,
                                  const uint8_t* bytes,
                                  size_t length,
                                  const uint8_t mac[6],
                                  int8_t rssi,
                                  RemoteIdTransport transport,
                                  uint32_t seenMs);
};

} // namespace MOONCAKE::APPS
