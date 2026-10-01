/**
 * @file remote_id_exporter.h
 * @brief USB JSON feed and optional SD encounter logging for Drone Scanner.
 */
#pragma once

#include "remote_id_types.h"
#include "../../bsp/devices.h"

namespace MOONCAKE::APPS
{

class RemoteIdExporter {
public:
    void begin(DEVICES* device);
    void end();

    // Emits newline-delimited JSON over USB serial for PC consumers such as
    // a future God's Eye View local-drone bridge.
    void emitJson(const RemoteIdTrack& track, bool firstSeen);

    // Appends a compact CSV snapshot to SD when available.
    void logCsv(const RemoteIdTrack& track, bool firstSeen);

private:
    DEVICES* _device = nullptr;
    bool _sdReady = false;
    bool _mountedByUs = false;

    static const char* _transport(RemoteIdTransport t);
    static void _printEscapedJson(const char* value);
};

} // namespace MOONCAKE::APPS
