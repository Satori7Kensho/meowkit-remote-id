/**
 * @file remote_id_receiver.h
 * @brief Passive Remote ID receiver facade for the Drone Scanner app.
 *
 * Phase 1 deliberately keeps the radio backend isolated behind this interface.
 * The next implementation phase will add Wi-Fi Beacon/NAN and BLE receivers
 * without coupling ESP-IDF radio state directly into the UI.
 */
#pragma once

#include "remote_id_types.h"
#include "../../bsp/devices.h"
#include <cstddef>

namespace MOONCAKE::APPS
{

class RemoteIdReceiver {
public:
    enum class State : uint8_t {
        Idle = 0,
        Starting,
        Listening,
        Error
    };

    bool begin(DEVICES* device);
    void update();
    void end();

    State state() const { return _state; }
    std::size_t count() const { return _count; }
    const RemoteIdTrack* track(std::size_t index) const;

private:
    DEVICES* _device = nullptr;
    State _state = State::Idle;
    RemoteIdTrack _tracks[REMOTE_ID_MAX_TRACKS] = {};
    std::size_t _count = 0;
};

} // namespace MOONCAKE::APPS
