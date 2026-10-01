/**
 * @file remote_id_receiver.cpp
 * @brief Receiver lifecycle scaffold.
 *
 * Radio capture is intentionally not enabled in this first milestone.  Keeping
 * the backend inert lets us validate launcher integration and app lifecycle on
 * real MeowKit hardware before taking ownership of the shared Wi-Fi/BLE radio.
 */
#include "remote_id_receiver.h"

namespace MOONCAKE::APPS
{

bool RemoteIdReceiver::begin(DEVICES* device)
{
    _device = device;
    _count = 0;
    for (auto& t : _tracks) t = RemoteIdTrack{};

    _state = State::Starting;

    // Phase 2 will initialise passive Wi-Fi/BLE Remote ID capture here.
    // For the app-shell milestone, expose a healthy listening state so the UI
    // and launcher lifecycle can be tested independently from radio changes.
    _state = State::Listening;
    return true;
}

void RemoteIdReceiver::update()
{
    if (_state != State::Listening) return;
    // Phase 2: drain decoded Remote ID observations into _tracks.
}

void RemoteIdReceiver::end()
{
    // Phase 2 will stop promiscuous Wi-Fi/BLE scanning and restore radio state.
    _state = State::Idle;
    _count = 0;
    _device = nullptr;
}

const RemoteIdTrack* RemoteIdReceiver::track(std::size_t index) const
{
    if (index >= _count || index >= REMOTE_ID_MAX_TRACKS) return nullptr;
    return &_tracks[index];
}

} // namespace MOONCAKE::APPS
