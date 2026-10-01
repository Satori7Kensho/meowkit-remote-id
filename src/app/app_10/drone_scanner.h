/**
 * @file drone_scanner.h
 * @brief MeowKit Drone Scanner — passive FAA/ASTM Remote ID receiver UI.
 */
#pragma once

#include <mooncake.h>
#include <cstdint>
#include "../../bsp/devices.h"
#include "remote_id_receiver.h"

using namespace mooncake;

namespace MOONCAKE::APPS
{

class DroneScanner : public AppAbility {
public:
    explicit DroneScanner(DEVICES* device);
    void onOpen() override;
    void onRunning() override;
    void onClose() override;

private:
    DEVICES* _device = nullptr;
    RemoteIdReceiver _receiver;

    uint32_t _lastRenderMs = 0;
    uint8_t  _spinner = 0;

    void _drawStatic();
    void _drawStatus();
    const char* _stateText() const;
};

} // namespace MOONCAKE::APPS
