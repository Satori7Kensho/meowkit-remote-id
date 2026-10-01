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
    enum class Page : uint8_t {
        Scan = 0,
        Nearby,
        Radar
    };

    DEVICES* _device = nullptr;
    RemoteIdReceiver _receiver;

    Page _page = Page::Scan;
    uint32_t _lastRenderMs = 0;
    uint8_t _spinner = 0;
    size_t _selected = 0;
    size_t _lastCount = static_cast<size_t>(-1);

    void _handleInput();
    void _switchPage(Page page);

    void _drawFrame(const char* title);
    void _drawScan();
    void _drawNearby();
    void _drawRadar();
    void _drawFooter(const char* hint);

    const char* _stateText() const;
    static const char* _transportText(RemoteIdTransport t);
    static double _distanceMeters(double lat1, double lon1, double lat2, double lon2);
    static double _bearingDegrees(double lat1, double lon1, double lat2, double lon2);
};

} // namespace MOONCAKE::APPS
