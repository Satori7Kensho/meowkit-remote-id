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
        Radar,
        Diagnostics,
        HomeSetup
    };

    enum class HomeEditField : uint8_t {
        Latitude = 0,
        Longitude,
        AlertRadius,
        Save
    };

    DEVICES* _device = nullptr;
    RemoteIdReceiver _receiver;

    Page _page = Page::Scan;
    uint32_t _lastRenderMs = 0;
    uint8_t _spinner = 0;
    size_t _selected = 0;
    size_t _lastCount = static_cast<size_t>(-1);

    // Receiver/Home location is stored only on the MeowKit in a dedicated
    // Preferences namespace. It is never uploaded by Drone Scanner.
    bool _homeConfigured = false;
    double _homeLat = 0.0;
    double _homeLon = 0.0;
    HomeEditField _homeEditField = HomeEditField::Latitude;
    uint8_t _homeStepIndex = 0;
    float _homeAlertRadiusM = 250.0f;

    void _handleInput();
    void _switchPage(Page page);

    void _drawFrame(const char* title);
    void _drawScan();
    void _drawNearby();
    void _drawRadar();
    void _drawDiagnostics();
    void _drawHomeSetup();
    void _drawFooter(const char* hint);

    void _loadHomeLocation();
    void _saveHomeLocation();
    void _adjustHome(int direction);
    void _adjustAlertRadius(int direction);

    static const char* _cardinal(double bearing);
    void _drawBearingArrow(int cx, int cy, double bearing, uint16_t color);
    static const char* _motionText(const RemoteIdTrack& track,
                                   double homeLat, double homeLon);
    static double _directRangeMeters(double horizontalM, float heightM);

    const char* _stateText() const;
    static const char* _transportText(RemoteIdTransport t);
    static double _distanceMeters(double lat1, double lon1, double lat2, double lon2);
    static double _bearingDegrees(double lat1, double lon1, double lat2, double lon2);
};

} // namespace MOONCAKE::APPS
