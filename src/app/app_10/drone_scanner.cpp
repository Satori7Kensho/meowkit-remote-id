/**
 * @file drone_scanner.cpp
 * @brief Drone Scanner UI: scan status, nearby list/details, and Drone Radar.
 */
#include "drone_scanner.h"

#include <Arduino.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <Preferences.h>

namespace MOONCAKE::APPS
{

static constexpr uint16_t BG      = TFT_BLACK;
static constexpr uint16_t FG      = TFT_WHITE;
static constexpr uint16_t ACCENT  = 0x07E0;
static constexpr uint16_t DIM     = 0x7BEF;
static constexpr uint16_t PANEL   = 0x1082;
static constexpr uint16_t GRID    = 0x3186;

DroneScanner::DroneScanner(DEVICES* device)
    : _device(device)
{
    setAppInfo().name = "Drone Scanner";
}

void DroneScanner::onOpen()
{
    _lastRenderMs = 0;
    _spinner = 0;
    _selected = 0;
    _lastCount = static_cast<size_t>(-1);
    _page = Page::Scan;
    _loadHomeLocation();

    _receiver.begin(_device);
    _drawScan();
}

void DroneScanner::onRunning()
{
    _receiver.update();
    _handleInput();

    const uint32_t now = millis();
    const size_t count = _receiver.count();

    if ((now - _lastRenderMs) < 250 && count == _lastCount) return;

    _lastRenderMs = now;
    _lastCount = count;
    _spinner = (_spinner + 1) & 0x03;

    if (_selected >= count && count > 0) _selected = count - 1;
    if (count == 0) _selected = 0;

    switch (_page) {
    case Page::Scan:      _drawScan(); break;
    case Page::Nearby:    _drawNearby(); break;
    case Page::Radar:     _drawRadar(); break;
    case Page::HomeSetup: _drawHomeSetup(); break;
    }
}

void DroneScanner::onClose()
{
    _receiver.end();
    if (_device) _device->Lcd.fillScreen(TFT_BLACK);
}

void DroneScanner::_handleInput()
{
    if (!_device) return;

    _device->button.update();
    _device->button.tick();

    if (_page == Page::HomeSetup) {
        if (_device->button.Up.pressed()) {
            if (_homeEditField == HomeEditField::Latitude)
                _homeEditField = HomeEditField::Save;
            else
                _homeEditField = static_cast<HomeEditField>(
                    static_cast<uint8_t>(_homeEditField) - 1);
            _drawHomeSetup();
        }
        if (_device->button.Down.pressed()) {
            if (_homeEditField == HomeEditField::Save)
                _homeEditField = HomeEditField::Latitude;
            else
                _homeEditField = static_cast<HomeEditField>(
                    static_cast<uint8_t>(_homeEditField) + 1);
            _drawHomeSetup();
        }

        if (_homeEditField != HomeEditField::Save) {
            if (_device->button.Left.pressed()) {
                _adjustHome(-1);
                _drawHomeSetup();
            }
            if (_device->button.Right.pressed()) {
                _adjustHome(+1);
                _drawHomeSetup();
            }
            if (_device->button.A.pressed()) {
                _homeStepIndex = (_homeStepIndex + 1) % 5;
                _drawHomeSetup();
            }
        } else if (_device->button.A.pressed()) {
            _saveHomeLocation();
            _switchPage(Page::Radar);
        }
        return;
    }

    if (_device->button.Left.pressed()) {
        if (_page == Page::Scan) _switchPage(Page::Radar);
        else _switchPage(static_cast<Page>(static_cast<uint8_t>(_page) - 1));
    }

    if (_device->button.Right.pressed()) {
        if (_page == Page::Radar) _switchPage(Page::Scan);
        else _switchPage(static_cast<Page>(static_cast<uint8_t>(_page) + 1));
    }

    if (_page == Page::Radar && _device->button.A.pressed()) {
        _homeEditField = HomeEditField::Latitude;
        _switchPage(Page::HomeSetup);
        return;
    }

    const size_t count = _receiver.count();
    if (_page == Page::Nearby && count > 0) {
        if (_device->button.Up.pressed()) {
            _selected = (_selected == 0) ? count - 1 : _selected - 1;
            _drawNearby();
        }
        if (_device->button.Down.pressed()) {
            _selected = (_selected + 1) % count;
            _drawNearby();
        }
    }
}

void DroneScanner::_switchPage(Page page)
{
    _page = page;
    switch (_page) {
    case Page::Scan:      _drawScan(); break;
    case Page::Nearby:    _drawNearby(); break;
    case Page::Radar:     _drawRadar(); break;
    case Page::HomeSetup: _drawHomeSetup(); break;
    }
}

const char* DroneScanner::_stateText() const
{
    switch (_receiver.state()) {
    case RemoteIdReceiver::State::Idle:      return "IDLE";
    case RemoteIdReceiver::State::Starting:  return "STARTING";
    case RemoteIdReceiver::State::Listening: return "LISTENING";
    case RemoteIdReceiver::State::Error:     return "ERROR";
    }
    return "UNKNOWN";
}

const char* DroneScanner::_transportText(RemoteIdTransport t)
{
    switch (t) {
    case RemoteIdTransport::WiFiBeacon:  return "WiFi Beacon";
    case RemoteIdTransport::WiFiNaN:     return "WiFi NAN";
    case RemoteIdTransport::BleLegacy:   return "BLE";
    case RemoteIdTransport::BleExtended: return "BLE 5";
    default:                             return "Unknown";
    }
}

void DroneScanner::_drawFrame(const char* title)
{
    auto& lcd = _device->Lcd;
    lcd.fillScreen(BG);

    lcd.fillRect(0, 0, 320, 30, PANEL);
    lcd.setTextFont(2);
    lcd.setTextColor(ACCENT, PANEL);
    lcd.drawString(title, 8, 7);
    lcd.setTextColor(DIM, PANEL);
    lcd.drawRightString("Remote ID", 312, 7);
    lcd.drawFastHLine(0, 31, 320, ACCENT);
}

void DroneScanner::_drawFooter(const char* hint)
{
    auto& lcd = _device->Lcd;
    lcd.fillRect(0, 214, 320, 26, BG);
    lcd.drawFastHLine(0, 214, 320, GRID);
    lcd.setTextFont(1);
    lcd.setTextColor(DIM, BG);
    lcd.drawString("< > pages", 8, 220);
    lcd.drawRightString(hint ? hint : "Hold B: exit", 312, 220);
}

void DroneScanner::_drawScan()
{
    _drawFrame("DRONE SCANNER");
    auto& lcd = _device->Lcd;

    char spin = '|';
    switch (_spinner) {
    case 0: spin = '|'; break;
    case 1: spin = '/'; break;
    case 2: spin = '-'; break;
    case 3: spin = '\\'; break;
    }

    lcd.setTextFont(2);
    lcd.setTextColor(FG, BG);
    lcd.drawString("Passive Wi-Fi Remote ID", 12, 48);

    lcd.setTextColor(DIM, BG);
    lcd.drawString("ASTM / FAA compatible broadcasts", 12, 70);

    lcd.drawRoundRect(12, 100, 296, 76, 5, ACCENT);
    lcd.setTextColor(DIM, BG);
    lcd.drawString("Status", 24, 111);
    lcd.drawString("Nearby drones", 24, 143);

    char buf[40];
    snprintf(buf, sizeof(buf), "%s  %c", _stateText(), spin);
    lcd.setTextColor(ACCENT, BG);
    lcd.drawRightString(buf, 294, 111);

    snprintf(buf, sizeof(buf), "%u", static_cast<unsigned>(_receiver.count()));
    lcd.setTextColor(FG, BG);
    lcd.drawRightString(buf, 294, 143);

    _drawFooter("Hold B: exit");
}

void DroneScanner::_drawNearby()
{
    _drawFrame("NEARBY DRONES");
    auto& lcd = _device->Lcd;
    const size_t count = _receiver.count();

    if (count == 0) {
        lcd.setTextFont(2);
        lcd.setTextColor(DIM, BG);
        lcd.drawCentreString("No Remote ID drones detected", 160, 96);
        lcd.drawCentreString("Scanner is still listening", 160, 120);
        _drawFooter("Hold B: exit");
        return;
    }

    const RemoteIdTrack* t = _receiver.track(_selected);
    if (!t) {
        _drawFooter("Hold B: exit");
        return;
    }

    lcd.setTextFont(2);
    lcd.setTextColor(ACCENT, BG);

    char header[32];
    snprintf(header, sizeof(header), "%u / %u",
             static_cast<unsigned>(_selected + 1),
             static_cast<unsigned>(count));
    lcd.drawRightString(header, 306, 42);

    lcd.setTextColor(FG, BG);
    const char* id = t->uasId[0] ? t->uasId : "(ID unavailable)";
    lcd.drawString(id, 12, 42);

    lcd.setTextColor(DIM, BG);
    lcd.drawString(_transportText(t->transport), 12, 64);

    char buf[64];
    snprintf(buf, sizeof(buf), "RSSI  %d dBm", static_cast<int>(t->rssi));
    lcd.drawString(buf, 12, 88);

    if (t->hasLocation) {
        snprintf(buf, sizeof(buf), "LAT   %.6f", t->latitude);
        lcd.drawString(buf, 12, 110);
        snprintf(buf, sizeof(buf), "LON   %.6f", t->longitude);
        lcd.drawString(buf, 12, 130);
        snprintf(buf, sizeof(buf), "ALT   %.1f m", t->altitudeMslM);
        lcd.drawString(buf, 12, 150);
        snprintf(buf, sizeof(buf), "SPD   %.1f m/s   HDG %.0f", t->speedMps, t->headingDeg);
        lcd.drawString(buf, 12, 170);
    } else {
        lcd.drawString("Position not yet received", 12, 118);
    }

    _drawFooter("Up/Down: select");
}

void DroneScanner::_drawRadar()
{
    _drawFrame("DRONE RADAR");
    auto& lcd = _device->Lcd;

    const int cx = 160;
    const int cy = 124;
    const int r1 = 25;
    const int r2 = 50;
    const int r3 = 76;

    lcd.drawCircle(cx, cy, r1, GRID);
    lcd.drawCircle(cx, cy, r2, GRID);
    lcd.drawCircle(cx, cy, r3, ACCENT);
    lcd.drawFastHLine(cx - r3, cy, r3 * 2, GRID);
    lcd.drawFastVLine(cx, cy - r3, r3 * 2, GRID);
    lcd.fillCircle(cx, cy, 3, FG);

    lcd.setTextFont(1);
    lcd.setTextColor(DIM, BG);
    lcd.drawCentreString(_homeConfigured ? "HOME" : "RID ORIGIN", cx, cy + 4);

    bool plotted = false;
    double maxDistance = 1.0;

    for (size_t i = 0; i < _receiver.count(); ++i) {
        const RemoteIdTrack* t = _receiver.track(i);
        if (!t || !t->hasLocation) continue;

        double originLat = 0.0;
        double originLon = 0.0;
        bool haveOrigin = false;

        if (_homeConfigured) {
            originLat = _homeLat;
            originLon = _homeLon;
            haveOrigin = true;
        } else if (t->hasOperatorLocation) {
            originLat = t->operatorLatitude;
            originLon = t->operatorLongitude;
            haveOrigin = true;
        }

        if (!haveOrigin) continue;

        const double d = _distanceMeters(
            originLat, originLon, t->latitude, t->longitude);
        if (d > maxDistance) maxDistance = d;
    }

    maxDistance *= 1.2;
    if (maxDistance < 50.0) maxDistance = 50.0;

    for (size_t i = 0; i < _receiver.count(); ++i) {
        const RemoteIdTrack* t = _receiver.track(i);
        if (!t || !t->hasLocation) continue;

        double originLat = 0.0;
        double originLon = 0.0;
        bool haveOrigin = false;

        if (_homeConfigured) {
            originLat = _homeLat;
            originLon = _homeLon;
            haveOrigin = true;
        } else if (t->hasOperatorLocation) {
            originLat = t->operatorLatitude;
            originLon = t->operatorLongitude;
            haveOrigin = true;
        }

        if (!haveOrigin) continue;

        const double d = _distanceMeters(
            originLat, originLon, t->latitude, t->longitude);
        const double bearing = _bearingDegrees(
            originLat, originLon, t->latitude, t->longitude);

        const double angle = (bearing - 90.0) * M_PI / 180.0;
        const double radius = (d / maxDistance) * static_cast<double>(r3 - 7);

        const int x = cx + static_cast<int>(std::cos(angle) * radius);
        const int y = cy + static_cast<int>(std::sin(angle) * radius);

        lcd.fillCircle(x, y, 4, ACCENT);
        lcd.drawCircle(x, y, 5, FG);
        plotted = true;
    }

    lcd.setTextFont(1);
    lcd.setTextColor(DIM, BG);
    if (plotted) {
        char scale[48];
        snprintf(scale, sizeof(scale), "Outer ring ~ %.0f m", maxDistance);
        lcd.drawString(scale, 8, 198);
    } else if (_homeConfigured) {
        lcd.drawCentreString("Home set - waiting for drone position", 160, 198);
    } else {
        lcd.drawCentreString("A: set Home for property-relative radar", 160, 198);
    }

    _drawFooter(_homeConfigured ? "A: edit Home   Hold B: exit"
                                : "A: set Home   Hold B: exit");
}

void DroneScanner::_loadHomeLocation()
{
    Preferences prefs;
    if (!prefs.begin("drone_radar", true)) {
        _homeConfigured = false;
        return;
    }

    _homeConfigured = prefs.getBool("home_set", false);
    _homeLat = prefs.getDouble("home_lat", 0.0);
    _homeLon = prefs.getDouble("home_lon", 0.0);
    prefs.end();

    if (_homeLat < -90.0 || _homeLat > 90.0 ||
        _homeLon < -180.0 || _homeLon > 180.0) {
        _homeConfigured = false;
        _homeLat = 0.0;
        _homeLon = 0.0;
    }
}

void DroneScanner::_saveHomeLocation()
{
    if (_homeLat < -90.0) _homeLat = -90.0;
    if (_homeLat > 90.0) _homeLat = 90.0;
    if (_homeLon < -180.0) _homeLon = -180.0;
    if (_homeLon > 180.0) _homeLon = 180.0;

    Preferences prefs;
    if (!prefs.begin("drone_radar", false)) return;

    prefs.putDouble("home_lat", _homeLat);
    prefs.putDouble("home_lon", _homeLon);
    prefs.putBool("home_set", true);
    prefs.end();

    _homeConfigured = true;
    Serial.printf("[DroneScanner] Home location saved: %.6f, %.6f\n",
                  _homeLat, _homeLon);
}

void DroneScanner::_adjustHome(int direction)
{
    static const double steps[] = {1.0, 0.1, 0.01, 0.001, 0.0001};
    const double delta = steps[_homeStepIndex] * static_cast<double>(direction);

    if (_homeEditField == HomeEditField::Latitude) {
        _homeLat += delta;
        if (_homeLat > 90.0) _homeLat = 90.0;
        if (_homeLat < -90.0) _homeLat = -90.0;
    } else if (_homeEditField == HomeEditField::Longitude) {
        _homeLon += delta;
        if (_homeLon > 180.0) _homeLon = 180.0;
        if (_homeLon < -180.0) _homeLon = -180.0;
    }
}

void DroneScanner::_drawHomeSetup()
{
    _drawFrame("HOME / RECEIVER LOCATION");
    auto& lcd = _device->Lcd;

    static const double steps[] = {1.0, 0.1, 0.01, 0.001, 0.0001};

    lcd.setTextFont(1);
    lcd.setTextColor(DIM, BG);
    lcd.drawString("Used as center of Drone Radar.", 10, 40);
    lcd.drawString("Stored locally on this MeowKit.", 10, 54);

    char buf[64];

    const bool latSel = _homeEditField == HomeEditField::Latitude;
    const bool lonSel = _homeEditField == HomeEditField::Longitude;
    const bool saveSel = _homeEditField == HomeEditField::Save;

    lcd.setTextFont(2);
    lcd.setTextColor(latSel ? ACCENT : FG, BG);
    snprintf(buf, sizeof(buf), "%c LAT  %.6f", latSel ? '>' : ' ', _homeLat);
    lcd.drawString(buf, 18, 82);

    lcd.setTextColor(lonSel ? ACCENT : FG, BG);
    snprintf(buf, sizeof(buf), "%c LON  %.6f", lonSel ? '>' : ' ', _homeLon);
    lcd.drawString(buf, 18, 108);

    lcd.setTextColor(DIM, BG);
    snprintf(buf, sizeof(buf), "Step: %.4f deg   [A] change", steps[_homeStepIndex]);
    lcd.drawString(buf, 18, 138);

    lcd.setTextColor(saveSel ? ACCENT : FG, BG);
    lcd.drawString(saveSel ? "> SAVE HOME LOCATION" : "  SAVE HOME LOCATION", 18, 172);

    lcd.setTextFont(1);
    lcd.setTextColor(DIM, BG);
    lcd.drawString("Up/Down: field   Left/Right: adjust", 10, 198);

    _drawFooter(saveSel ? "A: save + return" : "A: change step");
}

double DroneScanner::_distanceMeters(double lat1, double lon1, double lat2, double lon2)
{
    constexpr double R = 6371000.0;
    const double p1 = lat1 * M_PI / 180.0;
    const double p2 = lat2 * M_PI / 180.0;
    const double dp = (lat2 - lat1) * M_PI / 180.0;
    const double dl = (lon2 - lon1) * M_PI / 180.0;

    const double a = std::sin(dp / 2.0) * std::sin(dp / 2.0) +
                     std::cos(p1) * std::cos(p2) *
                     std::sin(dl / 2.0) * std::sin(dl / 2.0);
    return R * 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
}

double DroneScanner::_bearingDegrees(double lat1, double lon1, double lat2, double lon2)
{
    const double p1 = lat1 * M_PI / 180.0;
    const double p2 = lat2 * M_PI / 180.0;
    const double dl = (lon2 - lon1) * M_PI / 180.0;

    const double y = std::sin(dl) * std::cos(p2);
    const double x = std::cos(p1) * std::sin(p2) -
                     std::sin(p1) * std::cos(p2) * std::cos(dl);

    double b = std::atan2(y, x) * 180.0 / M_PI;
    if (b < 0.0) b += 360.0;
    return b;
}

} // namespace MOONCAKE::APPS
