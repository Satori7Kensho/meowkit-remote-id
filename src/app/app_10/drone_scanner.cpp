/**
 * @file drone_scanner.cpp
 * @brief Drone Scanner UI: scan status, nearby list/details, and Drone Radar.
 */
#include "drone_scanner.h"

#include <Arduino.h>
#include <algorithm>
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
    _receiver.setHomeZone(_homeConfigured, _homeLat, _homeLon, _homeAlertRadiusM);
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
    case Page::Radar:       _drawRadar(); break;
    case Page::Diagnostics: _drawDiagnostics(); break;
    case Page::HomeSetup:   _drawHomeSetup(); break;
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
                if (_homeEditField == HomeEditField::AlertRadius) _adjustAlertRadius(-1);
                else _adjustHome(-1);
                _drawHomeSetup();
            }
            if (_device->button.Right.pressed()) {
                if (_homeEditField == HomeEditField::AlertRadius) _adjustAlertRadius(+1);
                else _adjustHome(+1);
                _drawHomeSetup();
            }
            if (_device->button.A.pressed() && _homeEditField != HomeEditField::AlertRadius) {
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
        if (_page == Page::Scan) _switchPage(Page::Diagnostics);
        else _switchPage(static_cast<Page>(static_cast<uint8_t>(_page) - 1));
    }

    if (_device->button.Right.pressed()) {
        if (_page == Page::Diagnostics) _switchPage(Page::Scan);
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
    case Page::Scan:        _drawScan(); break;
    case Page::Nearby:      _drawNearby(); break;
    case Page::Radar:       _drawRadar(); break;
    case Page::Diagnostics: _drawDiagnostics(); break;
    case Page::HomeSetup:   _drawHomeSetup(); break;
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

    if (_homeConfigured && _receiver.count() > 0) {
        const RemoteIdTrack* closest = nullptr;
        size_t closestIndex = 0;
        double closestM = 1e30;

        for (size_t i = 0; i < _receiver.count(); ++i) {
            const RemoteIdTrack* t = _receiver.track(i);
            if (!t || !t->hasLocation) continue;
            const double d = _distanceMeters(_homeLat, _homeLon, t->latitude, t->longitude);
            if (d < closestM) {
                closestM = d;
                closest = t;
                closestIndex = i;
            }
        }

        if (closest) {
            const double b = _bearingDegrees(_homeLat, _homeLon,
                                             closest->latitude, closest->longitude);
            lcd.setTextFont(1);
            lcd.setTextColor(closestM <= _homeAlertRadiusM ? TFT_ORANGE : ACCENT, BG);
            snprintf(buf, sizeof(buf), "Closest D%u: %.0fm %s | %s",
                     static_cast<unsigned>(closestIndex + 1), closestM,
                     _cardinal(b), _motionText(*closest, _homeLat, _homeLon));
            lcd.drawString(buf, 12, 190);
        }
    }

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
        snprintf(buf, sizeof(buf), "ALT %.0fm   SPD %.1fm/s   HDG %.0f",
                 t->altitudeMslM, t->speedMps, t->headingDeg);
        lcd.drawString(buf, 12, 108);

        lcd.setTextFont(1);
        lcd.setTextColor(DIM, BG);
        snprintf(buf, sizeof(buf), "%.6f, %.6f", t->latitude, t->longitude);
        lcd.drawString(buf, 12, 132);

        if (_homeConfigured) {
            const double d = _distanceMeters(_homeLat, _homeLon, t->latitude, t->longitude);
            const double b = _bearingDegrees(_homeLat, _homeLon, t->latitude, t->longitude);

            lcd.setTextFont(2);
            lcd.setTextColor(ACCENT, BG);
            snprintf(buf, sizeof(buf), "HOME %.0fm  %s  BRG %.0f",
                     d, _cardinal(b), b);
            lcd.drawString(buf, 12, 151);

            lcd.setTextFont(1);
            lcd.setTextColor(FG, BG);

            if (t->heightAglM > 0.0f && t->heightAglM < 5000.0f) {
                const double direct = _directRangeMeters(d, t->heightAglM);
                snprintf(buf, sizeof(buf), "Direct range ~%.0fm (RID height)", direct);
                lcd.drawString(buf, 12, 176);
            }

            snprintf(buf, sizeof(buf), "%s%s",
                     d <= 100.0 ? "OVERHEAD VICINITY  |  " : "",
                     _motionText(*t, _homeLat, _homeLon));
            lcd.setTextColor(d <= _homeAlertRadiusM ? TFT_ORANGE : DIM, BG);
            lcd.drawString(buf, 12, 194);
        }
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
    const int cy = 123;
    const int outerPx = 73;

    double farthest = 0.0;
    for (size_t i = 0; i < _receiver.count(); ++i) {
        const RemoteIdTrack* t = _receiver.track(i);
        if (!t || !t->hasLocation) continue;

        double oLat = 0.0, oLon = 0.0;
        bool haveOrigin = false;
        if (_homeConfigured) {
            oLat = _homeLat; oLon = _homeLon; haveOrigin = true;
        } else if (t->hasOperatorLocation) {
            oLat = t->operatorLatitude; oLon = t->operatorLongitude; haveOrigin = true;
        }
        if (!haveOrigin) continue;

        const double d = _distanceMeters(oLat, oLon, t->latitude, t->longitude);
        if (d > farthest) farthest = d;
    }

    // Stable, human-friendly radar scales rather than a continuously resizing plot.
    static const double SCALES[] = {100, 250, 500, 1000, 2000, 5000, 10000};
    double outerM = 100.0;
    const double needed = std::max(farthest * 1.15,
                                   _homeConfigured ? static_cast<double>(_homeAlertRadiusM) * 1.15 : 0.0);
    for (double s : SCALES) {
        outerM = s;
        if (s >= needed) break;
    }

    // Three distance rings plus compass axes.
    lcd.drawCircle(cx, cy, outerPx / 3, GRID);
    lcd.drawCircle(cx, cy, (outerPx * 2) / 3, GRID);
    lcd.drawCircle(cx, cy, outerPx, ACCENT);
    lcd.drawFastHLine(cx - outerPx, cy, outerPx * 2, GRID);
    lcd.drawFastVLine(cx, cy - outerPx, outerPx * 2, GRID);

    lcd.setTextFont(1);
    lcd.setTextColor(DIM, BG);
    lcd.drawCentreString("N", cx, 38);
    lcd.drawCentreString("S", cx, 198);
    lcd.drawString("W", 75, cy - 4);
    lcd.drawRightString("E", 245, cy - 4);

    char ring[20];
    snprintf(ring, sizeof(ring), "%.0fm", outerM / 3.0);
    lcd.drawString(ring, cx + 2, cy - outerPx / 3 - 7);
    snprintf(ring, sizeof(ring), "%.0fm", outerM * 2.0 / 3.0);
    lcd.drawString(ring, cx + 2, cy - (outerPx * 2) / 3 - 7);
    snprintf(ring, sizeof(ring), "%.0fm", outerM);
    lcd.drawString(ring, cx + 2, cy - outerPx - 7);

    if (_homeConfigured && _homeAlertRadiusM < outerM) {
        const int alertPx = std::max(4, static_cast<int>(
            (_homeAlertRadiusM / outerM) * static_cast<double>(outerPx)));
        lcd.drawCircle(cx, cy, alertPx, TFT_ORANGE);
    }

    lcd.fillCircle(cx, cy, 3, FG);
    lcd.setTextColor(_homeConfigured ? ACCENT : DIM, BG);
    lcd.drawCentreString(_homeConfigured ? "HOME" : "RID ORIGIN", cx, cy + 4);

    bool plotted = false;

    for (size_t i = 0; i < _receiver.count(); ++i) {
        const RemoteIdTrack* t = _receiver.track(i);
        if (!t || !t->hasLocation) continue;

        double oLat = 0.0, oLon = 0.0;
        bool haveOrigin = false;
        if (_homeConfigured) {
            oLat = _homeLat; oLon = _homeLon; haveOrigin = true;
        } else if (t->hasOperatorLocation) {
            oLat = t->operatorLatitude; oLon = t->operatorLongitude; haveOrigin = true;
        }
        if (!haveOrigin) continue;

        const double d = _distanceMeters(oLat, oLon, t->latitude, t->longitude);
        const double bearing = _bearingDegrees(oLat, oLon, t->latitude, t->longitude);
        const double angle = (bearing - 90.0) * M_PI / 180.0;
        const double radius = std::min(
            static_cast<double>(outerPx - 6),
            (d / outerM) * static_cast<double>(outerPx - 6));

        const int x = cx + static_cast<int>(std::cos(angle) * radius);
        const int y = cy + static_cast<int>(std::sin(angle) * radius);

        const uint16_t markerColor =
            (_homeConfigured && d <= _homeAlertRadiusM) ? TFT_ORANGE : ACCENT;

        lcd.fillCircle(x, y, 4, markerColor);
        lcd.drawCircle(x, y, 5, FG);

        // Heading tick: the dot tells where it is; this line tells where it is moving.
        const double hdgAngle = (static_cast<double>(t->headingDeg) - 90.0) * M_PI / 180.0;
        const int hx = x + static_cast<int>(std::cos(hdgAngle) * 11.0);
        const int hy = y + static_cast<int>(std::sin(hdgAngle) * 11.0);
        lcd.drawLine(x, y, hx, hy, FG);

        char label[8];
        snprintf(label, sizeof(label), "D%u", static_cast<unsigned>(i + 1));
        lcd.setTextColor(FG, BG);
        lcd.drawString(label, x + 6, y - 5);

        plotted = true;
    }

    lcd.setTextFont(1);
    if (plotted) {
        char status[64];
        snprintf(status, sizeof(status), "Outer %.0fm | alert %.0fm",
                 outerM, _homeConfigured ? _homeAlertRadiusM : 0.0f);
        lcd.setTextColor(DIM, BG);
        lcd.drawString(status, 8, 199);
    } else if (_homeConfigured) {
        lcd.setTextColor(DIM, BG);
        lcd.drawCentreString("Home set - waiting for drone position", 160, 199);
    } else {
        lcd.setTextColor(DIM, BG);
        lcd.drawCentreString("A: set Home for property-relative radar", 160, 199);
    }

    _drawFooter(_homeConfigured ? "A: edit Home   Hold B: exit"
                                : "A: set Home   Hold B: exit");
}

void DroneScanner::_drawDiagnostics()
{
    _drawFrame("DIAGNOSTICS");
    auto& lcd = _device->Lcd;

    lcd.setTextFont(2);

    char buf[64];

    lcd.setTextColor(DIM, BG);
    lcd.drawString("Receiver state", 12, 44);
    lcd.setTextColor(ACCENT, BG);
    lcd.drawRightString(_stateText(), 306, 44);

    lcd.setTextColor(DIM, BG);
    lcd.drawString("Wi-Fi channel", 12, 66);
    snprintf(buf, sizeof(buf), "%u", static_cast<unsigned>(_receiver.wifiChannel()));
    lcd.setTextColor(FG, BG);
    lcd.drawRightString(buf, 306, 66);

    lcd.setTextColor(DIM, BG);
    lcd.drawString("BLE scanner", 12, 88);
    lcd.setTextColor(_receiver.bleActive() ? ACCENT : FG, BG);
    lcd.drawRightString(_receiver.bleActive() ? "ACTIVE" : "OFF", 306, 88);

    lcd.setTextColor(DIM, BG);
    lcd.drawString("RID messages", 12, 110);
    snprintf(buf, sizeof(buf), "%lu",
             static_cast<unsigned long>(_receiver.remoteIdMessages()));
    lcd.setTextColor(FG, BG);
    lcd.drawRightString(buf, 306, 110);

    lcd.setTextColor(DIM, BG);
    lcd.drawString("Wi-Fi / BLE", 12, 132);
    snprintf(buf, sizeof(buf), "%lu / %lu",
             static_cast<unsigned long>(_receiver.wifiRemoteIdMessages()),
             static_cast<unsigned long>(_receiver.bleRemoteIdMessages()));
    lcd.setTextColor(FG, BG);
    lcd.drawRightString(buf, 306, 132);

    lcd.setTextColor(DIM, BG);
    lcd.drawString("Unique this session", 12, 154);
    snprintf(buf, sizeof(buf), "%lu",
             static_cast<unsigned long>(_receiver.sessionUniqueDrones()));
    lcd.setTextColor(FG, BG);
    lcd.drawRightString(buf, 306, 154);

    lcd.setTextColor(DIM, BG);
    lcd.drawString("Dropped RF frames", 12, 176);
    snprintf(buf, sizeof(buf), "%lu",
             static_cast<unsigned long>(_receiver.droppedFrames()));
    lcd.setTextColor(FG, BG);
    lcd.drawRightString(buf, 306, 176);

    _drawFooter("Useful for hardware testing");
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
    _homeAlertRadiusM = prefs.getFloat("alert_m", 250.0f);
    prefs.end();

    if (_homeLat < -90.0 || _homeLat > 90.0 ||
        _homeLon < -180.0 || _homeLon > 180.0) {
        _homeConfigured = false;
        _homeLat = 0.0;
        _homeLon = 0.0;
    }
    if (_homeAlertRadiusM < 25.0f || _homeAlertRadiusM > 5000.0f)
        _homeAlertRadiusM = 250.0f;
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
    prefs.putFloat("alert_m", _homeAlertRadiusM);
    prefs.putBool("home_set", true);
    prefs.end();

    _homeConfigured = true;
    _receiver.setHomeZone(true, _homeLat, _homeLon, _homeAlertRadiusM);
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
    const bool radiusSel = _homeEditField == HomeEditField::AlertRadius;
    const bool saveSel = _homeEditField == HomeEditField::Save;

    lcd.setTextFont(2);
    lcd.setTextColor(latSel ? ACCENT : FG, BG);
    snprintf(buf, sizeof(buf), "%c LAT  %.6f", latSel ? '>' : ' ', _homeLat);
    lcd.drawString(buf, 18, 76);

    lcd.setTextColor(lonSel ? ACCENT : FG, BG);
    snprintf(buf, sizeof(buf), "%c LON  %.6f", lonSel ? '>' : ' ', _homeLon);
    lcd.drawString(buf, 18, 100);

    lcd.setTextColor(radiusSel ? TFT_ORANGE : FG, BG);
    snprintf(buf, sizeof(buf), "%c ALERT RADIUS  %.0f m",
             radiusSel ? '>' : ' ', _homeAlertRadiusM);
    lcd.drawString(buf, 18, 124);

    lcd.setTextFont(1);
    lcd.setTextColor(DIM, BG);
    snprintf(buf, sizeof(buf), "Coordinate step: %.4f deg", steps[_homeStepIndex]);
    lcd.drawString(buf, 18, 150);

    lcd.setTextFont(2);
    lcd.setTextColor(saveSel ? ACCENT : FG, BG);
    lcd.drawString(saveSel ? "> SAVE HOME + ZONE" : "  SAVE HOME + ZONE", 18, 174);

    lcd.setTextFont(1);
    lcd.setTextColor(DIM, BG);
    lcd.drawString("Up/Down field | Left/Right adjust", 10, 198);

    _drawFooter(saveSel ? "A: save + return"
                        : (radiusSel ? "Left/Right: radius" : "A: coord step"));
}

void DroneScanner::_adjustAlertRadius(int direction)
{
    static const float RADII[] = {50, 100, 250, 500, 1000, 2000, 5000};
    int nearest = 0;
    float best = std::fabs(_homeAlertRadiusM - RADII[0]);
    for (int i = 1; i < 7; ++i) {
        const float delta = std::fabs(_homeAlertRadiusM - RADII[i]);
        if (delta < best) { best = delta; nearest = i; }
    }

    nearest += direction;
    if (nearest < 0) nearest = 0;
    if (nearest > 6) nearest = 6;
    _homeAlertRadiusM = RADII[nearest];
}

const char* DroneScanner::_cardinal(double bearing)
{
    static const char* DIRS[] = {"N","NE","E","SE","S","SW","W","NW"};
    int idx = static_cast<int>(std::floor((bearing + 22.5) / 45.0)) & 7;
    return DIRS[idx];
}

const char* DroneScanner::_motionText(const RemoteIdTrack& track,
                                     double homeLat, double homeLon)
{
    if (!track.hasPreviousLocation || !track.hasLocation)
        return "TRACKING";

    const double before = _distanceMeters(
        homeLat, homeLon, track.previousLatitude, track.previousLongitude);
    const double now = _distanceMeters(
        homeLat, homeLon, track.latitude, track.longitude);
    const double delta = now - before;

    // A few metres of dead-band avoids GPS/Remote-ID jitter being described
    // as meaningful motion toward or away from Home.
    if (delta < -3.0) return "CLOSING";
    if (delta >  3.0) return "DEPARTING";
    return "STEADY";
}

double DroneScanner::_directRangeMeters(double horizontalM, float heightM)
{
    const double h = static_cast<double>(heightM);
    return std::sqrt(horizontalM * horizontalM + h * h);
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
