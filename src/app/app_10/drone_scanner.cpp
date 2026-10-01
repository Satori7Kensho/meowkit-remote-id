/**
 * @file drone_scanner.cpp
 * @brief Phase-1 Drone Scanner app shell.
 *
 * This milestone only proves launcher/app lifecycle integration.  Remote ID
 * packet capture and OpenDroneID decoding are added in the next phase.
 */
#include "drone_scanner.h"
#include <Arduino.h>

namespace MOONCAKE::APPS
{

static constexpr uint16_t BG       = TFT_BLACK;
static constexpr uint16_t FG       = TFT_WHITE;
static constexpr uint16_t ACCENT   = 0x07E0; // green
static constexpr uint16_t DIM      = 0x7BEF; // grey
static constexpr uint16_t PANEL    = 0x1082; // very dark grey/green

DroneScanner::DroneScanner(DEVICES* device)
    : _device(device)
{
    setAppInfo().name = "Drone Scanner";
}

void DroneScanner::onOpen()
{
    _lastRenderMs = 0;
    _spinner = 0;

    _receiver.begin(_device);
    _drawStatic();
    _drawStatus();
}

void DroneScanner::onRunning()
{
    _receiver.update();

    const uint32_t now = millis();
    if ((now - _lastRenderMs) < 350) return;

    _lastRenderMs = now;
    _spinner = (_spinner + 1) & 0x03;
    _drawStatus();
}

void DroneScanner::onClose()
{
    _receiver.end();
    if (_device) _device->Lcd.fillScreen(TFT_BLACK);
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

void DroneScanner::_drawStatic()
{
    if (!_device) return;

    auto& lcd = _device->Lcd;
    lcd.fillScreen(BG);

    lcd.fillRect(0, 0, 320, 34, PANEL);
    lcd.setTextColor(ACCENT, PANEL);
    lcd.setTextFont(2);
    lcd.drawString("DRONE SCANNER", 10, 8);

    lcd.setTextColor(DIM, PANEL);
    lcd.drawRightString("Remote ID", 310, 8);

    lcd.drawFastHLine(0, 35, 320, ACCENT);

    lcd.setTextColor(FG, BG);
    lcd.setTextFont(2);
    lcd.drawString("Passive ASTM / FAA Remote ID receiver", 10, 50);

    lcd.setTextColor(DIM, BG);
    lcd.drawString("Wi-Fi + Bluetooth backend", 10, 72);

    lcd.drawRoundRect(10, 103, 300, 82, 6, ACCENT);

    lcd.setTextColor(DIM, BG);
    lcd.drawString("Status", 22, 114);
    lcd.drawString("Nearby drones", 22, 146);

    lcd.drawFastHLine(0, 215, 320, 0x3186);
    lcd.setTextColor(DIM, BG);
    lcd.drawString("Hold B to exit", 10, 220);
}

void DroneScanner::_drawStatus()
{
    if (!_device) return;

    auto& lcd = _device->Lcd;

    lcd.fillRect(146, 110, 150, 62, BG);

    char spin = '|';
    switch (_spinner) {
    case 0: spin = '|'; break;
    case 1: spin = '/'; break;
    case 2: spin = '-'; break;
    case 3: spin = '\\'; break;
    }

    lcd.setTextFont(2);
    lcd.setTextColor(ACCENT, BG);

    char line[40];
    snprintf(line, sizeof(line), "%s  %c", _stateText(), spin);
    lcd.drawRightString(line, 292, 114);

    lcd.setTextColor(FG, BG);
    snprintf(line, sizeof(line), "%u", static_cast<unsigned>(_receiver.count()));
    lcd.drawRightString(line, 292, 146);

    lcd.fillRect(10, 190, 300, 18, BG);
    lcd.setTextColor(DIM, BG);
    lcd.drawString("Phase 1: launcher + UI validation", 10, 190);
}

} // namespace MOONCAKE::APPS
