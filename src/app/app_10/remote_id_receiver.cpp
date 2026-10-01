/**
 * @file remote_id_receiver.cpp
 * @brief Passive Wi-Fi Remote ID capture for MeowKit Drone Scanner.
 */
#include "remote_id_receiver.h"

#include <Arduino.h>
#include <WiFi.h>
#include <cstring>

extern "C" {
#include <opendroneid.h>
}

namespace MOONCAKE::APPS
{

RemoteIdReceiver* RemoteIdReceiver::s_active = nullptr;

bool RemoteIdReceiver::begin(DEVICES* device)
{
    if (!device) return false;

    end();

    _device = device;
    _state = State::Starting;
    _count = 0;
    for (auto& t : _tracks) t = RemoteIdTrack{};

    _frameQueue = xQueueCreate(RAW_QUEUE_LEN, sizeof(RawFrame));
    if (!_frameQueue) {
        _state = State::Error;
        return false;
    }

    _restoreConnection = _device->wifi.isConnected();
    if (_restoreConnection)
        _device->wifi.disconnect();

    // Arduino owns ESP-IDF Wi-Fi driver initialization.  STA mode keeps the
    // driver alive while we use passive promiscuous capture.
    WiFi.mode(WIFI_STA);
    delay(50);

    s_active = this;

    if (esp_wifi_set_promiscuous_rx_cb(&_promiscuousCallback) != ESP_OK ||
        esp_wifi_set_promiscuous(true) != ESP_OK) {
        s_active = nullptr;
        vQueueDelete(_frameQueue);
        _frameQueue = nullptr;
        _state = State::Error;
        return false;
    }

    _channel = 1;
    esp_wifi_set_channel(_channel, WIFI_SECOND_CHAN_NONE);
    _lastChannelHopMs = millis();

    _state = State::Listening;
    Serial.println("[DroneScanner] Wi-Fi Remote ID listening started");
    return true;
}

void RemoteIdReceiver::update()
{
    if (_state != State::Listening) return;

    const uint32_t now = millis();

    RawFrame frame{};
    int processed = 0;
    while (_frameQueue && xQueueReceive(_frameQueue, &frame, 0) == pdTRUE) {
        _processFrame(frame);
        if (++processed >= 8) break; // keep UI responsive under heavy RF traffic
    }

    _expireTracks(now);
    _hopChannel(now);
}

void RemoteIdReceiver::end()
{
    if (_state == State::Idle && !_frameQueue && !_device) return;

    if (s_active == this) s_active = nullptr;

    esp_wifi_set_promiscuous(false);
    esp_wifi_set_promiscuous_rx_cb(nullptr);

    if (_frameQueue) {
        vQueueDelete(_frameQueue);
        _frameQueue = nullptr;
    }

    if (_device) {
        WiFi.mode(WIFI_STA);
        if (_restoreConnection)
            _device->wifi.reconnect();
    }

    _restoreConnection = false;
    _device = nullptr;
    _count = 0;
    _state = State::Idle;

    Serial.println("[DroneScanner] Wi-Fi Remote ID listening stopped");
}

const RemoteIdTrack* RemoteIdReceiver::track(std::size_t index) const
{
    if (index >= _count || index >= REMOTE_ID_MAX_TRACKS) return nullptr;
    return &_tracks[index];
}

void RemoteIdReceiver::_promiscuousCallback(void* buffer, wifi_promiscuous_pkt_type_t type)
{
    if (!s_active || !s_active->_frameQueue || !buffer) return;
    if (type != WIFI_PKT_MGMT) return;

    const auto* packet = static_cast<const wifi_promiscuous_pkt_t*>(buffer);
    if (!packet) return;

    const uint16_t len = packet->rx_ctrl.sig_len;
    if (len < 24 || len > RAW_FRAME_MAX) return;

    RawFrame out{};
    out.length = len;
    out.rssi = packet->rx_ctrl.rssi;
    out.channel = packet->rx_ctrl.channel;
    std::memcpy(out.bytes, packet->payload, len);

    // Non-blocking by design. Dropping a packet is preferable to blocking the
    // Wi-Fi driver task; Remote ID broadcasts repeat frequently.
    xQueueSend(s_active->_frameQueue, &out, 0);
}

void RemoteIdReceiver::_processFrame(const RawFrame& frame)
{
    RemoteIdTrack decoded{};

    if (_decodeNan(frame, decoded) || _decodeBeacon(frame, decoded))
        _mergeTrack(decoded);
}

bool RemoteIdReceiver::_decodeNan(const RawFrame& frame, RemoteIdTrack& out)
{
    if (frame.length < 24) return false;

    // NAN Remote ID action frames are fully validated by the OpenDroneID helper.
    ODID_UAS_Data uas{};
    uint8_t mac[6] = {0};

    odid_initUasData(&uas);
    if (odid_wifi_receive_message_pack_nan_action_frame(
            &uas, reinterpret_cast<char*>(mac), frame.bytes, frame.length) != 0)
        return false;

    return RemoteIdDecoder::fromUasData(
        out, uas, mac, frame.rssi, RemoteIdTransport::WiFiNaN, millis());
}

bool RemoteIdReceiver::_decodeBeacon(const RawFrame& frame, RemoteIdTrack& out)
{
    // 802.11 beacon: 24-byte management header + 12-byte fixed parameters.
    if (frame.length < 38) return false;
    if ((frame.bytes[0] & 0xF0) != 0x80) return false;

    const uint8_t* mac = &frame.bytes[10]; // transmitter/source address
    size_t offset = 36;

    while (offset + 2 <= frame.length) {
        const uint8_t id  = frame.bytes[offset];
        const uint8_t len = frame.bytes[offset + 1];
        const size_t next = offset + 2u + len;
        if (next > frame.length) break;

        if (id == 0xDD && len >= 5) {
            const uint8_t* value = &frame.bytes[offset + 2];

            // ASTM Remote ID vendor IE:
            // OUI FA:0B:BC, OUI type 0x0D, then one-byte message counter,
            // followed by an OpenDroneID message pack.
            if (value[0] == 0xFA && value[1] == 0x0B && value[2] == 0xBC &&
                value[3] == 0x0D && len > 5) {
                const uint8_t* pack = value + 5;
                const size_t packLen = len - 5;

                return RemoteIdDecoder::decodeMessagePack(
                    out, pack, packLen, mac, frame.rssi,
                    RemoteIdTransport::WiFiBeacon, millis());
            }
        }

        offset = next;
    }

    return false;
}

void RemoteIdReceiver::_mergeTrack(const RemoteIdTrack& incoming)
{
    // Prefer MAC address as the transport-level identity. If it changes, fall
    // back to Basic ID when available.
    size_t found = REMOTE_ID_MAX_TRACKS;

    for (size_t i = 0; i < _count; ++i) {
        if (std::memcmp(_tracks[i].mac, incoming.mac, 6) == 0) {
            found = i;
            break;
        }
        if (incoming.uasId[0] && _tracks[i].uasId[0] &&
            std::strncmp(_tracks[i].uasId, incoming.uasId, REMOTE_ID_ID_LEN) == 0) {
            found = i;
            break;
        }
    }

    if (found < _count) {
        _tracks[found] = incoming;
        return;
    }

    if (_count < REMOTE_ID_MAX_TRACKS) {
        _tracks[_count++] = incoming;
        return;
    }

    // Table full: replace the stalest track.
    size_t oldest = 0;
    for (size_t i = 1; i < _count; ++i) {
        if (_tracks[i].lastSeenMs < _tracks[oldest].lastSeenMs)
            oldest = i;
    }
    _tracks[oldest] = incoming;
}

void RemoteIdReceiver::_expireTracks(uint32_t nowMs)
{
    size_t write = 0;
    for (size_t read = 0; read < _count; ++read) {
        if ((uint32_t)(nowMs - _tracks[read].lastSeenMs) <= TRACK_TTL_MS) {
            if (write != read) _tracks[write] = _tracks[read];
            ++write;
        }
    }

    for (size_t i = write; i < _count; ++i)
        _tracks[i] = RemoteIdTrack{};

    _count = write;
}

void RemoteIdReceiver::_hopChannel(uint32_t nowMs)
{
    if ((uint32_t)(nowMs - _lastChannelHopMs) < CHANNEL_DWELL_MS) return;

    _lastChannelHopMs = nowMs;

    // Cycle all 2.4 GHz channels supported by the ESP32-S3. Regulatory-domain
    // enforcement remains in the ESP-IDF driver; unsupported channels fail
    // harmlessly and are skipped on the next dwell.
    ++_channel;
    if (_channel > 13) _channel = 1;
    esp_wifi_set_channel(_channel, WIFI_SECOND_CHAN_NONE);
}

} // namespace MOONCAKE::APPS
