/**
 * @file remote_id_receiver.h
 * @brief Passive Wi-Fi Remote ID receiver for the Drone Scanner app.
 */
#pragma once

#include "remote_id_types.h"
#include "remote_id_decoder.h"
#include "remote_id_exporter.h"
#include "../../bsp/devices.h"
#include <cstddef>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <esp_wifi.h>
#include <esp_gap_ble_api.h>

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
    static constexpr size_t RAW_FRAME_MAX = 512;
    static constexpr size_t RAW_QUEUE_LEN = 12;
    static constexpr uint32_t TRACK_TTL_MS = 120000;
    static constexpr uint32_t CHANNEL_DWELL_MS = 280;

    struct RawFrame {
        uint16_t length = 0;
        int8_t rssi = 0;
        uint8_t channel = 0;
        uint8_t bytes[RAW_FRAME_MAX] = {0};
    };

    struct BleFrame {
        uint16_t length = 0;
        int8_t rssi = 0;
        bool extended = false;
        uint8_t mac[6] = {0};
        uint8_t bytes[255] = {0};
    };

    DEVICES* _device = nullptr;
    State _state = State::Idle;
    RemoteIdTrack _tracks[REMOTE_ID_MAX_TRACKS] = {};
    std::size_t _count = 0;
    RemoteIdExporter _exporter;

    QueueHandle_t _frameQueue = nullptr;
    QueueHandle_t _bleQueue = nullptr;
    bool _bleInited = false;
    bool _restoreConnection = false;
    uint8_t _channel = 1;
    uint32_t _lastChannelHopMs = 0;

    static RemoteIdReceiver* s_active;
    static void _promiscuousCallback(void* buffer, wifi_promiscuous_pkt_type_t type);
    static void _bleGapCallback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);

    bool _startBle();
    void _stopBle();
    void _processBleFrame(const BleFrame& frame);
    static bool _extractBleRemoteId(const uint8_t* adv, size_t advLen,
                                    const uint8_t*& msg, size_t& msgLen);

    void _processFrame(const RawFrame& frame);
    bool _decodeBeacon(const RawFrame& frame, RemoteIdTrack& out);
    bool _decodeNan(const RawFrame& frame, RemoteIdTrack& out);
    void _mergeTrack(const RemoteIdTrack& incoming);
    void _expireTracks(uint32_t nowMs);
    void _hopChannel(uint32_t nowMs);
};

} // namespace MOONCAKE::APPS
