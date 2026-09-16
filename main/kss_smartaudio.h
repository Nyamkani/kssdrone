#pragma once

#include <cstddef>
#include <cstdint>

#include "freertos/FreeRTOS.h"

#include "esp_err.h"

enum class Command : uint8_t
{
    GET_SETTINGS  = 0x03,
    SET_POWER     = 0x05,
    SET_CHANNEL   = 0x07,
    SET_FREQUENCY = 0x09,
    SET_MODE      = 0x0B,
};

enum class SmartAudioVersion : uint8_t
{
    V1   = 1,
    V2   = 2,
    V2_1 = 3,
};

struct smartaudio_settings_t
{
    SmartAudioVersion version = SmartAudioVersion::V1;

    uint8_t channel = 0;
    uint8_t power_level = 0;
    uint8_t operation_mode = 0;

    uint16_t frequency_mhz = 0;

    // SmartAudio V2.1
    uint8_t power_dbm = 0;
    uint8_t power_level_count = 0;
    uint8_t power_levels_dbm[8] = {};
};

class KssSmartAudio
{
public:
    static constexpr uint8_t VTX_POWER_DISARMED_DBM = 14U;
    static constexpr uint8_t VTX_POWER_ARMED_DBM    = 20U;
    static constexpr uint8_t VTX_POWER_HIGH_DBM     = 26U;

    esp_err_t Initialize();

    esp_err_t GetSettings(
        smartaudio_settings_t* settings);

    esp_err_t SetPowerDbm(
        uint8_t requested_dbm);

    // Debug helpers
    esp_err_t PrintSettings();
    esp_err_t TestLocalEcho(
        smartaudio_settings_t* settings);

private:
    static constexpr uint8_t SET_POWER_DBM_FLAG = 0x80U;
    static constexpr uint8_t SET_POWER_RESPONSE_ID = 0x02U;

    static constexpr size_t MAX_FRAME_SIZE = 32U;
    static constexpr size_t MAX_PAYLOAD_SIZE = 16U;
    static constexpr size_t RAW_RX_BUFFER_SIZE = 64U;

    static constexpr size_t MIN_RESPONSE_FRAME_SIZE = 5U;
    static constexpr int MAX_TRANSACTION_ATTEMPTS = 1;

    static constexpr TickType_t TX_TIMEOUT_TICKS =
        pdMS_TO_TICKS(50);

    static constexpr TickType_t RX_TIMEOUT_TICKS =
        pdMS_TO_TICKS(3000);

    static constexpr TickType_t RX_POLL_TICKS =
        pdMS_TO_TICKS(20);

    static constexpr TickType_t COMMAND_GUARD_TICKS =
        pdMS_TO_TICKS(150);

    esp_err_t SetLineTxEnabled(bool enabled);

    esp_err_t SendCommand(
        Command command,
        const uint8_t* payload,
        size_t payload_length);

    esp_err_t ReceiveResponse(
        uint8_t expected_response_id,
        uint8_t* frame,
        size_t frame_capacity,
        size_t* frame_length);

    esp_err_t ParseSettings(
        const uint8_t* frame,
        size_t frame_length,
        smartaudio_settings_t* settings);

    static bool SupportsPowerDbm(
        const smartaudio_settings_t& settings,
        uint8_t requested_dbm);

    static esp_err_t ParseSetPowerResponse(
        const uint8_t* frame,
        size_t frame_length,
        uint8_t* acknowledged_dbm);

    void WaitForCommandGuard();
    void MarkTransactionFinished();

    static uint8_t CalculateCrc(
        const uint8_t* data,
        size_t length);

    bool initialized_ = false;
    bool transaction_finished_once_ = false;
    TickType_t last_transaction_finish_tick_ = 0;
};