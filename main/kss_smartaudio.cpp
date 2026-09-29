#include "kss_smartaudio.h"

#include <cstring>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "pins.h"

namespace
{
constexpr uint8_t SMARTAUDIO_SYNC = 0xAAU;
constexpr uint8_t SMARTAUDIO_HEADER = 0x55U;
constexpr uint8_t GET_SETTINGS_RESPONSE_ID = 0x01U;

const char* TAG = "KssSmartAudio";
}

esp_err_t KssSmartAudio::Initialize()
{
    if (initialized_)
    {
        return ESP_OK;
    }

    if (!uart_is_driver_installed(SMARTAUDIO_UART_NUM))
    {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = uart_set_mode(
        SMARTAUDIO_UART_NUM,
        UART_MODE_UART);

    if (ret != ESP_OK)
    {
        return ret;
    }

    // 평상시에는 VTX가 1-wire 선을 구동할 수 있도록 High-Z로 둔다.
    ret = SetLineTxEnabled(false);

    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = uart_flush_input(SMARTAUDIO_UART_NUM);

    if (ret != ESP_OK)
    {
        return ret;
    }

    transaction_finished_once_ = false;
    last_transaction_finish_tick_ = 0;
    initialized_ = true;

    return ESP_OK;
}

uint8_t KssSmartAudio::CalculateCrc(
    const uint8_t* data,
    size_t length)
{
    uint8_t crc = 0;

    for (size_t i = 0; i < length; ++i)
    {
        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8U; ++bit)
        {
            crc = ((crc & 0x80U) != 0U)
                ? static_cast<uint8_t>((crc << 1U) ^ 0xD5U)
                : static_cast<uint8_t>(crc << 1U);
        }
    }

    return crc;
}

esp_err_t KssSmartAudio::SetLineTxEnabled(bool enabled)
{
    const gpio_mode_t mode = enabled
        ? GPIO_MODE_OUTPUT
        : GPIO_MODE_INPUT;

    esp_err_t ret = gpio_set_direction(
        SMARTAUDIO_UART_TX_PIN,
        mode);

    if (ret != ESP_OK)
    {
        return ret;
    }

    return gpio_set_pull_mode(
        SMARTAUDIO_UART_TX_PIN,
        GPIO_FLOATING);
}

void KssSmartAudio::WaitForCommandGuard()
{
    if (!transaction_finished_once_)
    {
        return;
    }

    const TickType_t now = xTaskGetTickCount();
    const TickType_t elapsed = now - last_transaction_finish_tick_;

    if (elapsed < COMMAND_GUARD_TICKS)
    {
        vTaskDelay(COMMAND_GUARD_TICKS - elapsed);
    }
}

void KssSmartAudio::MarkTransactionFinished()
{
    last_transaction_finish_tick_ = xTaskGetTickCount();
    transaction_finished_once_ = true;
}

esp_err_t KssSmartAudio::SendCommand(
    Command command,
    const uint8_t* payload,
    size_t payload_length)
{
    if (!initialized_)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (payload_length > MAX_PAYLOAD_SIZE)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    if ((payload_length > 0U) && (payload == nullptr))
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Dummy + AA 55 + Command + Length + Payload + CRC
    uint8_t frame[MAX_PAYLOAD_SIZE + 6U] = {};
    size_t frame_index = 0;

    // SmartAudio 송신 전 LOW 구간을 만드는 dummy byte.
    frame[frame_index++] = 0x00U;

    const size_t crc_start_index = frame_index;

    frame[frame_index++] = SMARTAUDIO_SYNC;
    frame[frame_index++] = SMARTAUDIO_HEADER;
    frame[frame_index++] = static_cast<uint8_t>(command);
    frame[frame_index++] = static_cast<uint8_t>(payload_length);

    if (payload_length > 0U)
    {
        std::memcpy(
            &frame[frame_index],
            payload,
            payload_length);
        frame_index += payload_length;
    }

    // 요청 CRC 범위는 AA 55부터 Payload 끝까지이다.
    const uint8_t crc = CalculateCrc(
        &frame[crc_start_index],
        frame_index - crc_start_index);
    frame[frame_index++] = crc;

    esp_err_t ret = uart_flush_input(SMARTAUDIO_UART_NUM);

    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = SetLineTxEnabled(true);

    if (ret != ESP_OK)
    {
        return ret;
    }



ESP_LOGI(TAG, "TX frame: length=%u",
         static_cast<unsigned>(frame_index));

ESP_LOG_BUFFER_HEX_LEVEL(
    TAG,
    frame,
    frame_index,
    ESP_LOG_INFO);

    const int written_length = uart_write_bytes(
        SMARTAUDIO_UART_NUM,
        frame,
        frame_index);

    if (written_length != static_cast<int>(frame_index))
    {
        const esp_err_t rx_ret = SetLineTxEnabled(false);

        if (rx_ret != ESP_OK)
        {
            return rx_ret;
        }

        return (written_length < 0)
            ? ESP_FAIL
            : ESP_ERR_INVALID_SIZE;
    }

    const esp_err_t tx_ret = uart_wait_tx_done(
        SMARTAUDIO_UART_NUM,
        TX_TIMEOUT_TICKS);

    // 성공/실패와 관계없이 송신 직후 VTX에 선을 반환한다.
    const esp_err_t rx_ret = SetLineTxEnabled(false);

    if (rx_ret != ESP_OK)
    {
        return rx_ret;
    }

    return tx_ret;
}


esp_err_t KssSmartAudio::ReceiveResponse(
    uint8_t expected_response_id,
    uint8_t* frame,
    size_t frame_capacity,
    size_t* frame_length)
{
    if ((frame == nullptr) || (frame_length == nullptr))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (frame_capacity < MIN_RESPONSE_FRAME_SIZE)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    *frame_length = 0U;

    uint8_t raw_rx[RAW_RX_BUFFER_SIZE] = {};
    size_t received_length = 0U;

    const TickType_t start_tick = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start_tick) < RX_TIMEOUT_TICKS)
    {
        const TickType_t elapsed_ticks =
            xTaskGetTickCount() - start_tick;

        const TickType_t remaining_ticks =
            RX_TIMEOUT_TICKS - elapsed_ticks;

        const TickType_t read_timeout =
            (remaining_ticks < RX_POLL_TICKS)
                ? remaining_ticks
                : RX_POLL_TICKS;

        if (received_length < sizeof(raw_rx))
        {
            const int read_length = uart_read_bytes(
                SMARTAUDIO_UART_NUM,
                &raw_rx[received_length],
                sizeof(raw_rx) - received_length,
                read_timeout);

            if (read_length < 0)
            {
                return ESP_FAIL;
            }

            received_length +=
                static_cast<size_t>(read_length);
        }

        /*
         * 최소 응답 프레임:
         *
         * AA 55 Command Length CRC
         */
        if (received_length < MIN_RESPONSE_FRAME_SIZE)
        {
            continue;
        }

        for (size_t offset = 0U;
             (offset + MIN_RESPONSE_FRAME_SIZE) <= received_length;
             ++offset)
        {
            if ((raw_rx[offset] != SMARTAUDIO_SYNC) ||
                (raw_rx[offset + 1U] != SMARTAUDIO_HEADER))
            {
                continue;
            }

            const uint8_t response_command =
                raw_rx[offset + 2U];

            const size_t payload_length =
                raw_rx[offset + 3U];

            if (payload_length > MAX_PAYLOAD_SIZE)
            {
                ESP_LOGW(
                    TAG,
                    "RX candidate invalid payload length: "
                    "command=0x%02X, length=%u",
                    response_command,
                    static_cast<unsigned>(payload_length));

                continue;
            }

            // AA 55 + Command + Length + Payload + CRC
            const size_t candidate_length =
                payload_length + 5U;

            // 아직 프레임 전체가 수신되지 않은 경우
            if ((offset + candidate_length) > received_length)
            {
                continue;
            }

            ESP_LOGI(
                TAG,
                "RX candidate: command=0x%02X, "
                "expected_id=0x%02X, payload_length=%u, "
                "frame_length=%u",
                response_command,
                expected_response_id,
                static_cast<unsigned>(payload_length),
                static_cast<unsigned>(candidate_length));

            ESP_LOG_BUFFER_HEX_LEVEL(
                TAG,
                &raw_rx[offset],
                candidate_length,
                ESP_LOG_INFO);

            /*
             * VTX → Host 응답 CRC:
             * AA 55는 제외하고 Command부터 Payload 끝까지 계산
             */
            const uint8_t calculated_crc = CalculateCrc(
                &raw_rx[offset + 2U],
                candidate_length - 3U);

            const uint8_t received_crc =
                raw_rx[offset + candidate_length - 1U];

            if (calculated_crc != received_crc)
            {
                ESP_LOGW(
                    TAG,
                    "RX candidate CRC mismatch: "
                    "command=0x%02X, calculated=0x%02X, "
                    "received=0x%02X",
                    response_command,
                    calculated_crc,
                    received_crc);

                continue;
            }

            /*
             * SmartAudio 응답 ID:
             *
             * GET_SETTINGS V2.1: 0x11 & 0x07 = 0x01
             * SET_POWER:        0x02 & 0x07 = 0x02
             */
            const uint8_t response_id =
                response_command & 0x07U;

            if (response_id != expected_response_id)
            {
                ESP_LOGW(
                    TAG,
                    "RX valid frame but unexpected response: "
                    "command=0x%02X, response_id=0x%02X, "
                    "expected_id=0x%02X",
                    response_command,
                    response_id,
                    expected_response_id);

                continue;
            }

            if (candidate_length > frame_capacity)
            {
                return ESP_ERR_INVALID_SIZE;
            }

            std::memcpy(
                frame,
                &raw_rx[offset],
                candidate_length);

            *frame_length = candidate_length;

            return ESP_OK;
        }

        if (received_length >= sizeof(raw_rx))
        {
            ESP_LOGW(
                TAG,
                "RX buffer full without expected response");

            break;
        }
    }

    if (received_length == 0U)
    {
        ESP_LOGW(
            TAG,
            "RX timeout: expected_id=0x%02X, no bytes received",
            expected_response_id);
    }
    else
    {
        ESP_LOGW(
            TAG,
            "RX timeout: expected_id=0x%02X, received=%u bytes",
            expected_response_id,
            static_cast<unsigned>(received_length));

        ESP_LOG_BUFFER_HEX_LEVEL(
            TAG,
            raw_rx,
            received_length,
            ESP_LOG_WARN);
    }

    return ESP_ERR_TIMEOUT;
}


esp_err_t KssSmartAudio::ParseSettings(
    const uint8_t* frame,
    size_t frame_length,
    smartaudio_settings_t* settings)
{
    if ((frame == nullptr) || (settings == nullptr))
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Header(2) + Command + Length + 기본 Payload(5) + CRC
    if (frame_length < 10U)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    if ((frame[0] != SMARTAUDIO_SYNC) ||
        (frame[1] != SMARTAUDIO_HEADER))
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    const uint8_t response_command = frame[2];

    if ((response_command & 0x07U) != GET_SETTINGS_RESPONSE_ID)
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    const size_t payload_length = frame[3];

    if (frame_length != (5U + payload_length))
    {
        return ESP_ERR_INVALID_SIZE;
    }

    SmartAudioVersion version;

    switch (response_command >> 3U)
    {
    case 0U:
        version = SmartAudioVersion::V1;
        break;

    case 1U:
        version = SmartAudioVersion::V2;
        break;

    case 2U:
        version = SmartAudioVersion::V2_1;
        break;

    default:
        return ESP_ERR_NOT_SUPPORTED;
    }

    smartaudio_settings_t parsed = {};

    parsed.version = version;
    parsed.channel = frame[4];
    parsed.power_level = frame[5];
    parsed.operation_mode = frame[6];
    parsed.frequency_mhz = static_cast<uint16_t>(
        (static_cast<uint16_t>(frame[7]) << 8U) |
        static_cast<uint16_t>(frame[8]));

    if (version == SmartAudioVersion::V2_1)
    {
        // 기본 5바이트 + 현재 dBm + 단계 개수 = 최소 7바이트.
        if (payload_length < 7U)
        {
            return ESP_ERR_INVALID_SIZE;
        }

        parsed.power_dbm = frame[9];

        // V2.1의 이 필드는 마지막 인덱스가 아니라 실제 단계 개수다.
        const size_t power_level_count = frame[10];
        const size_t available_power_levels = payload_length - 7U;

        if ((power_level_count > available_power_levels) ||
            (power_level_count > sizeof(parsed.power_levels_dbm)))
        {
            return ESP_ERR_INVALID_SIZE;
        }

        parsed.power_level_count =
            static_cast<uint8_t>(power_level_count);

        for (size_t i = 0; i < power_level_count; ++i)
        {
            parsed.power_levels_dbm[i] = frame[11U + i];
        }
    }

    *settings = parsed;
    return ESP_OK;
}

esp_err_t KssSmartAudio::GetSettings(
    smartaudio_settings_t* settings)
{
    if (!initialized_)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (settings == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t last_error = ESP_ERR_TIMEOUT;

    for (int attempt = 0;
         attempt < MAX_TRANSACTION_ATTEMPTS;
         ++attempt)
    {
        WaitForCommandGuard();

        esp_err_t ret = SendCommand(
            Command::GET_SETTINGS,
            nullptr,
            0U);

        if (ret != ESP_OK)
        {
            MarkTransactionFinished();
            last_error = ret;

            ESP_LOGW(
                TAG,
                "Get Settings TX failed: attempt=%d/%d, error=%s",
                attempt + 1,
                MAX_TRANSACTION_ATTEMPTS,
                esp_err_to_name(ret));
            continue;
        }

        uint8_t response[MAX_FRAME_SIZE] = {};
        size_t response_length = 0;

        ret = ReceiveResponse(
            GET_SETTINGS_RESPONSE_ID,
            response,
            sizeof(response),
            &response_length);

        MarkTransactionFinished();

        if (ret != ESP_OK)
        {
            last_error = ret;

            ESP_LOGW(
                TAG,
                "Get Settings RX failed: attempt=%d/%d, error=%s",
                attempt + 1,
                MAX_TRANSACTION_ATTEMPTS,
                esp_err_to_name(ret));
            continue;
        }

        ret = ParseSettings(
            response,
            response_length,
            settings);

        if (ret == ESP_OK)
        {
            return ESP_OK;
        }

        last_error = ret;

        ESP_LOGW(
            TAG,
            "Get Settings parse failed: attempt=%d/%d, error=%s",
            attempt + 1,
            MAX_TRANSACTION_ATTEMPTS,
            esp_err_to_name(ret));
    }

    return last_error;
}

esp_err_t KssSmartAudio::TestLocalEcho(
    smartaudio_settings_t* settings)
{
    // 기존 디버그 호출부와의 호환성을 유지한다.
    return GetSettings(settings);
}

esp_err_t KssSmartAudio::PrintSettings()
{
    smartaudio_settings_t settings = {};

    const esp_err_t ret = GetSettings(&settings);

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "GetSettings failed: %s",
            esp_err_to_name(ret));
        return ret;
    }

    const char* version_string = "UNKNOWN";

    switch (settings.version)
    {
    case SmartAudioVersion::V1:
        version_string = "V1";
        break;

    case SmartAudioVersion::V2:
        version_string = "V2";
        break;

    case SmartAudioVersion::V2_1:
        version_string = "V2.1";
        break;
    }

    ESP_LOGI(TAG, "SmartAudio settings");
    ESP_LOGI(TAG, "  Version        : %s", version_string);
    ESP_LOGI(TAG, "  Channel        : %u", settings.channel);
    ESP_LOGI(TAG, "  Power level    : %u", settings.power_level);
    ESP_LOGI(TAG, "  Operation mode : 0x%02X", settings.operation_mode);
    ESP_LOGI(TAG, "  Frequency      : %u MHz", settings.frequency_mhz);

    if (settings.version == SmartAudioVersion::V2_1)
    {
        ESP_LOGI(TAG, "  Current power  : %u dBm", settings.power_dbm);
        ESP_LOGI(TAG, "  Power count    : %u", settings.power_level_count);

        for (uint8_t i = 0;
             i < settings.power_level_count;
             ++i)
        {
            ESP_LOGI(
                TAG,
                "  Power[%u]       : %u dBm",
                i,
                settings.power_levels_dbm[i]);
        }
    }

    return ESP_OK;
}

void smartaudio_test_loop()
{
    static KssSmartAudio smartaudio;
    static bool initialized = false;

    if (!initialized)
    {
        const esp_err_t ret = smartaudio.Initialize();

        if (ret != ESP_OK)
        {
            ESP_LOGE(
                TAG,
                "SmartAudio Initialize failed: %s",
                esp_err_to_name(ret));
            return;
        }

        initialized = true;
    }

    smartaudio.PrintSettings();
}

void test_smartaudio()
{
    ESP_LOGI(TAG, "OpenVTX SmartAudio test start (30 iterations)");

    for (int i = 0; i < 30; ++i)
    {
        ESP_LOGI(TAG, "[%d/30]", i + 1);
        smartaudio_test_loop();
    }

    ESP_LOGI(TAG, "SmartAudio test finished");
}

bool KssSmartAudio::SupportsPowerDbm(
    const smartaudio_settings_t& settings,
    uint8_t requested_dbm)
{
    if (settings.version != SmartAudioVersion::V2_1)
    {
        return false;
    }

    constexpr size_t POWER_LEVEL_CAPACITY =
        sizeof(settings.power_levels_dbm) /
        sizeof(settings.power_levels_dbm[0]);

    if (settings.power_level_count > POWER_LEVEL_CAPACITY)
    {
        return false;
    }

    for (size_t index = 0;
         index < settings.power_level_count;
         ++index)
    {
        if (settings.power_levels_dbm[index] == requested_dbm)
        {
            return true;
        }
    }

    return false;
}

esp_err_t KssSmartAudio::ParseSetPowerResponse(
    const uint8_t* frame,
    size_t frame_length,
    uint8_t* acknowledged_dbm)
{
    if ((frame == nullptr) || (acknowledged_dbm == nullptr))
    {
        return ESP_ERR_INVALID_ARG;
    }

    // AA 55 + Command + Length + Payload(2) + CRC
    if (frame_length != 7U)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    if ((frame[0] != SMARTAUDIO_SYNC) ||
        (frame[1] != SMARTAUDIO_HEADER))
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    if ((frame[2] & 0x07U) != SET_POWER_RESPONSE_ID)
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    const size_t payload_length = frame[3];

    if ((payload_length != 2U) ||
        (frame_length != (5U + payload_length)))
    {
        return ESP_ERR_INVALID_SIZE;
    }

    // Payload[0]: VTX가 실제 적용한 출력값(dBm)
    // Payload[1]: reserved
    *acknowledged_dbm = frame[4];

    return ESP_OK;
}

esp_err_t KssSmartAudio::SetPowerDbm(
    uint8_t requested_dbm)
{
    if (!initialized_)
    {
        return ESP_ERR_INVALID_STATE;
    }

    // 호출자는 MSB가 없는 순수 dBm 값만 전달한다.
    if ((requested_dbm & SET_POWER_DBM_FLAG) != 0U)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // GET_SETTINGS에서 VTX가 제공한 출력 테이블을 확인한다.
    smartaudio_settings_t settings = {};

    esp_err_t ret = GetSettings(&settings);

    if (ret != ESP_OK)
    {
        ESP_LOGW(
            TAG,
            "GetSettings before SET_POWER failed: %s",
            esp_err_to_name(ret));

        return ret;
    }

    if (settings.version != SmartAudioVersion::V2_1)
    {
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (!SupportsPowerDbm(settings, requested_dbm))
    {
        ESP_LOGE(
            TAG,
            "Unsupported VTX power: %u dBm",
            requested_dbm);

        return ESP_ERR_NOT_SUPPORTED;
    }

    // SmartAudio V2.1 dBm 설정은 MSB를 1로 설정한다.
    const uint8_t payload = static_cast<uint8_t>(
        SET_POWER_DBM_FLAG | requested_dbm);

    esp_err_t last_error = ESP_ERR_TIMEOUT;

    for (int attempt = 0;
         attempt < MAX_TRANSACTION_ATTEMPTS;
         ++attempt)
    {
        WaitForCommandGuard();

        ret = SendCommand(
            Command::SET_POWER,
            &payload,
            sizeof(payload));

        if (ret != ESP_OK)
        {
            MarkTransactionFinished();
            last_error = ret;

            ESP_LOGW(
                TAG,
                "Set Power TX failed: attempt=%d/%d, error=%s",
                attempt + 1,
                MAX_TRANSACTION_ATTEMPTS,
                esp_err_to_name(ret));

            continue;
        }

        uint8_t response[MAX_FRAME_SIZE] = {};
        size_t response_length = 0U;

        ret = ReceiveResponse(
            SET_POWER_RESPONSE_ID,
            response,
            sizeof(response),
            &response_length);

        MarkTransactionFinished();

        if (ret != ESP_OK)
        {
            last_error = ret;

            ESP_LOGW(
                TAG,
                "Set Power RX failed: attempt=%d/%d, error=%s",
                attempt + 1,
                MAX_TRANSACTION_ATTEMPTS,
                esp_err_to_name(ret));

            continue;
        }

        uint8_t acknowledged_dbm = 0U;

        ret = ParseSetPowerResponse(
            response,
            response_length,
            &acknowledged_dbm);

        if (ret != ESP_OK)
        {
            last_error = ret;

            ESP_LOGW(
                TAG,
                "Set Power parse failed: attempt=%d/%d, error=%s",
                attempt + 1,
                MAX_TRANSACTION_ATTEMPTS,
                esp_err_to_name(ret));

            continue;
        }

        if (acknowledged_dbm != requested_dbm)
        {
            last_error = ESP_ERR_INVALID_RESPONSE;

            ESP_LOGW(
                TAG,
                "Set Power mismatch: requested=%u dBm, response=%u dBm",
                requested_dbm,
                acknowledged_dbm);

            continue;
        }

        last_error = ESP_OK;
        break;
    }

if (last_error != ESP_OK)
{
    if (last_error == ESP_ERR_TIMEOUT)
    {
        ESP_LOGW(
            TAG,
            "SET_POWER ACK timeout; verify applied power after 1 second");
    }
    else
    {
        return last_error;
    }
}

// vTaskDelay(pdMS_TO_TICKS(1000));

    // SET_POWER 응답뿐 아니라 GET_SETTINGS로 최종 적용값을 확인한다.
    smartaudio_settings_t applied_settings = {};

    ret = GetSettings(&applied_settings);

    if (ret != ESP_OK)
    {
        return ret;
    }

    if ((applied_settings.version != SmartAudioVersion::V2_1) ||
        (applied_settings.power_dbm != requested_dbm))
    {
        ESP_LOGE(
            TAG,
            "Power verification failed: requested=%u dBm, applied=%u dBm",
            requested_dbm,
            applied_settings.power_dbm);

        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_LOGI(
        TAG,
        "VTX power applied: %u dBm",
        requested_dbm);

    return ESP_OK;
}