
#include "motor.h"


constexpr uint32_t DSHOT_RESOLUTION_HZ = DSHOT_RESOL_HZ;

constexpr uint16_t DSHOT600_T1H = 50;
constexpr uint16_t DSHOT600_T1L = 17;
constexpr uint16_t DSHOT600_T0H = 25;
constexpr uint16_t DSHOT600_T0L = 42;

// DShot300 @ 40MHz (25ns/tick)
// bit period = 3.33us = about 133 ticks
constexpr uint16_t DSHOT300_T1H = 100;
constexpr uint16_t DSHOT300_T1L = 33;
constexpr uint16_t DSHOT300_T0H = 50;
constexpr uint16_t DSHOT300_T0L = 83;

esp_err_t Motor::Initialize(gpio_num_t pin)
{
    this->pin_ = pin;

    if (this->initialized_)
    {
        return ESP_OK;
    }

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<this->pin_);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    gpio_set_level(this->pin_, 0);

    gpio_set_drive_capability(this->pin_, GPIO_DRIVE_CAP_3);

    rmt_tx_channel_config_t tx_chan_cfg = {};
    tx_chan_cfg.gpio_num = this->pin_;
    tx_chan_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
    tx_chan_cfg.resolution_hz = DSHOT_RESOLUTION_HZ; // 40 MHz resolution for DSHOT (1us : 1MHZ, 40MHz : 25ns)
    tx_chan_cfg.mem_block_symbols = RMT_TX_CHANNEL_CONFIG_MEM_BLOCK_SYMBOLS; // Adjust based on your needs: DSHOT600 requires 16 symbols per packet, so 64 allows for 4 packets in the buffer
    tx_chan_cfg.trans_queue_depth = RMT_TX_CHANNEL_CONFIG_QUEUE_DEPTH; // Adjust based on your needs
    tx_chan_cfg.flags.invert_out = false;
    tx_chan_cfg.flags.init_level = 0;

    ESP_RETURN_ON_ERROR(
        rmt_new_tx_channel(&tx_chan_cfg, &this->channel_),
        "motor",
        "create channel failed"
    );

    ESP_RETURN_ON_ERROR(
        rmt_enable(this->channel_),
        "motor",
        "enable failed"
    );

    rmt_copy_encoder_config_t encoder_cfg = {};
    ESP_RETURN_ON_ERROR(
        rmt_new_copy_encoder(&encoder_cfg, &this->encoder_),
        "motor",
        "encoder create failed"
    );
    this->initialized_ = true;

    return ESP_OK;
}

/** dshot packet format: [15:0] throttle(11) | telemetry(1) | checksum(4)  */
/* bit15 bit14 ... bit5 | bit4 | bit3 bit2 bit1 bit0*/
/*throttle (11)       | T    | checksum(4)*/
/*Throttle range : 0-> stop, 48 ~2047(11bits)*/
/*Telemetry : 0-> no telemetry, 1-> telemetry enabled*/
/*Checksum : 4 bits, LSB of (throttle + telemetry)*/

uint16_t Motor::MakeDshotPacket(uint16_t throttle, bool telemetry)
{
    throttle &= 0x7FF; // 11bit mask

    uint16_t packet = (throttle << 1) | (telemetry ? 1 : 0);

    uint16_t csum = 0;
    uint16_t csum_data = packet;

    //checksum is the XOR of the 3 nibbles (4 bits) of the packet
    for (int i = 0; i < 3; i++)
    {
        csum ^= csum_data;
        csum_data >>= 4;
    }

    csum &= 0xF;

    packet = (packet << 4) | csum;

    return packet;
}

 /** DSHOT600 bitrates 600kbits/s : 1bit per 1.67us */
/*bit 1 : high = 1.25us, low = 0.42us*/
/*bit 0 : high = 0.625us, low = 1.04us*/
/*40Mhz -> 25ns*/
/*bit 1 : high = 1.25us/25ns = 50ticks , low = 0.42us/25ns = 17ticks*/
/*bit 0 : high = 0.625us/25ns = 25ticks, low = 1.04us/25ns = 42ticks*/
static rmt_symbol_word_t MakeDshotSymbol(bool bit)
{
    rmt_symbol_word_t symbol = {};

    if (bit)
    {
        symbol.level0 = 1;
        symbol.duration0 = DSHOT300_T1H;
        symbol.level1 = 0;
        symbol.duration1 = DSHOT300_T1L;
    }
    else
    {
        symbol.level0 = 1;
        symbol.duration0 = DSHOT300_T0H;
        symbol.level1 = 0;
        symbol.duration1 = DSHOT300_T0L;
    }

    return symbol;
}

void Motor::BuildDshotSymbols(uint16_t packet, rmt_symbol_word_t* symbols, uint16_t count)
{
    if (symbols == nullptr || count < 16)
    {
        return;
    }

    for (int i = 0; i < 16; ++i)
    {
        bool bit = (packet & (1U << (15 - i))) != 0;
        symbols[i] = MakeDshotSymbol(bit);
    }
}

esp_err_t Motor::SendThrottle(uint16_t throttle, bool telemetry)
{
    if (!initialized_ || channel_ == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }

    // esp_err_t done = rmt_tx_wait_all_done(this->channel_, 0);
    // if (done != ESP_OK)
    // {
    //     return ESP_ERR_TIMEOUT;
    // }

    uint16_t packet = MakeDshotPacket(throttle, telemetry);

    BuildDshotSymbols(packet, this->symbols_, 16);

    rmt_transmit_config_t tx_cfg = {};
    tx_cfg.loop_count = 0;

    return rmt_transmit(this->channel_, this->encoder_, this->symbols_, sizeof(this->symbols_), &tx_cfg);
}


esp_err_t Motor::WaitDone(TickType_t timeout_ticks)
{
    if (!initialized_ || channel_ == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }

    return rmt_tx_wait_all_done(this->channel_, timeout_ticks);
}