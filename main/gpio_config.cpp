#include "gpio_config.h"
#include "pins.h"




/** GPIO Initialization Functions */
/*  Notice : Due to RMT(DSHOT) Control, The GPIO using motors are in the "motor.cpp"*/


static const char *tag = "gpio_init";
// Internal ADC handle
static adc_oneshot_unit_handle_t s_adc_handle = nullptr;
static bool s_gpio_isr_service_installed = false;

constexpr gpio_num_t ELRS_UART_TX_PIN = GPIO_NUM_40;
constexpr gpio_num_t ELRS_UART_RX_PIN = GPIO_NUM_39;

constexpr int ELRS_UART_BAUD_RATE = 420000;
constexpr int ELRS_UART_RX_BUFFER_SIZE = 2048;
constexpr int ELRS_UART_EVENT_QUEUE_SIZE = 20;
static constexpr int ELRS_UART_TX_BUFFER_SIZE = 256;


QueueHandle_t g_elrs_uart_event_queue = nullptr;

esp_err_t BoardInitGPIOOutputs(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pin_bit_mask =
        (1ULL << PIN_BUZZER_LED);

    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), tag, "gpio_config output failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(PIN_BUZZER_LED, 0), tag, "set buzzer, led default failed");

    ESP_LOGI(tag, "GPIO outputs initialized");
    return ESP_OK;
}

esp_err_t BoardInitIMUIntGPIO(void)
{
    gpio_config_t imu_int_conf = {};
    imu_int_conf.pin_bit_mask = (1ULL << PIN_IMU_INT);
    imu_int_conf.mode = GPIO_MODE_INPUT;
    imu_int_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    imu_int_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    imu_int_conf.intr_type = GPIO_INTR_POSEDGE;

    ESP_RETURN_ON_ERROR(gpio_config(&imu_int_conf), tag, "imu int gpio config failed");

    ESP_LOGI(tag, "IMU INT GPIO initialized");
    return ESP_OK;
}

esp_err_t BoardInitSPI(board_handles_t *handles)
{
    ESP_RETURN_ON_FALSE(handles != nullptr, ESP_ERR_INVALID_ARG, tag, "handles is null");

    handles->spi_host = SPI2_HOST;
    handles->imu_spi = nullptr;

    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = PIN_IMU_MOSI;
    bus_cfg.miso_io_num = PIN_IMU_MISO;
    bus_cfg.sclk_io_num = PIN_IMU_SCLK;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 64;

    ESP_RETURN_ON_ERROR(
        spi_bus_initialize(handles->spi_host, &bus_cfg, SPI_DMA_CH_AUTO),
        tag,
        "spi bus init failed"
    );

    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.mode = 0;
    dev_cfg.clock_speed_hz = 24 * 1000 * 1000; // 24 MHz, ICM42688P
    dev_cfg.spics_io_num = PIN_IMU_CS;
    dev_cfg.queue_size = 4;
    dev_cfg.command_bits = 0;
    dev_cfg.address_bits = 0;
    dev_cfg.dummy_bits = 0;
    dev_cfg.flags = 0;

    ESP_RETURN_ON_ERROR(
        spi_bus_add_device(handles->spi_host, &dev_cfg, &handles->imu_spi),
        tag,
        "spi add imu device failed"
    );

    ESP_LOGI(tag, "SPI initialized, IMU attached");
    return ESP_OK;
}

esp_err_t BoardInitBatteryADC(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {};
    unit_cfg.unit_id = ADC_UNIT_1;
    unit_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;

    ESP_RETURN_ON_ERROR(
        adc_oneshot_new_unit(&unit_cfg, &s_adc_handle),
        tag,
        "adc new unit failed"
    );

    ///GPIO1 has ADC_CHANNEL_0
    adc_oneshot_chan_cfg_t chan_cfg = {};
    chan_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    chan_cfg.atten = ADC_ATTEN_DB_12; //from official documentation It can be read up to 3.3v

    ESP_RETURN_ON_ERROR(
        adc_oneshot_config_channel(s_adc_handle, ADC_CHANNEL_0, &chan_cfg),
        tag,
        "adc channel config failed"
    );

    ESP_LOGI(tag, "Battery ADC initialized");
    return ESP_OK;
}

esp_err_t BoardReadBatteryADCRaw(int *raw)
{
    ESP_RETURN_ON_FALSE(raw != nullptr, ESP_ERR_INVALID_ARG, tag, "raw is null");
    ESP_RETURN_ON_FALSE(s_adc_handle != nullptr, ESP_ERR_INVALID_STATE, tag, "adc not initialized");

    return adc_oneshot_read(s_adc_handle, ADC_CHANNEL_0, raw);
}


esp_err_t BoardInstallIMUISR(gpio_isr_t isr_handler, void *arg)
{
    ESP_RETURN_ON_FALSE(isr_handler != nullptr, ESP_ERR_INVALID_ARG, tag, "isr_handler is null");

    if (!s_gpio_isr_service_installed)
    {
        esp_err_t ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
        {
            ESP_LOGE(tag, "gpio_install_isr_service failed: %s", esp_err_to_name(ret));
            return ret;
        }
        s_gpio_isr_service_installed = true;
    }

    esp_err_t remove_ret = gpio_isr_handler_remove(PIN_IMU_INT);
    if (remove_ret != ESP_OK && remove_ret != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(tag, "gpio_isr_handler_remove returned: %s", esp_err_to_name(remove_ret));
    }

    ESP_RETURN_ON_ERROR(
        gpio_isr_handler_add(PIN_IMU_INT, isr_handler, arg),
        tag,
        "gpio isr handler add failed"
    );

    ESP_LOGI(tag, "IMU ISR installed");
    return ESP_OK;
}

esp_err_t BoardUartInit()
{
    {
        uart_config_t cfg{};
        cfg.baud_rate = 420000;
        cfg.data_bits = UART_DATA_8_BITS;
        cfg.parity    = UART_PARITY_DISABLE;
        cfg.stop_bits = UART_STOP_BITS_1;
        cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        cfg.source_clk = UART_SCLK_DEFAULT;

        ESP_ERROR_CHECK(uart_driver_install(    
            ELRS_UART_NUM,
            ELRS_UART_RX_BUFFER_SIZE,
            ELRS_UART_TX_BUFFER_SIZE,
            ELRS_UART_EVENT_QUEUE_SIZE,
            &g_elrs_uart_event_queue,
            0));
        ESP_ERROR_CHECK(uart_param_config(ELRS_UART_NUM, &cfg));
        ESP_ERROR_CHECK(uart_set_pin(ELRS_UART_NUM,
                                     ELRS_UART_TX,
                                     ELRS_UART_RX,
                                     UART_PIN_NO_CHANGE,
                                     UART_PIN_NO_CHANGE));
    }

    {
        uart_config_t cfg{};
        cfg.baud_rate = 115200;   // MTF-02P
        cfg.data_bits = UART_DATA_8_BITS;
        cfg.parity    = UART_PARITY_DISABLE;
        cfg.stop_bits = UART_STOP_BITS_1;
        cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        cfg.source_clk = UART_SCLK_DEFAULT;

        ESP_ERROR_CHECK(uart_driver_install(MTF_UART_NUM, 2048, 0, 0, nullptr, 0));
        ESP_ERROR_CHECK(uart_param_config(MTF_UART_NUM, &cfg));
        ESP_ERROR_CHECK(uart_set_pin(MTF_UART_NUM,
                                     MTF_UART_TX,
                                     MTF_UART_RX,
                                     UART_PIN_NO_CHANGE,
                                     UART_PIN_NO_CHANGE));
    }

    return ESP_OK;
}


esp_err_t BoardInitAll(board_handles_t *handles)
{
    ESP_RETURN_ON_ERROR(BoardInitGPIOOutputs(), tag, "gpio outputs init failed");
    ESP_RETURN_ON_ERROR(BoardInitBatteryADC(), tag, "battery adc init failed");
    ESP_RETURN_ON_ERROR(BoardInitSPI(handles), tag, "spi init failed");
    ESP_RETURN_ON_ERROR(BoardInitIMUIntGPIO(), tag, "imu int gpio init failed");
    ESP_RETURN_ON_ERROR(BoardUartInit(), tag, "uart init failed");

    ESP_LOGI(tag, "Board initialization complete");
    return ESP_OK;
}


void BoardSetBuzzerLed(bool on)
{
    gpio_set_level(PIN_BUZZER_LED, on ? 0 : 1);
}

QueueHandle_t GetElrsUartEventQueue()
{
    return g_elrs_uart_event_queue;
}
