

/*Overloading new and delete operators*/
/*Guess esp32 has already set up the memory allocation functions*/
// void* operator new(size_t size) {
//     return pvPortMalloc(size);
// }

// void* operator new[](size_t size) {
//     return pvPortMalloc(size);
// }

// void operator delete(void* p, size_t size) noexcept {
//     vPortFree(p);
// }

// void operator delete[](void* p, size_t size) noexcept {
//     vPortFree(p);
// }

///////////////////////////////////////////////////////////////
#include <cstdlib>
#include <cstdio>
#include <string>


/*Application headers*/
#include "kss_drone.h"

extern "C"
{
    /*ESP-IDF specific headers*/
    #include "esp_log.h"
    
    /*Configuration of GPIO pins*/
    #include "gpio_config.h"
    #include "pins.h"

    /*Configuration of GPIO MODE*/
    #include "driver/gpio.h"

    /*Configuration of FreeRTOS*/
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/queue.h"
}

///////////////////////////////////////////////////////////////////

static const char *tag = "app_main";
static board_handles_t board = {};
static KSSDrone drone{};

/*
 * @brief Main application entry point.
 *
 * This function initializes the application and starts the main task.
 */
extern "C" void app_main(void)
{
    /* Initialize the GPIO */
    if(BoardInitAll(&board) != ESP_OK)
    {
        ESP_LOGE(tag, "Board initialization failed");
        
        return;
    }

    ESP_LOGI(tag, "Board initialization successful");


    if(drone.Initialize(board) != ESP_OK)
    {
        ESP_LOGE(tag, "Drone initialization failed, check board handles and subsystem initialization");
        
        return;
    }

    ESP_LOGI(tag, "KSS_DRONE FW Version %.3f", FW_VERSION);
    
    vTaskDelete(nullptr);

    // printf("End of program.\n");
    return;
}

///////////////////////////////////////////////////////////////
// static const char *tag = "app_main";
// static MotorInterface motor_interface_ = {};
// static board_handles_t board = {};

// void SendMotorForDuration(MotorInterface& motor,
//                           uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4,
//                           uint32_t duration_ms)
// {
//     const TickType_t step = pdMS_TO_TICKS(2); // 500Hz 정도
//     uint32_t elapsed = 0;

//     while (elapsed < duration_ms)
//     {
//         motor.SetMotorOutput(m1, m2, m3, m4);
//         vTaskDelay(step);
//         elapsed += 2;
//     }
// }

// void TestMotorsSequential(MotorInterface& motor)
// {
//     const uint16_t TEST_DSHOT = 88;

//     while (true)
//     {
//         ESP_LOGI(tag, "All stop");
//         SendMotorForDuration(motor, 0, 0, 0, 0, 2000);

//         ESP_LOGI(tag, "M1 ON");
//         SendMotorForDuration(motor, TEST_DSHOT, 0, 0, 0, 1000);
//         SendMotorForDuration(motor, 0, 0, 0, 0, 1000);

//         ESP_LOGI(tag, "M2 ON");
//         SendMotorForDuration(motor, 0, TEST_DSHOT, 0, 0, 1000);
//         SendMotorForDuration(motor, 0, 0, 0, 0, 1000);

//         ESP_LOGI(tag, "M3 ON");
//         SendMotorForDuration(motor, 0, 0, TEST_DSHOT, 0, 1000);
//         SendMotorForDuration(motor, 0, 0, 0, 0, 1000);

//         ESP_LOGI(tag, "M4 ON");
//         SendMotorForDuration(motor, 0, 0, 0, TEST_DSHOT, 1000);
//         SendMotorForDuration(motor, 0, 0, 0, 0, 1500);
//     }
// }


// /*
//  * @brief Main application entry point.
//  *
//  * This function initializes the application and starts the main task.
//  */
// extern "C" void app_main(void)
// {
//     /* Initialize the GPIO */
//     if(BoardInitAll(&board) != ESP_OK)
//     {
//         ESP_LOGE(tag, "Board initialization failed");
        
//         return;
//     }

//     ESP_LOGI(tag, "Board initialization successful");

//     motor_interface_.Initialize();

//     SendMotorForDuration(motor_interface_, 0, 0, 0, 0, 3000);

//     motor_interface_.Arm();

//     SendMotorForDuration(motor_interface_, 0, 0, 0, 0, 2000);

//     TestMotorsSequential(motor_interface_);

//     ESP_LOGI(tag, "KSS_DRONE FW Version %.3f", FW_VERSION);
    
//     vTaskDelete(nullptr);

//     // printf("End of program.\n");
//     return;
// }



///////////////////////////////////////////////////////////////




// #include <stdio.h>
// #include "esp_mac.h" // MAC 관련 함수 포함
// #include "esp_log.h"

// extern "C" void app_main(void)
// {
//     uint8_t sta_mac[6];
//     uint8_t ap_mac[6];
    
//     // Base MAC 주소를 읽어옵니다. (6바이트 배열에 저장)
//     // esp_read_mac()은 esp_mac.h에 정의되어 있습니다.
//     esp_read_mac(sta_mac, ESP_MAC_WIFI_STA);
//     esp_read_mac(ap_mac, ESP_MAC_WIFI_SOFTAP);

//     // 읽어온 MAC 주소를 16진수 형식(XX:XX:XX:XX:XX:XX)으로 출력합니다.
//     printf("ESP32-S3 Station MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
//            sta_mac[0], sta_mac[1], sta_mac[2], sta_mac[3], sta_mac[4], sta_mac[5]);
//     printf("ESP32-S3 AP MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
//            ap_mac[0], ap_mac[1], ap_mac[2], ap_mac[3], ap_mac[4], ap_mac[5]);

// }

