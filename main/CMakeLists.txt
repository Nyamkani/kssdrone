
file(GLOB_RECURSE SOURCES "*.cpp") 

idf_component_register(SRCS "kss_crsf_receiver.cpp" "kss_drone_compensation.cpp" "kss_drone_background.cpp" "kss_drone_armed.cpp" "kss_drone_state.cpp" "kss_drone_task.cpp" "led.cpp" "adc_battery.cpp" "espnow_interface.cpp" "kss_drone.cpp" "gpio_config.cpp" "motor_interface.cpp" "motor.cpp" "imu_icm42688p.cpp" "imu_interface.cpp" "main.cpp" ${SOURCES}
                    INCLUDE_DIRS "." 
                    REQUIRES
                    espressif__eigen
                    esp_driver_gpio
                    esp_driver_spi
                    esp_driver_rmt
                    esp_driver_uart
                    esp_adc
                    esp_wifi
                    esp_event
                    esp_netif
                    nvs_flash
                    esp_timer
                    )
