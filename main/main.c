/**
 * @file main.c
 * @brief Main application loop to run the application on the Esp32-P4-eye.
 *
 * @author Anthony Smaldore
 * @date 2025-06-28
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <unistd.h>        

#include "camera_init.h"
#include "serial_utils.h"


uint8_t *photo_buffer;
uint16_t photo_width = FRAME_WIDTH;
uint16_t photo_height = FRAME_HEIGHT;
size_t photo_buffer_size;
char cmd_buffer[MAX_CMD_LENGTH];

/**
 * @brief Main application loop to run on the Esp32-P4-eye.
 *
 * Initalize NVS, camera sensor, camera gpio, I2C, and esp_video subsystem.
 * Allocate space for captured frames.
 * Recognize command to capture frame.
 * Capture frame and save to buffer
 * Serialize and send over usb_serial_jtag
 * De-initalize the hardware before shutdown of the main application.
 */
void app_main(void)
{

    ESP_LOGI("main","Main App Started:");
    esp_log_level_set("*", ESP_LOG_NONE);

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_LOGI("main", "Starting camera hardware init test");

    // Initialize camera hardware (XCLK + GPIOs + I2C)
    ret = camera_hardware_init();
    if (ret != ESP_OK) {
        ESP_LOGE("main", "Camera hardware init failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = setup_video_camera_device();
    if (ret != ESP_OK) {
        ESP_LOGE("main", "Video camera device setup failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI("main", "Camera hardware and esp_video initialized successfully!");

    ESP_LOGI("main", "Initializing serial...");
    ESP_ERROR_CHECK(serial_init()); 
    ESP_LOGI("main", "Serial initialized successfully!");

    ESP_LOGI("main", "Listening for commands...");
    while(1) {
        // Wait for command
        esp_err_t err = serial_receive_command(cmd_buffer, MAX_CMD_LENGTH);
        
        if (err == ESP_OK) {
            // Verify command
            if (strcmp(cmd_buffer, "TAKE_PHOTO") == 0) {
                ESP_LOGI("main", "Received TAKE_PHOTO command");
                ESP_LOGI("main", "Waiting 10 seconds to capture frame...");
                vTaskDelay(pdMS_TO_TICKS(10000));
                ESP_LOGI("main", "Starting to capture frame...");
                ret = camera_capture_one_frame(&photo_buffer, &photo_buffer_size);

                if (ret == ESP_OK) {
                    ESP_LOGI("main", "Frame captured successfully! Size: %d bytes", photo_buffer_size);       
                    
                    ret = serial_send_frame(photo_buffer, photo_buffer_size, photo_width, photo_height);
                    if (ret != ESP_OK) {
                        ESP_LOGE("main", "Failed to send frame: %s", esp_err_to_name(ret));
                    } else {
                        ESP_LOGI("main", "Frame sent successfully");
                    }        
                }
                else {
                    ESP_LOGE("main", "Failed to capture frame: %s", esp_err_to_name(ret));
                }
            }
            else if (strcmp(cmd_buffer, "SHUTDOWN") == 0) {
                ESP_LOGI("main", "Received SHUTDOWN command");
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
            }
            else {
                ESP_LOGI("main", "Received unknown command: %s", cmd_buffer);
            }
        }
        
        // Delay to prevent tight loop
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    // De-initalize the hardware, free the allocated space for the frame, close the serial port.
    ESP_LOGI("main", "Begining de-initialization process..."); 
    ESP_LOGI("main", "De-initializing camera hardware...");
    ret = camera_hardware_deinit();
    if (ret != ESP_OK) {
        ESP_LOGE("main", "Camera hardware de-initialization failed: %s", esp_err_to_name(ret));
    }
    ESP_LOGI("main", "Camera hardware de-initialized successfully!");

    ESP_LOGI("main", "De-initializing serial...");
    ret = serial_deinit();
    if (ret != ESP_OK) {
        ESP_LOGE("main", "Serial de-initialization failed: %s", esp_err_to_name(ret));
    }
    ESP_LOGI("main", "Serial de-initialized successfully!");

    ESP_LOGI("main", "De-initializing NVS...");
    ret = nvs_flash_deinit();
    if (ret != ESP_OK) {
        ESP_LOGE("main", "NVS de-initialization failed: %s", esp_err_to_name(ret));
    }
    ESP_LOGI("main", "NVS de-initialized successfully!");

    ESP_LOGI("main", "De-initialization process completed successfully!");
}
