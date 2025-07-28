#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <unistd.h>        

#include "esp32_p4_eye_audio.h"
#include "camera_init.h"
#include "serial_utils.h"


#define PCM_LEN     (16000 * 2 * 5)     // 5 seconds @ 16kHz, 16-bit mono
#define WAV_HEADER  44
#define WAV_LEN     (PCM_LEN + WAV_HEADER)

uint8_t *photo_buffer;
uint16_t photo_width = FRAME_WIDTH;
uint16_t photo_height = FRAME_HEIGHT;
size_t photo_buffer_size;

void app_main(void)
{

    ESP_LOGI("main","Main App Started:");
    esp_log_level_set("*", ESP_LOG_NONE);

  
    
    // 1. Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_LOGI("main", "Starting camera hardware init test");

    // 2. Initialize camera hardware (XCLK + GPIOs + I2C)
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

    ESP_LOGI("main", "Camera hardware initialized successfully!");

    
    
    vTaskDelay(pdMS_TO_TICKS(10000));
    ESP_LOGI("main", "Waiting 10 seconds to capture frame...");
    ESP_LOGI("main", "Starting to capture frame...");

    ret = camera_capture_one_frame(&photo_buffer, &photo_buffer_size);

     if (ret == ESP_OK) {
        ESP_LOGI("main", "Frame captured successfully! Size: %d bytes", photo_buffer_size);       
        
        ESP_ERROR_CHECK(serial_init()); 
        
        
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
    


    /*
    ESP_LOGI("main", "Initializing audio...");
    bsp_extra_pdm_codec_init();

    
    esp_err_t err = record_audio_5s();
    if (err != ESP_OK) {
        ESP_LOGE("main", "Failed to record audio: %s", esp_err_to_name(err));
    }
    

    uint8_t *pcm_buf = malloc(PCM_LEN);
    uint8_t *wav_buf = malloc(WAV_LEN);

    size_t read = 0;
    bsp_extra_pdm_i2s_read(pcm_buf, PCM_LEN, &read, 5000); // Blocking read

    audio_add_wav_header(wav_buf, PCM_LEN, 16000);          // Add WAV header
    memcpy(wav_buf + WAV_HEADER, pcm_buf, PCM_LEN);         // Append PCM data

    wifi_send_wav_file(wav_buf, WAV_LEN, "http://10.0.0.57:5050/upload");

    free(pcm_buf);
    free(wav_buf);

    
    ESP_LOGI("main", "Audio recording completed");
    */
}
