/**
 * @file camera_init.h
 * @brief Header file documenting the functions for the camera initalization and de-initalization.
 *
 * @author Anthony Smaldore
 * @date 2025-06-28
 */

#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"
#include "esp_cam_ctlr.h"
#include "esp_video_init.h"
#include "esp_video_device.h"
#include "esp_video_isp_ioctl.h"

#define CAMERA_EN_PIN        GPIO_NUM_12
#define CAMERA_RST_PIN       GPIO_NUM_26
#define CAMERA_XCLK_PIN      GPIO_NUM_11
#define CAMERA_XCLK_FREQ     24000000
#define CAMERA_DEV_PATH     "/dev/video0"
#define CAMERA_BUF_NUM     2
#define FRAME_WIDTH 1280
#define FRAME_HEIGHT 720
#define FRAME_BPP 2  

// ISP default values
#define DEFAULT_CONTRAST_PERCENT    53
#define DEFAULT_SATURATION_PERCENT  63
#define DEFAULT_BRIGHTNESS_PERCENT  50
#define DEFAULT_HUE_PERCENT        5

// Declare the globals (without initialization)
extern esp_cam_ctlr_handle_t cam_handle;
extern uint8_t *frame_buffer;
extern size_t frame_buffer_len;

// Camera device structure to hold state
typedef struct {
    int fd;                         // Video device file descriptor
    int isp_fd;                     // ISP device file descriptor
    void *buffers[CAMERA_BUF_NUM]; // Mapped memory buffers
    size_t buffer_size;            // Size of each buffer
    uint32_t width;                // Current frame width
    uint32_t height;               // Current frame height
    uint32_t pixelformat;          // Current pixel format
} camera_dev_t;


#define I2C_SCL_PIN          GPIO_NUM_13
#define I2C_SDA_PIN          GPIO_NUM_14
#define I2C_PORT             I2C_NUM_0
#ifndef MAP_FAILED
#define MAP_FAILED ((void *)-1)
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Overall setup function for camera hardware.
 * @return ESP_OK for success, ESP_FAIL for failure 
 */
esp_err_t camera_hardware_init(void);      
/**
 * @brief ISP device initalization.
 * @return ESP_OK for success, ESP_FAIL for failure 
 */
esp_err_t isp_init(int camera_fd);
/**
 * @brief Set ISP with given contrast.
 * @return ESP_OK for success, ESP_FAIL for failure 
 */
esp_err_t isp_set_contrast(uint32_t percent); 
/**
 * @brief Set ISP with given saturation.
 * @return ESP_OK for success, ESP_FAIL for failure 
 */
esp_err_t isp_set_saturation(uint32_t percent); 
/**
 * @brief Set ISP with given brightness.
 * @return ESP_OK for success, ESP_FAIL for failure 
 */
esp_err_t isp_set_brightness(uint32_t percent); 
/**
 * @brief Set ISP with given hue.
 * @return ESP_OK for success, ESP_FAIL for failure 
 */
esp_err_t isp_set_hue(uint32_t percent); 
/**
 * @brief Configure the esp_video subsystem with allocated buffers for frames, then start the system and begin to capture frames.
 * @return ESP_OK for success, ESP_FAIL for failure 
 */
esp_err_t setup_video_camera_device(void);  
/**
 * @brief De-initalize hardware that was setup for the camera and esp_video system.
 * @return ESP_OK for success, ESP_FAIL for failure 
 */
esp_err_t camera_hardware_deinit(void);     
/**
 * @brief Intialize the I2C bus.
 * @return ESP_OK for success, ESP_FAIL for failure 
 */
esp_err_t camera_i2c_init(void);           
/**
 * @brief De-intialize the I2C bus.
 * @return ESP_OK for success, ESP_FAIL for failure 
 */
esp_err_t camera_i2c_deinit(void);         
/**
 * @brief Retreive the stored I2C bus handle.
 * @return ESP_OK for success, ESP_FAIL for failure 
 */
esp_err_t camera_get_i2c_bus_handle(i2c_master_bus_handle_t *handle);  
/**
 * @brief Capture frame of image data and store in provided buffer.
 * @return ESP_OK for success, ESP_FAIL for failure 
 */
esp_err_t camera_capture_one_frame(uint8_t **out_buffer, size_t *out_buffer_size);  

#ifdef __cplusplus
}
#endif
