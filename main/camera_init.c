/**
 * @file camera_init.c
 * @brief Initalize the camera hardware and start the video system to capture frames
 *
 * @author Anthony Smaldore
 * @date 2025-06-28
 */


#include "esp_cam_sensor_xclk.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "ov2710.h"
#include "esp_sccb_i2c.h" 
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include "camera_init.h"
#include "camera_sensor.h"
#include "psram_utils.h"

esp_cam_ctlr_handle_t cam_handle = NULL;
uint8_t *frame_buffer = NULL;
size_t frame_buffer_len = 0;

// Global camera device structure
static camera_dev_t g_camera_dev = {
    .fd = -1,
    .isp_fd = -1,
    .buffers = {NULL},
    .buffer_size = 0,
    .width = 0,
    .height = 0,
    .pixelformat = 0
};

// Camera hardware variables
static i2c_master_bus_handle_t i2c_bus = NULL;
static esp_cam_sensor_xclk_handle_t xclk_handle = NULL;
static bool i2c_initialized = false;
static int isp_fd;


/**
 * @brief Setup the camera sensor, configure gpio pins, esp_video system settings, and initalize the camera. 
 * Configure camera hardware including:
 *  - internal clock
 *  - power (EN) and reset (RST) pins
 *  - MIPI camera settings for esp_video 
 *  - Power on the camera
 * 
 * @return ESP_OK for success, ESP_FAIL for failure
 */
esp_err_t camera_hardware_init(void) {

    ESP_LOGI("camera_init", "Using resolution: %u x %u ", FRAME_WIDTH, FRAME_HEIGHT);

    esp_cam_sensor_xclk_config_t xclk_cfg = {
        .esp_clock_router_cfg = {
            .xclk_pin = CAMERA_XCLK_PIN,
            .xclk_freq_hz = CAMERA_XCLK_FREQ
        }
    };
    ESP_ERROR_CHECK(esp_cam_sensor_xclk_allocate(ESP_CAM_SENSOR_XCLK_ESP_CLOCK_ROUTER, &xclk_handle));
    ESP_ERROR_CHECK(esp_cam_sensor_xclk_start(xclk_handle, &xclk_cfg));
    ESP_LOGI("camera_init", "External clock started");

    ESP_ERROR_CHECK(rtc_gpio_init(CAMERA_EN_PIN));
    ESP_ERROR_CHECK(rtc_gpio_set_direction(CAMERA_EN_PIN, RTC_GPIO_MODE_OUTPUT_ONLY));
    rtc_gpio_pulldown_dis(CAMERA_EN_PIN);
    rtc_gpio_pullup_dis(CAMERA_EN_PIN);
    rtc_gpio_hold_dis(CAMERA_EN_PIN);

    ESP_ERROR_CHECK(rtc_gpio_set_level(CAMERA_EN_PIN, 1));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI("camera_init", "Camera powered on");

    gpio_config_t rst_cfg = {
        .pin_bit_mask = BIT64(CAMERA_RST_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&rst_cfg));
    ESP_ERROR_CHECK(gpio_set_level(CAMERA_RST_PIN, 0));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(gpio_set_level(CAMERA_RST_PIN, 1));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI("camera_init", "Camera reset completed");

    ESP_ERROR_CHECK(camera_i2c_init());
    ESP_LOGI("camera_init", "I2C bus initialized");

    esp_video_init_csi_config_t csi_config = {
        .sccb_config = {
            .init_sccb = false,  
            .i2c_handle = i2c_bus,
            .freq = 100000,  
        },
        .reset_pin = CAMERA_RST_PIN,  
        .pwdn_pin = -1, 
    };

    esp_video_init_config_t cam_config = {
        .csi = &csi_config,
    };

    esp_err_t ret = esp_video_init(&cam_config);
    if (ret != ESP_OK) {
        ESP_LOGE("camera_init", "Failed to initialize camera: 0x%x", ret);
        return ret;
    }

    return ESP_OK;
}

/**
 * @brief Setup the ISP for the camera.
 * @param fd File descriptor referencing the open video device
 * Open the ISP device and store the reference to it.
 * @return ESP_OK for success, ESP_FAIL for failure
 */
esp_err_t isp_init(int camera_fd) {

    g_camera_dev.isp_fd = open(ESP_VIDEO_ISP1_DEVICE_NAME, O_RDWR);
    if (g_camera_dev.isp_fd < 0) {
        ESP_LOGE("camera_init", "Failed to open ISP device");
        return ESP_FAIL;
    }
    isp_fd = g_camera_dev.isp_fd;

    ESP_LOGI("camera_init", "ISP initialized successfully");
    return ESP_OK;
}

/**
 * @brief Set the contrast value for the ISP.
 * @param percent Precentage of contrast to set for the ISP 
 * Configure contrast settings for the device with the provided value.
 * @return ESP_OK for success, ESP_FAIL for failure
 */
esp_err_t isp_set_contrast(uint32_t percent) {

    int32_t contrast_val = (percent * 255 / 100);

    struct v4l2_ext_controls controls = {
        .ctrl_class = V4L2_CID_USER_CLASS,
        .count = 1,
        .controls = &(struct v4l2_ext_control){
            .id = V4L2_CID_CONTRAST,
            .value = contrast_val
        }
    };

    if (ioctl(isp_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGE("camera_init", "Failed to set contrast");
        return ESP_FAIL;
    }

    ESP_LOGI("camera_init", "Contrast set to %d%%", percent);
    return ESP_OK;
}

/**
 * @brief Set the saturation value for the ISP.
 * @param percent Precentage of saturation to set for the ISP 
 * Configure saturation settings for the device with the provided value.
 * @return ESP_OK for success, ESP_FAIL for failure
 */
esp_err_t isp_set_saturation(uint32_t percent) {

    int32_t saturation_val = (percent * 255 / 100);

    struct v4l2_ext_controls controls = {
        .ctrl_class = V4L2_CID_USER_CLASS,
        .count = 1,
        .controls = &(struct v4l2_ext_control){
            .id = V4L2_CID_SATURATION,
            .value = saturation_val
        }
    };

    if (ioctl(isp_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGE("camera_init", "Failed to set saturation");
        return ESP_FAIL;
    }

    ESP_LOGI("camera_init", "Saturation set to %d%%", percent);
    return ESP_OK;
}

/**
 * @brief Set the brightness value for the ISP.
 * @param percent Precentage of brightness to set for the ISP 
 * Configure brightness settings for the device with the provided value.
 * @return ESP_OK for success, ESP_FAIL for failure
 */
esp_err_t isp_set_brightness(uint32_t percent) {

    int32_t brightness_val = (percent * 255 / 100) - 127;

    struct v4l2_ext_controls controls = {
        .ctrl_class = V4L2_CID_USER_CLASS,
        .count = 1,
        .controls = &(struct v4l2_ext_control){
            .id = V4L2_CID_BRIGHTNESS,
            .value = brightness_val
        }
    };

    if (ioctl(isp_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGE("camera_init", "Failed to set brightness");
        return ESP_FAIL;
    }

    ESP_LOGI("camera_init", "Brightness set to %d%%", percent);
    return ESP_OK;
}

/**
 * @brief Set the hue value for the ISP.
 * @param percent Precentage of hue to set for the ISP 
 * Configure hue settings for the device with the provided value.
 * @return ESP_OK for success, ESP_FAIL for failure
 */
esp_err_t isp_set_hue(uint32_t percent) {

    uint32_t hue_val = 360 * percent / 100;

    struct v4l2_ext_controls controls = {
        .ctrl_class = V4L2_CID_USER_CLASS,
        .count = 1,
        .controls = &(struct v4l2_ext_control){
            .id = V4L2_CID_HUE,
            .value = hue_val
        }
    };

    if (ioctl(isp_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGE("camera_init", "Failed to set hue");
        return ESP_FAIL;
    }

    ESP_LOGI("camera_init", "Hue set to %d%%", percent);
    return ESP_OK;
}

/**
 * @brief Setup esp_video subsystem with the given camera device.
 * Configure esp_video:
 *  - Open the video device, 
 *  - Setup the ISP
 *  - Set the ISP values for the camera sensor 
 *  - Set the default width, height, and pixel format for esp_video.g_camera_dev
 *  - Request the frame buffers
 *  - Allocate space for the buffers in the PSRAM 
 *  - Queue the buffers to be used for frame captures
 *  - Start streaming frames to the esp_video device
 * @return ESP_OK for success, ESP_FAIL for failure
 */
esp_err_t setup_video_camera_device(void) {

    esp_err_t ret = ESP_OK;
    
    g_camera_dev.fd = open(CAMERA_DEV_PATH, O_RDONLY);
    if (g_camera_dev.fd < 0) {
        ESP_LOGE("camera_init", "Failed to open video device");
        return ESP_FAIL;
    }

    ret = isp_init(g_camera_dev.fd);
    if (ret != ESP_OK) {
        ESP_LOGE("camera_init", "Failed to initialize ISP");
        close(g_camera_dev.fd);
        return ret;
    }

    ret = isp_set_contrast(DEFAULT_CONTRAST_PERCENT);
    if (ret != ESP_OK) {
        ESP_LOGE("camera_init", "Failed to set ISP contrast");
        close(g_camera_dev.fd);
        return ret;
    }

    ret = isp_set_saturation(DEFAULT_SATURATION_PERCENT);
    if (ret != ESP_OK) {
        ESP_LOGE("camera_init", "Failed to set ISP saturation");
        close(g_camera_dev.fd);
        return ret;
    }

    ret = isp_set_brightness(DEFAULT_BRIGHTNESS_PERCENT);
    if (ret != ESP_OK) {
        ESP_LOGE("camera_init", "Failed to set ISP brightness");
        close(g_camera_dev.fd);
        return ret;
    }

    ret = isp_set_hue(DEFAULT_HUE_PERCENT);
    if (ret != ESP_OK) {
        ESP_LOGE("camera_init", "Failed to set ISP hue");
        close(g_camera_dev.fd);
        return ret;
    }

    struct v4l2_format fmt = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE
    };
    if (ioctl(g_camera_dev.fd, VIDIOC_G_FMT, &fmt) < 0) {
        ESP_LOGE("camera_init", "Failed to get video format");
        close(g_camera_dev.fd);
        return ESP_FAIL;
    }

    ESP_LOGI("camera_init", "Current format - width: %d, height: %d, pixelformat: 0x%x", 
         fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.pixelformat);

    g_camera_dev.width = fmt.fmt.pix.width;
    g_camera_dev.height = fmt.fmt.pix.height;
    g_camera_dev.pixelformat = fmt.fmt.pix.pixelformat;

    struct v4l2_requestbuffers req = {0};
    req.count = CAMERA_BUF_NUM;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP; 
    if (ioctl(g_camera_dev.fd, VIDIOC_REQBUFS, &req) < 0) {
        ESP_LOGE("camera_init", "Failed to request buffers: %d", errno);
        close(g_camera_dev.fd);
        return ESP_FAIL;
    }
    ESP_LOGI("camera_init", "Buffers requested successfully");

    // Allocate and setup buffers
    for (uint8_t i = 0; i < req.count; i++) {
        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;  
        buf.index = i;

        if (ioctl(g_camera_dev.fd, VIDIOC_QUERYBUF, &buf) < 0) {
            ESP_LOGE("camera_init", "Failed to query buffer %d: %d", i, errno);
            close(g_camera_dev.fd);
            return ESP_FAIL;
        }
        ESP_LOGI("camera_init", "Buffer %d size from query: %d", i, buf.length);

        g_camera_dev.buffers[i] = mmap(
            NULL,
            buf.length,
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            g_camera_dev.fd,
            buf.m.offset
        );
        if (g_camera_dev.buffers[i] == MAP_FAILED) {
            ESP_LOGE("camera_init", "Failed to mmap buffer %d: %d", i, errno);
            close(g_camera_dev.fd);
            return ESP_FAIL;
        }
        g_camera_dev.buffer_size = buf.length;

        if (ioctl(g_camera_dev.fd, VIDIOC_QBUF, &buf) < 0) {
            ESP_LOGE("camera_init", "Failed to queue buffer %d: %d", i, errno);
            munmap(g_camera_dev.buffers[i], g_camera_dev.buffer_size);
            close(g_camera_dev.fd);
            return ESP_FAIL;
        }

        ESP_LOGI("camera_init", "Buffer %d queued successfully", i);
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(g_camera_dev.fd, VIDIOC_STREAMON, &type) < 0) {
        ESP_LOGE("camera_init", "Failed to start streaming");
        close(g_camera_dev.fd);
        return ESP_FAIL;
    }

    return ESP_OK;  
}

/**
 * @brief De-intialize the camera hardware and de-allocate the buffers for the camera frames. 
 * De-initalize the camera hardware and esp_video system:
 *  - Stop streaming frames 
 *  - De-allocate buffers for the frames
 *  - Shutdown the esp_video system
 *  - Turnoff the camera sensor
 *  - Powerdown the camera
 *  - De-initalize the I2C bus
 *  - De-initalize the EN and RST pins
 * 
 * @return ESP_OK for success, ESP_FAIL for failure
 */
esp_err_t camera_hardware_deinit(void) {

    esp_err_t ret = ESP_OK;

    if (g_camera_dev.fd >= 0) {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(g_camera_dev.fd, VIDIOC_STREAMOFF, &type) < 0) {
            ESP_LOGW("camera_init", "Failed to stop streaming: %d", errno);
            ret = ESP_FAIL;
        }

        for (int i = 0; i < CAMERA_BUF_NUM; i++) {
            if (g_camera_dev.buffers[i] != NULL && g_camera_dev.buffers[i] != MAP_FAILED) {
                if (munmap(g_camera_dev.buffers[i], g_camera_dev.buffer_size) < 0) {
                    ESP_LOGW("camera_init", "Failed to unmap buffer %d: %d", i, errno);
                    ret = ESP_FAIL;
                }
                g_camera_dev.buffers[i] = NULL;
            }
        }

        if (close(g_camera_dev.fd) < 0) {
            ESP_LOGW("camera_init", "Failed to close video device: %d", errno);
            ret = ESP_FAIL;
        }
        g_camera_dev.fd = -1;
    }

    if (g_camera_dev.fd >= 0) {
        if (close(g_camera_dev.fd) < 0) {
            ESP_LOGW("camera_init", "Failed to close video device: %d", errno);
            ret = ESP_FAIL;
        }
        g_camera_dev.fd = -1;
    }

    if (g_camera_dev.isp_fd >= 0) {
        if (close(g_camera_dev.isp_fd) < 0) {
            ESP_LOGW("camera_init", "Failed to close ISP device: %d", errno);
            ret = ESP_FAIL;
        }
        g_camera_dev.isp_fd = -1;
    }

    esp_err_t video_ret = esp_video_deinit();
    if (video_ret != ESP_OK) {
        ESP_LOGW("camera_init", "Failed to deinitialize video subsystem: %d", video_ret);
        ret = ESP_FAIL;
    }

    if (xclk_handle != NULL) {
        ESP_ERROR_CHECK(esp_cam_sensor_xclk_stop(xclk_handle));
        xclk_handle = NULL;
    }

    gpio_set_level(CAMERA_RST_PIN, 0); 
    rtc_gpio_set_level(CAMERA_EN_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(100)); 
    ESP_LOGI("camera_init", "Camera powered down");

    if (i2c_initialized) {
        ESP_ERROR_CHECK(i2c_del_master_bus(i2c_bus));
        i2c_initialized = false;
    }

    rtc_gpio_deinit(CAMERA_EN_PIN);
    gpio_reset_pin(CAMERA_RST_PIN);

    return ret;
}

/**
 * @brief Initialize I2C 
 *  - Configure I2C bus 
 *  - Initialize the I2C bus
 * @return ESP_OK for success, ESP_FAIL for failure
 */
esp_err_t camera_i2c_init(void){

    if (i2c_initialized) return ESP_OK;

    i2c_master_bus_config_t i2c_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = GPIO_NUM_13,
        .sda_io_num = GPIO_NUM_14,
        .flags.enable_internal_pullup = false
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_cfg, &i2c_bus));
    i2c_initialized = true;

    return ESP_OK;
}

/**
 * @brief De-initialize I2C 
 *  - Delete I2C bus 
 * @return ESP_OK for success, ESP_FAIL for failure
 */
esp_err_t camera_i2c_deinit(void) {

    if (!i2c_initialized) return ESP_OK;

    ESP_ERROR_CHECK(i2c_del_master_bus(i2c_bus));
    i2c_initialized = false;

    return ESP_OK;
}

/**
 * @brief Retreieve the I2C handld reference.
 * @param handle Reference to the buffer to store the open I2C bus in
 * @return ESP_OK for success, ESP_FAIL for failure
 */
esp_err_t camera_get_i2c_bus_handle(i2c_master_bus_handle_t *handle) {

    if (!i2c_initialized) return ESP_ERR_INVALID_STATE;

    *handle = i2c_bus;

    return ESP_OK;
}


/**
 * @brief Capture a frame of image data.
 * @param out_buffer Buffer to store the frame data captured
 * @param out_buffer_size Size of the storage buffer 
 *  - De-queue a buffer from the device  
 *  - Save the frame data to the given buffer
 *  - Re-queue the buffer to be used to capture more frames 
 * @return ESP_OK for success, ESP_FAIL for failure
 */
esp_err_t camera_capture_one_frame(uint8_t **out_buffer, size_t *out_buffer_size)
{
    if (!out_buffer || !out_buffer_size) {
        return ESP_ERR_INVALID_ARG;
    }

    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    ESP_LOGI("camera_init", "Attempting to capture frame...");

    if (ioctl(g_camera_dev.fd, VIDIOC_DQBUF, &buf) < 0) {
        ESP_LOGE("camera_init", "Failed to dequeue buffer: %d", errno);
        return ESP_FAIL;
    }

    *out_buffer = g_camera_dev.buffers[buf.index];
    *out_buffer_size = buf.bytesused;

    ESP_LOGI("camera_init", "Frame captured successfully - buffer index: %d, size: %d bytes", 
             buf.index, buf.bytesused);

    if (ioctl(g_camera_dev.fd, VIDIOC_QBUF, &buf) < 0) {
        ESP_LOGE("camera_init", "Failed to requeue buffer: %d", errno);
        return ESP_FAIL;
    }

    return ESP_OK;
   
}
