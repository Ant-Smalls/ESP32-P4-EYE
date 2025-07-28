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


static i2c_master_bus_handle_t i2c_bus = NULL;
static esp_cam_sensor_xclk_handle_t xclk_handle = NULL;
static bool i2c_initialized = false;
static int isp_fd;

esp_err_t camera_hardware_init(void) {

    ESP_LOGI("camera_init", "Using resolution: %u x %u ", FRAME_WIDTH, FRAME_HEIGHT);


     //Setup the hardware for the camera
     //Start the external clock for the camera
    esp_cam_sensor_xclk_config_t xclk_cfg = {
        .esp_clock_router_cfg = {
            .xclk_pin = CAMERA_XCLK_PIN,
            .xclk_freq_hz = CAMERA_XCLK_FREQ
        }
    };
    ESP_ERROR_CHECK(esp_cam_sensor_xclk_allocate(ESP_CAM_SENSOR_XCLK_ESP_CLOCK_ROUTER, &xclk_handle));
    ESP_ERROR_CHECK(esp_cam_sensor_xclk_start(xclk_handle, &xclk_cfg));

    ESP_LOGI("camera_init", "External clock started");

    //Power on the camera
    //Enable the power and reset pins
    ESP_ERROR_CHECK(rtc_gpio_init(CAMERA_EN_PIN));
    ESP_ERROR_CHECK(rtc_gpio_set_direction(CAMERA_EN_PIN, RTC_GPIO_MODE_OUTPUT_ONLY));
    rtc_gpio_pulldown_dis(CAMERA_EN_PIN);
    rtc_gpio_pullup_dis(CAMERA_EN_PIN);
    rtc_gpio_hold_dis(CAMERA_EN_PIN);

    //Turn on the camera and enable it to maintain power
    ESP_ERROR_CHECK(rtc_gpio_set_level(CAMERA_EN_PIN, 1));
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI("camera_init", "Camera powered on");

    //Configure the RST pin
    gpio_config_t rst_cfg = {
        .pin_bit_mask = BIT64(CAMERA_RST_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&rst_cfg));

    //Reset the camera
    ESP_ERROR_CHECK(gpio_set_level(CAMERA_RST_PIN, 0));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(gpio_set_level(CAMERA_RST_PIN, 1));
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI("camera_init", "Camera reset completed");


    //Initialize the I2C bus
    ESP_ERROR_CHECK(camera_i2c_init());
    ESP_LOGI("camera_init", "I2C bus initialized");


     // Configure MIPI CSI settings
    esp_video_init_csi_config_t csi_config = {
        .sccb_config = {
            .init_sccb = false,  
            .i2c_handle = i2c_bus,
            .freq = 100000,  // 100kHz - standard I2C frequency for OV2710
        },
        // These pins should match your hardware setup
        .reset_pin = CAMERA_RST_PIN,  // Usually GPIO 39
        .pwdn_pin = -1,   // Usually GPIO 40
    };

    // Create main camera config structure
    esp_video_init_config_t cam_config = {
        .csi = &csi_config,
    };

    // Initialize the camera
    esp_err_t ret = esp_video_init(&cam_config);
    if (ret != ESP_OK) {
        ESP_LOGE("camera_init", "Failed to initialize camera: 0x%x", ret);
        return ret;
    }

    return ESP_OK;
}


esp_err_t isp_init(int camera_fd) {

    g_camera_dev.isp_fd = open(ESP_VIDEO_ISP1_DEVICE_NAME, O_RDWR);
    if (g_camera_dev.isp_fd < 0) {
        ESP_LOGE("camera_init", "Failed to open ISP device");
        return ESP_FAIL;
    }

    // Set the ISP file descriptor
    isp_fd = g_camera_dev.isp_fd;

    ESP_LOGI("camera_init", "ISP initialized successfully");

    return ESP_OK;
}

esp_err_t isp_set_contrast(uint32_t percent) 
{
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

esp_err_t isp_set_saturation(uint32_t percent)
{
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

esp_err_t isp_set_brightness(uint32_t percent)
{
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

esp_err_t isp_set_hue(uint32_t percent)
{
    uint32_t hue_val = 360 * percent / 100;

    // Ensure the hue value is within the supported range
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

esp_err_t setup_video_camera_device(void) {
    esp_err_t ret = ESP_OK;
    
    // 1. Open the video device
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

    

    // 3. Query current format
    struct v4l2_format fmt = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE
    };
    if (ioctl(g_camera_dev.fd, VIDIOC_G_FMT, &fmt) < 0) {
        ESP_LOGE("camera_init", "Failed to get video format");
        close(g_camera_dev.fd);
        return ESP_FAIL;
    }

    // Log the current format
    ESP_LOGI("camera_init", "Current format - width: %d, height: %d, pixelformat: 0x%x", 
         fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.pixelformat);

    // Store the current format
    g_camera_dev.width = fmt.fmt.pix.width;
    g_camera_dev.height = fmt.fmt.pix.height;
    g_camera_dev.pixelformat = fmt.fmt.pix.pixelformat;
  
    // 4. Request buffers
    struct v4l2_requestbuffers req = {0};
    req.count = CAMERA_BUF_NUM;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;  // Change to MMAP instead of USERPTR

    if (ioctl(g_camera_dev.fd, VIDIOC_REQBUFS, &req) < 0) {
        ESP_LOGE("camera_init", "Failed to request buffers: %d", errno);
        close(g_camera_dev.fd);
        return ESP_FAIL;
    }

    ESP_LOGI("camera_init", "Buffers requested successfully");

    // 5. Allocate and setup buffers
    for (uint8_t i = 0; i < req.count; i++) {
        // First query the buffer to get the proper size
        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;  // Match the memory type from request
        buf.index = i;

        if (ioctl(g_camera_dev.fd, VIDIOC_QUERYBUF, &buf) < 0) {
            ESP_LOGE("camera_init", "Failed to query buffer %d: %d", i, errno);
            close(g_camera_dev.fd);
            return ESP_FAIL;
        }

        ESP_LOGI("camera_init", "Buffer %d size from query: %d", i, buf.length);

        // Map the buffer
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

        // Queue the buffer - keep the original queried buffer structure
        if (ioctl(g_camera_dev.fd, VIDIOC_QBUF, &buf) < 0) {
            ESP_LOGE("camera_init", "Failed to queue buffer %d: %d", i, errno);
            munmap(g_camera_dev.buffers[i], g_camera_dev.buffer_size);
            close(g_camera_dev.fd);
            return ESP_FAIL;
        }

        ESP_LOGI("camera_init", "Buffer %d queued successfully", i);
    }

    // 5. Start streaming
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(g_camera_dev.fd, VIDIOC_STREAMON, &type) < 0) {
        ESP_LOGE("camera_init", "Failed to start streaming");
        close(g_camera_dev.fd);
        return ESP_FAIL;
    }

    return ESP_OK;  // Return the file descriptor for future use
}





esp_err_t camera_hardware_deinit(void) {

    //Stop the external clock for the camera and deinitialize the I2C bus
    ESP_ERROR_CHECK(i2c_del_master_bus(i2c_bus));
    ESP_ERROR_CHECK(esp_cam_sensor_xclk_stop(xclk_handle));

    if (g_camera_dev.fd >= 0) {
        // Stop streaming
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(g_camera_dev.fd, VIDIOC_STREAMOFF, &type);

        // Unmap buffers
        for (int i = 0; i < CAMERA_BUF_NUM; i++) {
            if (g_camera_dev.buffers[i] != NULL && g_camera_dev.buffers[i] != MAP_FAILED) {
                munmap(g_camera_dev.buffers[i], g_camera_dev.buffer_size);
                g_camera_dev.buffers[i] = NULL;
            }
        }

        // Close device
        close(g_camera_dev.fd);
        g_camera_dev.fd = -1;
    }

    return ESP_OK;
}

esp_err_t camera_i2c_init(void){

    //Check if the I2C bus is already initialized
    if (i2c_initialized) return ESP_OK;

    //Configure the I2C bus
    i2c_master_bus_config_t i2c_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = GPIO_NUM_13,
        .sda_io_num = GPIO_NUM_14,
        .flags.enable_internal_pullup = false
    };

    //Initialize the I2C bus
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_cfg, &i2c_bus));
    i2c_initialized = true;

    return ESP_OK;
}


esp_err_t camera_i2c_deinit(void) {

    //Check if the I2C bus is initialized
    if (!i2c_initialized) return ESP_OK;

    //Deinitialize the I2C bus
    ESP_ERROR_CHECK(i2c_del_master_bus(i2c_bus));
    i2c_initialized = false;

    return ESP_OK;
}


esp_err_t camera_get_i2c_bus_handle(i2c_master_bus_handle_t *handle) {

    //Check if the I2C bus is initialized
    if (!i2c_initialized) return ESP_ERR_INVALID_STATE;

    //Return the I2C bus handle
    *handle = i2c_bus;

    return ESP_OK;
}



esp_err_t camera_capture_one_frame(uint8_t **out_buffer, size_t *out_buffer_size)
{
    if (!out_buffer || !out_buffer_size) {
        return ESP_ERR_INVALID_ARG;
    }

    // Prepare buffer structure for dequeuing
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    ESP_LOGI("camera_init", "Attempting to capture frame...");

    // Dequeue a buffer (this will wait for a frame to be ready)
    if (ioctl(g_camera_dev.fd, VIDIOC_DQBUF, &buf) < 0) {
        ESP_LOGE("camera_init", "Failed to dequeue buffer: %d", errno);
        return ESP_FAIL;
    }

    // Get the frame data from our mapped buffer
    *out_buffer = g_camera_dev.buffers[buf.index];
    *out_buffer_size = buf.bytesused;

    ESP_LOGI("camera_init", "Frame captured successfully - buffer index: %d, size: %d bytes", 
             buf.index, buf.bytesused);

    // Queue the buffer back to the camera
    if (ioctl(g_camera_dev.fd, VIDIOC_QBUF, &buf) < 0) {
        ESP_LOGE("camera_init", "Failed to requeue buffer: %d", errno);
        return ESP_FAIL;
    }

    return ESP_OK;
   
}
