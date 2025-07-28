
#include "camera_capture.h"
#include "camera_init.h"

#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_video_init.h"
#include <errno.h>
#include <dirent.h>
#include "esp_video_device.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" 


#define CAMERA_DEV_PATH     "/dev/video0"
#define CAMERA_BUF_COUNT     1
#define CAMERA_FMT_PIX       V4L2_PIX_FMT_RGB565

static const char *TAG = "camera_capture";

// Internal state
static int video_fd = -1;
static uint8_t *camera_buffers[CAMERA_BUF_COUNT] = {0};
static size_t camera_buf_len = 0;

static struct v4l2_format cached_fmt;

static bool camera_initialized = false;

#ifndef MAP_FAILED
#define MAP_FAILED ((void *) -1)
#endif




static esp_err_t initialize_esp_video_with_mipi_csi(void)
{
    i2c_master_bus_handle_t i2c_bus_handle;

    esp_err_t i2c_status = camera_get_i2c_bus_handle(&i2c_bus_handle);
    if (i2c_status != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get I2C handle for camera");
        return i2c_status;
    }

    esp_video_init_csi_config_t csi_config = {
        .sccb_config = {
            .init_sccb = false,              // Don't reinitialize SCCB/I2C
            .i2c_handle = i2c_bus_handle,    // Use the already-initialized I2C bus
            .freq = 100000,
        },
        .reset_pin = CAMERA_RST_PIN,
        .pwdn_pin = -1
    };


    esp_video_init_config_t config = {
        .csi = &csi_config,
    };

    esp_err_t ret = esp_video_init(&config);
    ESP_LOGI(TAG, "esp_video_init() returned: %s", esp_err_to_name(ret));
    ESP_LOGI(TAG, "Expected device path: %s", ESP_VIDEO_MIPI_CSI_DEVICE_NAME);

    return ret;
}


static esp_err_t camera_open_and_configure(void)
{

    //Open the camera device
    video_fd = open(CAMERA_DEV_PATH, O_RDWR);
    if (video_fd < 0) {
        ESP_LOGE(TAG, "Failed to open %s (errno: %d - %s)", CAMERA_DEV_PATH, errno, strerror(errno));
        return ESP_FAIL;
    }

    struct v4l2_fmtdesc fmtdesc = {0};
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    ESP_LOGI(TAG, "Supported pixel formats:");
    for (int i = 0; i < 10; i++) {
        fmtdesc.index = i;
        if (ioctl(video_fd, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
            ESP_LOGI(TAG, "  [%d] 0x%x = %s", i, fmtdesc.pixelformat, fmtdesc.description);
        } else {
            break;
        }
    }

    memset(&cached_fmt, 0, sizeof(cached_fmt));
    cached_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (ioctl(video_fd, VIDIOC_G_FMT, &cached_fmt) < 0) {
        ESP_LOGE(TAG, "VIDIOC_G_FMT failed: %s", strerror(errno));
        close(video_fd);
        return ESP_FAIL;
    }

     ESP_LOGI(TAG, "Final format: %ux%u, pixfmt=0x%x",
             cached_fmt.fmt.pix.width, cached_fmt.fmt.pix.height, cached_fmt.fmt.pix.pixelformat);

    camera_buf_len = cached_fmt.fmt.pix.width * cached_fmt.fmt.pix.height * 2;
    ESP_LOGI(TAG, "Driver reported sizeimage = %zu", camera_buf_len);


    return ESP_OK;
}


static esp_err_t camera_request_and_queue_buffers(void)
{
    struct v4l2_requestbuffers req = {
        .count = CAMERA_BUF_COUNT,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };

    ESP_LOGI(TAG, "Requesting %d buffers, each of length %zu", CAMERA_BUF_COUNT, camera_buf_len);

    if (ioctl(video_fd, VIDIOC_REQBUFS, &req) < 0) {
        ESP_LOGE(TAG, "VIDIOC_REQBUFS failed");
        return ESP_FAIL;
    }

    if (req.count < CAMERA_BUF_COUNT) {
        ESP_LOGW(TAG, "Driver only provided %d buffers (requested %d)", req.count, CAMERA_BUF_COUNT);
    }

    for (int i = 0; i < CAMERA_BUF_COUNT; ++i) {
        struct v4l2_buffer buf = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index = i,
        };

        if (ioctl(video_fd, VIDIOC_QUERYBUF, &buf) < 0) {
            ESP_LOGE(TAG, "VIDIOC_QUERYBUF failed on index %d", i);
            return ESP_FAIL;
        }

        camera_buffers[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, video_fd, buf.m.offset);
        if (camera_buffers[i] == MAP_FAILED) {
            ESP_LOGE(TAG, "mmap failed on buffer %d", i);
            return ESP_FAIL;
        }

        if (ioctl(video_fd, VIDIOC_QBUF, &buf) < 0) {
            ESP_LOGE(TAG, "VIDIOC_QBUF failed on index %d", i);
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}


void camera_capture_cleanup(void)
{
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    // Stop stream if still running
    if (video_fd >= 0) {
        ioctl(video_fd, VIDIOC_STREAMOFF, &type);
    }

    // Unmap buffers
    for (int i = 0; i < CAMERA_BUF_COUNT; ++i) {
        if (camera_buffers[i]) {
            munmap(camera_buffers[i], camera_buf_len);
            camera_buffers[i] = NULL;
        }
    }

    // Close video device
    if (video_fd >= 0) {
        close(video_fd);
        video_fd = -1;
    }

    ESP_LOGI(TAG, "Camera resources cleaned up");
}



esp_err_t esp_cam_capture_frame(uint8_t **image_data, uint32_t *width, uint32_t *height, size_t *len)
{
    if (!cam_handle) {
        ESP_LOGE(TAG, "Camera not initialized");
        return ESP_FAIL;
    }

    esp_cam_ctlr_trans_t trans = {
        .buffer = frame_buffer,
        .buflen = frame_buffer_len,
    };

    ESP_LOGI(TAG, "trans.buffer: %p, buffer len: %zu", trans.buffer, trans.buflen);
    ESP_LOGI(TAG, "Expected resolution: %u x %u, expected frame size: %zu",
             FRAME_WIDTH, FRAME_HEIGHT,
             FRAME_WIDTH * FRAME_HEIGHT * FRAME_BPP);

    ESP_LOGI(TAG, "Receiving frame...");
    if (esp_cam_ctlr_receive(cam_handle, &trans, ESP_CAM_CTLR_MAX_DELAY) != ESP_OK) {
        ESP_LOGE(TAG, "Frame capture failed");
        return ESP_FAIL;
    }

    uint16_t *pixels = (uint16_t*)frame_buffer;
    ESP_LOGI(TAG, "First 8 RAW10 pixels (BGGR pattern):");
    for (int i = 0; i < 2; i++) {  // Process 2 groups of 4 pixels
        uint8_t lsb = frame_buffer[i*5 + 4];  // LSB byte contains 2 bits for each of 4 pixels
        for (int j = 0; j < 4; j++) {
            uint16_t pixel = (frame_buffer[i*5 + j] << 2) | ((lsb >> (6 - j*2)) & 0x03);
            ESP_LOGI("camera_capture", "Pixel %d: 0x%03X (%d)", i*4 + j, pixel, pixel);
        }
    }

    *image_data = frame_buffer;
    *width = FRAME_WIDTH;
    *height = FRAME_HEIGHT;
    *len = trans.buflen;

    ESP_LOGI(TAG, "Captured frame: %ux%u, %u bytes", *width, *height, *len);
    return ESP_OK;
}