#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Frees any allocated resources (buffer, stream).
 */
void camera_capture_cleanup(void);

esp_err_t esp_cam_capture_frame(uint8_t **image_data, uint32_t *width, uint32_t *height, size_t *len);

#ifdef __cplusplus
}
#endif
