#ifndef CAMERA_SENSOR_H
#define CAMERA_SENSOR_H

#include "esp_err.h"

#define OV2710_SCCB_ADDR   0x36  // OV2710 I2C address

esp_err_t camera_sensor_init(void);
esp_err_t camera_sensor_detect(void);

#endif // CAMERA_SENSOR_H