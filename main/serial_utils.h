#pragma once

#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

// Define buffer sizes for USB Serial/JTAG
#define USB_SERIAL_JTAG_PACKET_SIZE 1024
#define FRAME_START 0xA5
#define FRAME_END   0x5A

esp_err_t serial_init(void);
esp_err_t serial_send_frame(uint8_t *frame_data, size_t frame_len, uint16_t width, uint16_t height);
