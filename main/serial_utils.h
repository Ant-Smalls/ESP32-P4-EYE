/**
 * @file serial_utils.hh
 * @brief Header file documenting the functions for the serial initalization and communication.
 *
 * @author Anthony Smaldore
 * @date 2025-06-28
 */

#pragma once

#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include <stdbool.h>

// Define buffer sizes for USB Serial/JTAG
#define USB_SERIAL_JTAG_PACKET_SIZE 1024
#define FRAME_START 0xA5
#define FRAME_END   0x5A
#define MAX_CMD_LENGTH 32

/**
 * @brief Initalize usb_serial_jtag and the establish communication with the serial port.
 * @return ESP_OK for success, ESP_FAIL for failure 
 */
esp_err_t serial_init(void);
/**
 * @brief Send the frame data over usb_serial_jtag 
 * @param frame_data Frame image data captured
 * @param frame_len Length of the fram data buffer
 * @param width Pixel width of the image captured 
 * @param height Pixel height of the image captured
 * @return ESP_OK for success, ESP_FAIL for failure 
 */
esp_err_t serial_send_frame(uint8_t *frame_data, size_t frame_len, uint16_t width, uint16_t height);
/**
 * @brief Recieve the incoming serial command to capture a frame or trigger shutdown of the camera
 * @param cmd_buffer Buffer that stores the command to execute 
 * @param buffer_size Size of the buffer storing the command 
 * @return ESP_OK for success, ESP_FAIL for failure 
 */
esp_err_t serial_receive_command(char *cmd_buffer, size_t buffer_size);
/**
 * @brief De-initalize usb_serial_jtag
 * @return ESP_OK for success, ESP_FAIL for failure 
 */
esp_err_t serial_deinit(void);