/**
 * @file serial_utils.c
 * @brief Initalize the USB_SERIAL_JTAG line and send the frame data to the receiver
 *
 * @author Anthony Smaldore
 * @date 2025-06-28
 */


#include "serial_utils.h"
#include "esp_log.h"

static const char *TAG = "serial_utils";

/**
 * @brief Initalize usb_serial_jtag and the establish communication with the serial port.
 * Install the usb_serial_jtag driver
 * Clear any existing data in the serial port 
 * @return ESP_OK for success, ESP_FAIL for failure 
 */
esp_err_t serial_init(void) {

    ESP_LOGI(TAG, "Initializing USB Serial/JTAG interface");
    usb_serial_jtag_driver_config_t config = {
        .tx_buffer_size = USB_SERIAL_JTAG_PACKET_SIZE,
        .rx_buffer_size = USB_SERIAL_JTAG_PACKET_SIZE,
    };

    esp_err_t err = usb_serial_jtag_driver_install(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install USB Serial/JTAG driver: %s", esp_err_to_name(err));
        return err;
    }
a
    uint8_t dummy[64];
    int bytes_read;
    while ((bytes_read = usb_serial_jtag_read_bytes(dummy, sizeof(dummy), 0)) > 0) {
        // Read until buffer is empty
    }

    ESP_LOGI(TAG, "USB Serial/JTAG initialization successful");
    return ESP_OK;
}


 /**
 * @brief Send the frame data through usb_serial_jtag to the receiving device 
 * @param frame_data Frame image data captured
 * @param frame_len Length of the fram data buffer
 * @param width Pixel width of the image captured 
 * @param height Pixel height of the image captured
 *
 * Create and send a syncchronization pattern to the reciever 
 * Send the header containing:
 *  - pixel width of frame
 *  - pixel height of frame
 * Send the frame data in chunks and wait for all the frame data to be sent
 * @return ESP_OK for success, ESP_FAIL for failure 
 */
esp_err_t serial_send_frame(uint8_t *frame_data, size_t frame_len, uint16_t width, uint16_t height) {
    if (!frame_data) {
        ESP_LOGE(TAG, "Invalid frame data pointer (NULL)");
        return ESP_ERR_INVALID_ARG;
    }

    // Allow any pending log messages to be sent
    vTaskDelay(pdMS_TO_TICKS(100));

    const uint8_t sync_marker[] = {0xFF, 0xFF};
    usb_serial_jtag_write_bytes(sync_marker, sizeof(sync_marker), portMAX_DELAY);
    usb_serial_jtag_wait_tx_done(portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));  
    
    uint8_t header[6];  
    header[0] = FRAME_START;
    header[1] = width & 0xFF;
    header[2] = (width >> 8) & 0xFF;
    header[3] = height & 0xFF;
    header[4] = (height >> 8) & 0xFF;
    header[5] = FRAME_END;
    
    int written = usb_serial_jtag_write_bytes(header, sizeof(header), portMAX_DELAY);
    if (written != sizeof(header)) {
        ESP_LOGE(TAG, "Failed to write header: wrote %d of %zu bytes", written, sizeof(header));
        return ESP_FAIL;
    }
    usb_serial_jtag_wait_tx_done(portMAX_DELAY);

    const size_t chunk_size = USB_SERIAL_JTAG_PACKET_SIZE;
    size_t remaining = frame_len;
    size_t offset = 0;
    while (remaining > 0) {
        size_t to_write = (remaining > chunk_size) ? chunk_size : remaining;
        int ret = usb_serial_jtag_write_bytes(
            frame_data + offset,
            to_write,
            portMAX_DELAY
        );

        if (ret < 0) {
            ESP_LOGE(TAG, "Failed to write frame data chunk");
            return ESP_FAIL;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
        remaining -= to_write;
        offset += to_write;
    }
    usb_serial_jtag_wait_tx_done(portMAX_DELAY);

    ESP_LOGI(TAG, "Frame sent successfully");
    return ESP_OK;
}

 /**
 * @brief Recieve the incoming serial command to capture a frame or trigger shutdown of the camera
 * @param cmd_buffer Buffer that stores the command to execute 
 * @param buffer_size Size of the buffer storing the command 
 * Read the incoming serial commands 
 * Check for predefined commands, if read exit successfully
 * @return ESP_OK for success, ESP_FAIL for failure 
 */
esp_err_t serial_receive_command(char *cmd_buffer, size_t buffer_size) {
    if (!cmd_buffer || buffer_size < MAX_CMD_LENGTH) {
        ESP_LOGE(TAG, "Invalid buffer or buffer size too small");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t byte;
    enum {
        WAIT_START,
        READING_CMD,
    } state = WAIT_START;

    size_t cmd_idx = 0;
    bool cmd_prefix_found = false;
    const char CMD_PREFIX[] = "CMD:";
    size_t prefix_idx = 0;

    while (1) {
        int bytes_read = usb_serial_jtag_read_bytes(&byte, 1, pdMS_TO_TICKS(100));
        
        if (bytes_read <= 0) {
            continue; 
        }

        switch (state) {
            case WAIT_START:
                if (byte == FRAME_START) {
                    cmd_idx = 0;
                    prefix_idx = 0;
                    cmd_prefix_found = false;
                    state = READING_CMD;
                }
                break;

            case READING_CMD:
                if (!cmd_prefix_found) {
                    if (byte == CMD_PREFIX[prefix_idx]) {
                        prefix_idx++;
                        if (prefix_idx == sizeof(CMD_PREFIX) - 1) { 
                            cmd_prefix_found = true;
                        }
                    } else {
                        state = WAIT_START;
                    }
                } 
                else {
                    if (byte == FRAME_END) {
                        cmd_buffer[cmd_idx] = '\0';  
                        return ESP_OK; 
                    } else if (cmd_idx < buffer_size - 1) {  
                        cmd_buffer[cmd_idx++] = byte;
                    } else {
                        ESP_LOGE(TAG, "Command too long");
                        return ESP_FAIL;
                    }
                }
                break;
        }
    }
}

/**
 * @brief De-initalize usb_serial_jtag
 * Wait for all data to be released from usb_serial_jtag
 * Uninstall the driver for usb_serial_jtag
 * @return ESP_OK for success, ESP_FAIL for failure 
 */
esp_err_t serial_deinit(void) {
    ESP_LOGI(TAG, "Deinitializing USB Serial/JTAG interface");
    if (usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(1000)) != ESP_OK) {
        ESP_LOGW(TAG, "Timeout waiting for TX to complete");
    }

    esp_err_t err = usb_serial_jtag_driver_uninstall();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to uninstall USB Serial/JTAG driver: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "USB Serial/JTAG deinitialization successful");
    return ESP_OK;
}