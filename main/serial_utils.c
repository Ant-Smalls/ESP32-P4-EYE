#include "serial_utils.h"
#include "esp_log.h"

static const char *TAG = "serial_utils";

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

     // Clear any pending data
    uint8_t dummy[64];
    int bytes_read;
    // Read with no wait time (0) until no more data
    while ((bytes_read = usb_serial_jtag_read_bytes(dummy, sizeof(dummy), 0)) > 0) {
        // Keep reading until buffer is empty
    }


    ESP_LOGI(TAG, "USB Serial/JTAG initialization successful");
    return ESP_OK;
}

esp_err_t serial_send_frame(uint8_t *frame_data, size_t frame_len, uint16_t width, uint16_t height) {
    if (!frame_data) {
        ESP_LOGE(TAG, "Invalid frame data pointer (NULL)");
        return ESP_ERR_INVALID_ARG;
    }

    
    // Small delay to allow any pending log messages to be sent
    vTaskDelay(pdMS_TO_TICKS(100));

    // Add synchronization marker before header
    const uint8_t sync_marker[] = {0xFF, 0xFF};
    usb_serial_jtag_write_bytes(sync_marker, sizeof(sync_marker), portMAX_DELAY);
    usb_serial_jtag_wait_tx_done(portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));  // Small delay after sync
    
    // Send header with checksum
    uint8_t header[6];  // Increased size for checksum
    header[0] = FRAME_START;
    header[1] = width & 0xFF;
    header[2] = (width >> 8) & 0xFF;
    header[3] = height & 0xFF;
    header[4] = (height >> 8) & 0xFF;
    header[5] = FRAME_END;
    
    // Send header
    int written = usb_serial_jtag_write_bytes(header, sizeof(header), portMAX_DELAY);
    if (written != sizeof(header)) {
        ESP_LOGE(TAG, "Failed to write header: wrote %d of %zu bytes", written, sizeof(header));
        return ESP_FAIL;
    }
    usb_serial_jtag_wait_tx_done(portMAX_DELAY);

    // Send frame data in chunks
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