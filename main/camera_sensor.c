#include "camera_init.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

static const char *TAG = "camera_sensor";

// OV2710 register definitions
#define OV2710_REG_SENSOR_ID_H    0x300A
#define OV2710_REG_SENSOR_ID_L    0x300B
#define OV2710_CHIP_ID            0x2710

static esp_err_t sensor_write_reg(uint16_t reg, uint8_t value) {
    i2c_master_bus_handle_t i2c_bus;
    ESP_ERROR_CHECK(camera_get_i2c_bus_handle(&i2c_bus));
    
    uint8_t write_buf[3] = {
        (reg >> 8) & 0xFF,   // Register address high byte
        reg & 0xFF,          // Register address low byte
        value                // Value to write
    };
    
    return i2c_master_transmit(i2c_bus, OV2710_SCCB_ADDR, write_buf, sizeof(write_buf), -1);
}

static esp_err_t sensor_read_reg(uint16_t reg, uint8_t *value) {
    i2c_master_bus_handle_t i2c_bus;
    ESP_ERROR_CHECK(camera_get_i2c_bus_handle(&i2c_bus));
    
    uint8_t write_buf[2] = {
        (reg >> 8) & 0xFF,   // Register address high byte
        reg & 0xFF,          // Register address low byte
    };
    
    // Write register address
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_bus, OV2710_SCCB_ADDR, write_buf, sizeof(write_buf), -1));
    
    // Read value
    return i2c_master_receive(i2c_bus, OV2710_SCCB_ADDR, value, 1, -1);
}

esp_err_t camera_sensor_detect(void) {
    uint8_t id_h, id_l;
    ESP_ERROR_CHECK(sensor_read_reg(OV2710_REG_SENSOR_ID_H, &id_h));
    ESP_ERROR_CHECK(sensor_read_reg(OV2710_REG_SENSOR_ID_L, &id_l));
    
    uint16_t sensor_id = (id_h << 8) | id_l;
    ESP_LOGI(TAG, "Sensor ID: 0x%04X", sensor_id);
    
    if (sensor_id != OV2710_CHIP_ID) {
        ESP_LOGE(TAG, "Wrong sensor ID: expected 0x%04X, got 0x%04X", OV2710_CHIP_ID, sensor_id);
        return ESP_ERR_NOT_FOUND;
    }
    
    return ESP_OK;
}

esp_err_t camera_sensor_init(void) {
    // First detect the sensor
    ESP_ERROR_CHECK(camera_sensor_detect());
    
    // Configure for RAW10 output
    ESP_ERROR_CHECK(sensor_write_reg(0x3103, 0x93)); // Power up system clock from PLL
    ESP_ERROR_CHECK(sensor_write_reg(0x3008, 0x82)); // Software reset
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Set output format to RAW10
    ESP_ERROR_CHECK(sensor_write_reg(0x4800, 0x04)); // Clock lane in LP-11 state
    ESP_ERROR_CHECK(sensor_write_reg(0x4837, 0x20)); // MIPI global timing
    ESP_ERROR_CHECK(sensor_write_reg(0x0100, 0x00)); // Stream off
    ESP_ERROR_CHECK(sensor_write_reg(0x3017, 0x00)); // Disable output
    ESP_ERROR_CHECK(sensor_write_reg(0x3018, 0x00)); // Disable output
    
    // Configure for RAW10 BGGR format
    ESP_ERROR_CHECK(sensor_write_reg(0x4300, 0x00)); // RAW10 format
    ESP_ERROR_CHECK(sensor_write_reg(0x501f, 0x01)); // Enable ISP
    
    // Enable streaming
    ESP_ERROR_CHECK(sensor_write_reg(0x0100, 0x01)); // Stream on
    
    ESP_LOGI(TAG, "Camera sensor initialized for RAW10 output");
    return ESP_OK;
}
