#include "psram_utils.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "psram_utils";

/**
 * @brief Allocate memory from PSRAM with DMA capability
 * 
 * @param size Size in bytes to allocate
 * @param ptr Pointer to store the allocated memory address
 * Allocate the PSRAM memory for DMA 
 *  - If unsuccessful, allocate internal RAM with DMA 
 * @return ESP_OK on success, ESP_ERR_NO_MEM on failure
 */
esp_err_t psram_alloc_dma(size_t size, void **ptr) {

    if (!ptr) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    if (*ptr) {
        ESP_LOGI(TAG, "Allocated %zu bytes from PSRAM with DMA capability", size);
        return ESP_OK;
    }
    
    *ptr = heap_caps_malloc(size, MALLOC_CAP_DMA);
    if (*ptr) {
        ESP_LOGW(TAG, "Allocated %zu bytes from internal RAM (PSRAM DMA allocation failed)", size);
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Failed to allocate %zu bytes from both PSRAM and internal RAM", size);
    return ESP_ERR_NO_MEM;
}
/**
 * @brief Allocate memory from PSRAM for general use (no DMA requirement)
 * Allocate the PSRAM memory (general)
 *  - If unsuccessful, allocate internal RAM (general)
 * @param size Size in bytes to allocate
 * @param ptr Pointer to store the allocated memory address
 * @return ESP_OK on success, ESP_ERR_NO_MEM on failure
 */
esp_err_t psram_alloc_general(size_t size, void **ptr) {

    if (!ptr) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    if (*ptr) {
        ESP_LOGI(TAG, "Allocated %zu bytes from PSRAM", size);
        return ESP_OK;
    }
    
    *ptr = heap_caps_malloc(size, MALLOC_CAP_8BIT);
    if (*ptr) {
        ESP_LOGW(TAG, "Allocated %zu bytes from internal RAM (PSRAM allocation failed)", size);
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Failed to allocate %zu bytes from both PSRAM and internal RAM", size);
    return ESP_ERR_NO_MEM;
}

/**
 * @brief Free memory allocated from PSRAM
 * 
 * @param ptr Pointer to the memory to free
 */
void psram_free(void *ptr) {
    if (ptr) {
        heap_caps_free(ptr);
        ESP_LOGI(TAG, "Freed PSRAM memory");
    }
}

/**
 * @brief Print PSRAM memory statistics
 * Display the following:
 *  - PSRAM memory free
 *  - largest PSRAM memory block free 
 *  - PSRAM DMA memory free 
 *  - largest DMA capabkle memory block free
 */
void psram_print_stats(void) {
    size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t psram_largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    size_t psram_dma_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    size_t psram_dma_largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    
    ESP_LOGI(TAG, "PSRAM Statistics:");
    ESP_LOGI(TAG, "  Total free: %zu bytes (%.2f MB)", psram_free, psram_free / (1024.0 * 1024.0));
    ESP_LOGI(TAG, "  Largest free block: %zu bytes (%.2f MB)", psram_largest, psram_largest / (1024.0 * 1024.0));
    ESP_LOGI(TAG, "  DMA-capable free: %zu bytes (%.2f MB)", psram_dma_free, psram_dma_free / (1024.0 * 1024.0));
    ESP_LOGI(TAG, "  DMA-capable largest block: %zu bytes (%.2f MB)", psram_dma_largest, psram_dma_largest / (1024.0 * 1024.0));
} 