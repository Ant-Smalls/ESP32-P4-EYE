#include "psram_utils.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "psram_utils";

esp_err_t psram_alloc_dma(size_t size, void **ptr) {
    if (!ptr) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Try PSRAM with DMA capability first
    *ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    if (*ptr) {
        ESP_LOGI(TAG, "Allocated %zu bytes from PSRAM with DMA capability", size);
        return ESP_OK;
    }
    
    // Fallback to internal RAM with DMA
    *ptr = heap_caps_malloc(size, MALLOC_CAP_DMA);
    if (*ptr) {
        ESP_LOGW(TAG, "Allocated %zu bytes from internal RAM (PSRAM DMA allocation failed)", size);
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Failed to allocate %zu bytes from both PSRAM and internal RAM", size);
    return ESP_ERR_NO_MEM;
}

esp_err_t psram_alloc_general(size_t size, void **ptr) {
    if (!ptr) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Try PSRAM first
    *ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    if (*ptr) {
        ESP_LOGI(TAG, "Allocated %zu bytes from PSRAM", size);
        return ESP_OK;
    }
    
    // Fallback to internal RAM
    *ptr = heap_caps_malloc(size, MALLOC_CAP_8BIT);
    if (*ptr) {
        ESP_LOGW(TAG, "Allocated %zu bytes from internal RAM (PSRAM allocation failed)", size);
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Failed to allocate %zu bytes from both PSRAM and internal RAM", size);
    return ESP_ERR_NO_MEM;
}

esp_err_t psram_alloc_audio(size_t size, void **ptr) {
    if (!ptr) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // For audio, we might need DMA capability depending on the audio interface
    // Try PSRAM with DMA first
    *ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    if (*ptr) {
        ESP_LOGI(TAG, "Allocated %zu bytes for audio from PSRAM with DMA", size);
        return ESP_OK;
    }
    
    // Fallback to PSRAM without DMA
    *ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    if (*ptr) {
        ESP_LOGI(TAG, "Allocated %zu bytes for audio from PSRAM", size);
        return ESP_OK;
    }
    
    // Final fallback to internal RAM
    *ptr = heap_caps_malloc(size, MALLOC_CAP_8BIT);
    if (*ptr) {
        ESP_LOGW(TAG, "Allocated %zu bytes for audio from internal RAM (PSRAM allocation failed)", size);
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Failed to allocate %zu bytes for audio from any memory type", size);
    return ESP_ERR_NO_MEM;
}

void psram_free(void *ptr) {
    if (ptr) {
        heap_caps_free(ptr);
        ESP_LOGI(TAG, "Freed PSRAM memory");
    }
}

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