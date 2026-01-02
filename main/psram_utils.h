#ifndef PSRAM_UTILS_H
#define PSRAM_UTILS_H

#include <stddef.h>
#include "esp_err.h"

/**
 * @brief Allocate memory from PSRAM with DMA capability
 * 
 * @param size Size in bytes to allocate
 * @param ptr Pointer to store the allocated memory address
 * @return ESP_OK on success, ESP_ERR_NO_MEM on failure
 */
esp_err_t psram_alloc_dma(size_t size, void **ptr);

/**
 * @brief Allocate memory from PSRAM for general use (no DMA requirement)
 * 
 * @param size Size in bytes to allocate
 * @param ptr Pointer to store the allocated memory address
 * @return ESP_OK on success, ESP_ERR_NO_MEM on failure
 */
esp_err_t psram_alloc_general(size_t size, void **ptr);

/**
 * @brief Free memory allocated from PSRAM
 * 
 * @param ptr Pointer to the memory to free
 */
void psram_free(void *ptr);

/**
 * @brief Print PSRAM memory statistics
 */
void psram_print_stats(void);

#endif 