#include "esp_log.h"
#include "driver/i2s_std.h"
#include "driver/i2s_pdm.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp32_p4_eye_audio.h"
#include "esp_heap_caps.h" 
#include "esp_err.h"
#include "esp_check.h"

#include <string.h>
#include <stdint.h>

#define TAG "audio"
#define SAMPLE_RATE 16000
#define CHANNELS 1
#define BITS_PER_SAMPLE 16
#define RECORD_DURATION_SEC 5

#define BSP_I2S_CLK (GPIO_NUM_22)
#define BSP_I2S_DAT (GPIO_NUM_21)

#define BSP_I2S_PDM_GPIO_CFG     { .clk = BSP_I2S_CLK, .din = BSP_I2S_DAT, .invert_flags = { .clk_inv = false }, }
#define BSP_I2S_PDM_DUPLEX_MONO_CFG(_sample_rate) { \
    .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(_sample_rate), \
    .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO), \
    .gpio_cfg = BSP_I2S_PDM_GPIO_CFG, \
}

static i2s_chan_handle_t i2s_pdm_rx_chan = NULL;
static const audio_codec_data_if_t *i2s_pdm_data_if = NULL;
static esp_codec_dev_handle_t pdm_dev_handle = NULL;

esp_err_t bsp_pdm_audio_init(const i2s_pdm_rx_config_t *i2s_config)
{
    if (i2s_pdm_rx_chan) return ESP_OK;

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &i2s_pdm_rx_chan));

    const i2s_pdm_rx_config_t pdm_cfg_default = BSP_I2S_PDM_DUPLEX_MONO_CFG(16000);
    const i2s_pdm_rx_config_t *cfg = i2s_config ? i2s_config : &pdm_cfg_default;

    ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(i2s_pdm_rx_chan, cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(i2s_pdm_rx_chan));

    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = I2S_NUM_0,
        .rx_handle = i2s_pdm_rx_chan,
        .tx_handle = NULL,
    };
    i2s_pdm_data_if = audio_codec_new_i2s_data(&i2s_cfg);
    assert(i2s_pdm_data_if);

    return ESP_OK;
}

esp_codec_dev_handle_t bsp_audio_pdm_microphone_init(void)
{
    if (i2s_pdm_data_if == NULL) {
        ESP_ERROR_CHECK(bsp_pdm_audio_init(NULL));
    }

    esp_codec_dev_cfg_t codec_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = NULL,
        .data_if = i2s_pdm_data_if,
    };
    return esp_codec_dev_new(&codec_dev_cfg);
}

esp_err_t bsp_extra_pdm_codec_init(void)
{
    pdm_dev_handle = bsp_audio_pdm_microphone_init();
    assert(pdm_dev_handle);

    esp_codec_dev_sample_info_t fmt = {
        .sample_rate = 16000,
        .channel = 2,
        .bits_per_sample = 16,
    };
    return esp_codec_dev_open(pdm_dev_handle, &fmt);
}

esp_err_t bsp_extra_pdm_i2s_read(void *audio_buffer, size_t len, size_t *bytes_read, uint32_t timeout_ms)
{
    esp_err_t ret = esp_codec_dev_read(pdm_dev_handle, audio_buffer, len);
    *bytes_read = len;
    return ret;
}

esp_err_t record_audio_5s(void)
{
    size_t bytes_per_second = SAMPLE_RATE * CHANNELS * (BITS_PER_SAMPLE / 8);
    size_t buffer_size = bytes_per_second * RECORD_DURATION_SEC;

    uint8_t *audio_buffer = heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);

    ESP_LOGI(TAG, "Free internal heap: %d", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "Free SPIRAM heap: %d", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    ESP_RETURN_ON_FALSE(audio_buffer, ESP_ERR_NO_MEM, TAG, "Failed to allocate audio buffer");

    size_t total_read = 0, bytes_read = 0;
    ESP_LOGI(TAG, "Recording %d seconds of audio (%d bytes)", RECORD_DURATION_SEC, buffer_size);

    while (total_read < buffer_size) {
        esp_err_t err = bsp_extra_pdm_i2s_read(audio_buffer + total_read,
                                               buffer_size - total_read,
                                               &bytes_read,
                                               100);
        if (err != ESP_OK || bytes_read == 0) {
            ESP_LOGW(TAG, "Read failed or timed out: %s", esp_err_to_name(err));
            continue;
        }
        total_read += bytes_read;
    }

    ESP_LOGI(TAG, "Recording complete: read %d bytes", total_read);

    // Optional: print first 16 bytes to confirm content
    ESP_LOG_BUFFER_HEXDUMP(TAG, audio_buffer, 16, ESP_LOG_INFO);

    free(audio_buffer);
    return ESP_OK;
}


esp_err_t audio_add_wav_header(uint8_t *wav_buf, size_t pcm_len, uint32_t sample_rate)
{
    // WAV file header = 44 bytes
    uint32_t byte_rate = sample_rate * 1 * 16 / 8;
    uint32_t chunk_size = pcm_len + 36;

    memcpy(wav_buf, "RIFF", 4);
    *(uint32_t *)(wav_buf + 4)  = chunk_size;
    memcpy(wav_buf + 8, "WAVEfmt ", 8);
    *(uint32_t *)(wav_buf + 16) = 16;
    *(uint16_t *)(wav_buf + 20) = 1;
    *(uint16_t *)(wav_buf + 22) = 1;
    *(uint32_t *)(wav_buf + 24) = sample_rate;
    *(uint32_t *)(wav_buf + 28) = byte_rate;
    *(uint16_t *)(wav_buf + 32) = 2;
    *(uint16_t *)(wav_buf + 34) = 16;
    memcpy(wav_buf + 36, "data", 4);
    *(uint32_t *)(wav_buf + 40) = pcm_len;
    ESP_LOGI(TAG, "WAV header added");
    return ESP_OK;
}

