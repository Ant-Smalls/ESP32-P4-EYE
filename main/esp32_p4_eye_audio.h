#pragma once

#include "esp_err.h"
#include "esp_codec_dev.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t bsp_extra_pdm_codec_init(void);
esp_err_t bsp_extra_pdm_i2s_read(void *audio_buffer, size_t len, size_t *bytes_read, uint32_t timeout_ms);
esp_err_t record_audio_5s(void);
esp_err_t audio_add_wav_header(uint8_t *wav_buf, size_t pcm_len, uint32_t sample_rate);

#ifdef __cplusplus
}
#endif
