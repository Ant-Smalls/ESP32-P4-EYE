idf_component_register(SRCS "psram_utils.c" "main.c" "camera_init.c" "serial_utils.c"
                    INCLUDE_DIRS "." 
                    REQUIRES "esp_codec_dev" "esp_audio_codec" "nvs_flash" "vfs" "esp_driver_uart" "esp_driver_usb_serial_jtag" "esp_cam_sensor" "esp_video" "esp_http_client"
                    )