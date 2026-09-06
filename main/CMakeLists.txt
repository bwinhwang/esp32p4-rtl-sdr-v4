idf_component_register(SRCS "usb_host_lib_main.c" "class_driver.c" "esp_libusb.c" "librtlsdr.c" "feed_avr.c" "feed_beast.c" "feed_json.c" "mode-s.c" "net_eth.c" "net_wifi.c" "web_config.c" "shell.c" "plane_cat.c" "tuner_r82xx.c" "tuner_fc2580.c" "tuner_fc0013.c" "tuner_fc0012.c" "tuner_e4k.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES usb esp_driver_gpio esp_timer esp_driver_i2s esp_driver_i2c esp_codec_dev esp_driver_uart esp_eth esp_netif esp_event esp_wifi nvs_flash esp_http_server console esp_app_format lwip esp_psram espressif__ip101
                    )
set_source_files_properties(tuner_r82xx.c "class_driver.c"
    PROPERTIES COMPILE_FLAGS
    "-Wno-unused-function -Wno-unused-variable -Wno-incompatible-pointer-types"
)
