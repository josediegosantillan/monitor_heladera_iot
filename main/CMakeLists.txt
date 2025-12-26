idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES ac_meter ds18b20 connectivity mqtt_connector relay_driver zmct103c nvs_flash esp_event esp_adc)
                       