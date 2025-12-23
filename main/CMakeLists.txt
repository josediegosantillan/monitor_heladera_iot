idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES ds18b20 zmct103c ac_meter nvs_flash esp_adc driver)
                       