idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES ac_meter zmct103c ds18b20 nvs_flash driver esp_timer relay_driver)
                       