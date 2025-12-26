idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       # Agregamos "connectivity" a los requerimientos
                       REQUIRES ac_meter zmct103c ds18b20 connectivity nvs_flash driver esp_timer)
                       