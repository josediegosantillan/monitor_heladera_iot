idf_component_register(SRCS "ds18b20.c" # Ej: ac_meter.c
                       INCLUDE_DIRS "include"
                       REQUIRES esp_adc driver)   # Dependencias necesarias