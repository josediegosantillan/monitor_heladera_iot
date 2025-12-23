idf_component_register(SRCS "ac_meter.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_adc esp_timer log esp_rom)
                       