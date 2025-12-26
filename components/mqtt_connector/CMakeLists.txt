idf_component_register(SRCS "mqtt_connector.c"
                       INCLUDE_DIRS "include"
                       REQUIRES mqtt esp_timer mbedtls)