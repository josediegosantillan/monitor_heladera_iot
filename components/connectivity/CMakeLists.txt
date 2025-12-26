idf_component_register(SRCS "wifi_portal.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_wifi esp_event nvs_flash esp_http_server lwip)