#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicia la gestión de WiFi.
 * Intenta conectar a la red guardada. Si falla, levanta un AP + WebServer.
 */
void wifi_portal_init(void);

#ifdef __cplusplus
}
#endif
