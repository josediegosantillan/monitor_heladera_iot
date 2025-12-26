#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicia la gestión de conectividad.
 * - Gestiona conexión STA (Cliente).
 * - Si falla, levanta AP + Portal Cautivo.
 * - Incluye servidor DNS para Captive Portal (Pop-up automático).
 */
void wifi_portal_init(void);

#ifdef __cplusplus
}
#endif
