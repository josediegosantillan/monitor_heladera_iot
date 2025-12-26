#include "wifi_portal.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "lwip/sockets.h"
#include "lwip/err.h"
#include "lwip/sys.h"

static const char *TAG = "WIFI_PORTAL";

// Configuración
#define AP_SSID         "HELADERA_SETUP"
#define AP_PASS         "123456789" // Debería ir a Kconfig en prod
#define MAX_RETRY       3
#define DNS_PORT        53

// Buffers y Límites
#define MAX_HTTP_RECV_BUF 512
#define MAX_SSID_LEN      32
#define MAX_PASS_LEN      64
#define HTTP_TIMEOUT_SEC  10 // Aumentado para soportar Scan bloqueante

static int s_retry_num = 0;
static httpd_handle_t s_server = NULL;
static TaskHandle_t s_dns_task_handle = NULL;
static volatile int s_dns_socket_fd = -1; 
static volatile bool s_dns_running = false; // Flag de control

// Prototipos
static void stop_softap_provisioning(void);
static void start_softap_provisioning(void);

// ==========================================================
// 🛠️ UTILIDADES SEGURAS
// ==========================================================

// Helper para asegurar lectura completa del body POST
static int http_recv_full(httpd_req_t *req, char *buf, size_t total_len) {
    size_t received = 0;
    while (received < total_len) {
        int ret = httpd_req_recv(req, buf + received, total_len - received);
        if (ret <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                continue; // Reintentar si es solo timeout temporal
            }
            return -1; // Error fatal o cierre
        }
        received += ret;
    }
    return received;
}

static int url_decode(const char *src, char *dst, size_t dst_len) {
    size_t s_len = strlen(src);
    size_t d_idx = 0;
    for (size_t i = 0; i < s_len; i++) {
        if (d_idx >= dst_len - 1) return -1; 
        if (src[i] == '+') {
            dst[d_idx++] = ' ';
        } else if (src[i] == '%' && i + 2 < s_len && isxdigit((int)src[i+1]) && isxdigit((int)src[i+2])) {
            char hex[3] = { src[i+1], src[i+2], '\0' };
            dst[d_idx++] = (char)strtol(hex, NULL, 16);
            i += 2;
        } else {
            dst[d_idx++] = src[i];
        }
    }
    dst[d_idx] = '\0';
    return 0;
}

// ==========================================================
// 🌐 SERVIDOR DNS (Hardened)
// ==========================================================
static void dns_server_task(void *pvParameters) {
    uint8_t data[128];
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    socklen_t socklen = sizeof(client_addr);
    
    s_dns_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (s_dns_socket_fd < 0) {
        ESP_LOGE(TAG, "DNS: Error creando socket");
        vTaskDelete(NULL);
    }

    // Timeout para permitir verificar flag de salida
    struct timeval tv;
    tv.tv_sec = 2; tv.tv_usec = 0;
    setsockopt(s_dns_socket_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(DNS_PORT);

    if (bind(s_dns_socket_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "DNS: Error en bind");
        close(s_dns_socket_fd);
        s_dns_socket_fd = -1;
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "🛡️ DNS Server iniciado (Port 53)");
    s_dns_running = true;

    while (s_dns_running) {
        int len = recvfrom(s_dns_socket_fd, data, sizeof(data), 0, (struct sockaddr *)&client_addr, &socklen);
        
        // Si len < 0 es error o timeout. Si cerramos el socket desde afuera, len será -1 y errno EBADF o similar.
        if (len < 0) continue; 
        if (!s_dns_running) break; // Doble check

        if (len > 12) { // Mínimo header DNS
            // Hijacking: Responder siempre con nuestra IP
            data[2] = 0x81; data[3] = 0x80; // Flags Response
            data[6] = 0x00; data[7] = 0x01; // Answer RRs = 1
            data[8] = 0x00; data[9] = 0x00;
            data[10] = 0x00; data[11] = 0x00;

            // --- PARSING SEGURO DEL QNAME ---
            int idx = 12; 
            bool safe = true;
            
            // Recorremos los labels (ej: [3]www[6]google[3]com[0])
            while (idx < len) {
                uint8_t label_len = data[idx];
                if (label_len == 0) { // Fin del nombre
                    idx++; // Saltar el 0x00
                    break;
                }
                // Verificar que no nos salgamos del buffer
                if (idx + label_len + 1 >= len) {
                    safe = false;
                    break;
                }
                idx += label_len + 1;
            }
            
            // Saltar QTYPE (2) y QCLASS (2)
            idx += 4;

            // Verificar espacio para nuestra respuesta (16 bytes)
            // Header(12) + Query(...) + Answer(16) <= 128
            if (safe && idx + 16 <= sizeof(data)) {
                // Puntero al nombre (Compression pointer 0xC00C)
                data[idx++] = 0xC0; data[idx++] = 0x0C;
                data[idx++] = 0x00; data[idx++] = 0x01; // Type A
                data[idx++] = 0x00; data[idx++] = 0x01; // Class IN
                data[idx++] = 0x00; data[idx++] = 0x00; // TTL
                data[idx++] = 0x00; data[idx++] = 0x3C; // 60s
                data[idx++] = 0x00; data[idx++] = 0x04; // Len 4
                data[idx++] = 192; data[idx++] = 168; data[idx++] = 4; data[idx++] = 1;

                sendto(s_dns_socket_fd, data, idx, 0, (struct sockaddr *)&client_addr, socklen);
            }
        }
    }

    // Salida limpia
    ESP_LOGI(TAG, "DNS Server detenido (Tarea finalizada)");
    if (s_dns_socket_fd >= 0) {
        close(s_dns_socket_fd);
        s_dns_socket_fd = -1;
    }
    s_dns_task_handle = NULL;
    vTaskDelete(NULL);
}

static void stop_dns_server(void) {
    if (s_dns_running) {
        ESP_LOGI(TAG, "Deteniendo DNS...");
        s_dns_running = false;
        // Cerrar socket para despertar al recvfrom bloqueado
        if (s_dns_socket_fd >= 0) {
            shutdown(s_dns_socket_fd, 2); // Shutdown R/W
            close(s_dns_socket_fd);
            s_dns_socket_fd = -1;
        }
        // Damos un tiempo para que la tarea muera sola
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ==========================================================
// 📄 HTML & WEB SERVER
// ==========================================================

static const char *HTML_HEAD = 
"<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>"
"<title>Heladera IoT</title>"
"<style>"
"body{font-family:'Segoe UI',Helvetica,sans-serif;padding:20px;background:#1a1a1a;color:#f0f0f0}"
"input,select,button{width:100%;padding:12px;margin:8px 0;box-sizing:border-box;border-radius:6px;border:none;font-size:16px}"
"input,select{background:#333;color:#fff;border:1px solid #444}"
"button{background-color:#007bff;color:white;font-weight:bold;cursor:pointer;transition:0.3s}"
"button:hover{background-color:#0056b3}"
"h2{text-align:center;color:#fff;margin-bottom:20px}"
".card{background:#2d2d2d;padding:25px;border-radius:12px;box-shadow:0 4px 15px rgba(0,0,0,0.5);max-width:400px;margin:auto}"
"</style>"
"<script>"
"function copySSID() { var sel = document.getElementById('scan_result'); var input = document.getElementById('ssid'); if(sel.value !== '') input.value = sel.value; }"
"</script>"
"</head><body><div class='card'><h2>❄️ Heladera IoT</h2>";

static const char *HTML_FORM_START = "<form action='/save' method='post'><label>Redes Detectadas:</label><select id='scan_result' onchange='copySSID()'>";
static const char *HTML_FORM_END = "</select><br><a href='/scan' style='color:#4da3ff'>🔄 Recargar Lista</a><br><br><label>SSID:</label><input type='text' id='ssid' name='ssid'><label>Password:</label><input type='password' name='pass'><button type='submit'>Guardar</button></form></div></body></html>";

static char* perform_wifi_scan_safe(void) {
    wifi_scan_config_t scan_config = { .show_hidden = true };
    // NOTA: Bloqueante. Si hay muchos APs o canales ruidosos, puede tardar 2-3s.
    // El watchdog HTTP debe estar configurado acorde.
    esp_err_t err = esp_wifi_scan_start(&scan_config, true);
    if (err != ESP_OK) return NULL;

    uint16_t ap_num = 0;
    esp_wifi_scan_get_ap_num(&ap_num);
    if (ap_num > 15) ap_num = 15; // Límite duro

    wifi_ap_record_t *ap_records = (wifi_ap_record_t *)calloc(ap_num, sizeof(wifi_ap_record_t));
    if (!ap_records) return NULL;

    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_num, ap_records));

    size_t buf_size = (ap_num * 128) + 512;
    char *html_out = (char *)calloc(1, buf_size);
    if (!html_out) { free(ap_records); return NULL; }

    char *ptr = html_out;
    size_t remaining = buf_size;
    int w;

    w = snprintf(ptr, remaining, "<option value='' disabled selected>✅ %d Redes (Click para copiar)</option>", ap_num);
    if (w > 0 && w < remaining) { ptr += w; remaining -= w; }

    for (int i = 0; i < ap_num; i++) {
        if (strlen((char *)ap_records[i].ssid) == 0) continue;
        // Se asume SSID seguro para el contexto (sin < >). Para mayor seguridad usar escape_html.
        w = snprintf(ptr, remaining, "<option value='%s'>%s (%d dBm)</option>", 
                     (char *)ap_records[i].ssid, (char *)ap_records[i].ssid, ap_records[i].rssi);
        if (w > 0 && w < remaining) { ptr += w; remaining -= w; }
        else break;
    }

    free(ap_records);
    return html_out;
}

static esp_err_t scan_get_handler(httpd_req_t *req) {
    char *list = perform_wifi_scan_safe();
    httpd_resp_send_chunk(req, HTML_HEAD, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, HTML_FORM_START, HTTPD_RESP_USE_STRLEN);
    if (list) {
        httpd_resp_send_chunk(req, list, HTTPD_RESP_USE_STRLEN);
        free(list);
    } else {
        httpd_resp_send_chunk(req, "<option>Error al escanear</option>", HTTPD_RESP_USE_STRLEN);
    }
    httpd_resp_send_chunk(req, HTML_FORM_END, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t root_get_handler(httpd_req_t *req) {
    httpd_resp_send_chunk(req, HTML_HEAD, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, HTML_FORM_START, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, "<option>-- Presione 'Recargar Lista' --</option>", HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, HTML_FORM_END, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t save_post_handler(httpd_req_t *req) {
    char buf[MAX_HTTP_RECV_BUF]; 
    int remaining = req->content_len;

    if (remaining >= sizeof(buf)) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    // --- FIX: LECTURA COMPLETA ---
    memset(buf, 0, sizeof(buf));
    int received = http_recv_full(req, buf, remaining);
    if (received <= 0) {
        ESP_LOGE(TAG, "Error recibiendo POST body incompleto");
        return ESP_FAIL;
    }
    // -----------------------------

    char ssid_raw[MAX_SSID_LEN * 3] = {0};
    char pass_raw[MAX_PASS_LEN * 3] = {0};
    
    char *p_ssid = strstr(buf, "ssid=");
    char *p_pass = strstr(buf, "pass=");

    if (p_ssid) {
        p_ssid += 5;
        char *end = strchr(p_ssid, '&');
        size_t len = end ? (size_t)(end - p_ssid) : strlen(p_ssid);
        if (len >= sizeof(ssid_raw)) len = sizeof(ssid_raw) - 1;
        strncpy(ssid_raw, p_ssid, len);
    }

    if (p_pass) {
        p_pass += 5;
        size_t len = strlen(p_pass);
        if (len >= sizeof(pass_raw)) len = sizeof(pass_raw) - 1;
        strncpy(pass_raw, p_pass, len);
    }

    char ssid_clean[MAX_SSID_LEN + 1] = {0};
    char pass_clean[MAX_PASS_LEN + 1] = {0};

    if (url_decode(ssid_raw, ssid_clean, sizeof(ssid_clean)) != 0 ||
        url_decode(pass_raw, pass_clean, sizeof(pass_clean)) != 0) {
        ESP_LOGE(TAG, "Decoding fallido");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    if (strlen(ssid_clean) > 0) {
        ESP_LOGI(TAG, "Guardando SSID: %s", ssid_clean);
        nvs_handle_t h;
        if (nvs_open("storage", NVS_READWRITE, &h) == ESP_OK) {
            nvs_set_str(h, "wifi_ssid", ssid_clean);
            nvs_set_str(h, "wifi_pass", pass_clean);
            nvs_commit(h);
            nvs_close(h);
        }
        httpd_resp_send(req, "<h1>Guardado. Reiniciando...</h1>", HTTPD_RESP_USE_STRLEN);
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    } else {
        httpd_resp_send(req, "Error: SSID Vacio", HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

static esp_err_t captive_portal_handler(httpd_req_t *req) {
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static void stop_webserver(void) {
    if (s_server) {
        ESP_LOGI(TAG, "Deteniendo WebServer...");
        httpd_stop(s_server);
        s_server = NULL;
    }
}

static void start_webserver(void) {
    stop_webserver(); 

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.max_uri_handlers = 12;
    // --- FIX: Aumentar timeout para soportar Scan Bloqueante ---
    config.recv_wait_timeout = HTTP_TIMEOUT_SEC;
    config.send_wait_timeout = HTTP_TIMEOUT_SEC;
    // -----------------------------------------------------------
    config.uri_match_fn = httpd_uri_match_wildcard;

    if (httpd_start(&s_server, &config) == ESP_OK) {
        httpd_uri_t root = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
        httpd_register_uri_handler(s_server, &root);

        httpd_uri_t scan = { .uri = "/scan", .method = HTTP_GET, .handler = scan_get_handler };
        httpd_register_uri_handler(s_server, &scan);

        httpd_uri_t save = { .uri = "/save", .method = HTTP_POST, .handler = save_post_handler };
        httpd_register_uri_handler(s_server, &save);

        httpd_uri_t catch_all = { .uri = "*", .method = HTTP_GET, .handler = captive_portal_handler };
        httpd_register_uri_handler(s_server, &catch_all);
    }
}

// ==========================================================
// 🚀 GESTIÓN
// ==========================================================

static void stop_softap_provisioning(void) {
    stop_dns_server();
    stop_webserver();
}

static void start_softap_provisioning(void) {
    ESP_LOGW(TAG, "Iniciando AP Provisioning");
    stop_softap_provisioning();

    esp_wifi_stop();
    esp_wifi_set_mode(WIFI_MODE_APSTA);
    
    wifi_config_t ap_config = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = strlen(AP_SSID),
            .password = AP_PASS,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .channel = 1
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    
    wifi_config_t sta_config = { .sta = { .scan_method = WIFI_FAST_SCAN } };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));

    ESP_ERROR_CHECK(esp_wifi_start());

    start_webserver();
    xTaskCreate(dns_server_task, "dns_server", 4096, NULL, 5, &s_dns_task_handle); // Más stack por seguridad
}

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Reintento %d/%d...", s_retry_num, MAX_RETRY);
        } else {
            wifi_mode_t mode;
            esp_wifi_get_mode(&mode);
            if (mode != WIFI_MODE_APSTA) {
                ESP_LOGE(TAG, "Fallo conexion. Iniciando AP.");
                start_softap_provisioning();
            }
        }
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
             ESP_LOGI(TAG, "✅ Conectado a: %s", (char*)ap_info.ssid);
        }
        ESP_LOGI(TAG, "📍 IP: " IPSTR, IP2STR(&event->ip_info.ip));
        
        s_retry_num = 0;
        stop_softap_provisioning(); // Apagar portal
    }
}

void wifi_portal_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));

    char ssid[33] = {0}; char pass[65] = {0};
    size_t len;
    bool has_config = false;
    
    nvs_handle_t h;
    if (nvs_open("storage", NVS_READONLY, &h) == ESP_OK) {
        len = sizeof(ssid);
        if (nvs_get_str(h, "wifi_ssid", ssid, &len) == ESP_OK) {
            len = sizeof(pass);
            nvs_get_str(h, "wifi_pass", pass, &len);
            if (strlen(ssid) > 0) has_config = true;
        }
        nvs_close(h);
    }

    if (has_config) {
        ESP_LOGI(TAG, "Config: %s. Conectando...", ssid);
        wifi_config_t wifi_config = {0};
        strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
        strncpy((char*)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password));
        
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());
    } else {
        start_softap_provisioning();
    }
}