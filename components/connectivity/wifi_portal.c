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
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "sdkconfig.h"

static const char *TAG = "WIFI_PORTAL";

// Configuración Password
#ifdef CONFIG_ESP_WIFI_PASSWORD
    #define AP_PASS_CONFIG CONFIG_ESP_WIFI_PASSWORD
#else
    #define AP_PASS_CONFIG "123456789" 
#endif

#define AP_SSID         "HELADERA_SETUP"
#define MAX_RETRY       3
#define DNS_PORT        53
#define HTTP_TIMEOUT_SEC 10

// Buffers y Límites
#define MAX_HTTP_RECV_BUF 512
#define MAX_SSID_LEN      32
#define MAX_PASS_LEN      64

static int s_retry_num = 0;
static httpd_handle_t s_server = NULL;
static TaskHandle_t s_dns_task_handle = NULL;
static volatile int s_dns_socket_fd = -1; 
static volatile bool s_dns_running = false;

// Prototipos
static void stop_softap_provisioning(void);
static void start_softap_provisioning(void);

// ==========================================================
// 🛠️ UTILIDADES SEGURAS
// ==========================================================

static void safe_strcpy(char *dst, const char *src, size_t dst_size) {
    if (dst_size == 0) return;
    strncpy(dst, src, dst_size - 1);
    dst[dst_size - 1] = '\0';
}

// CORRECCIÓN: Límite de reintentos para evitar bucle infinito
static int http_recv_full(httpd_req_t *req, char *buf, size_t total_len) {
    size_t received = 0;
    int retries = 0;
    const int MAX_RETRIES = 5;

    while (received < total_len) {
        int ret = httpd_req_recv(req, buf + received, total_len - received);
        if (ret <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                retries++;
                if (retries >= MAX_RETRIES) {
                    ESP_LOGW(TAG, "HTTP Timeout acumulado. Abortando.");
                    return -1;
                }
                continue; 
            }
            return -1; 
        }
        received += ret;
        retries = 0; 
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

static void escape_html(const char *src, char *dst, size_t dst_len) {
    size_t d_idx = 0;
    while (*src && d_idx < dst_len - 1) {
        const char *entity = NULL;
        switch (*src) {
            case '<': entity = "&lt;"; break;
            case '>': entity = "&gt;"; break;
            case '&': entity = "&amp;"; break;
            case '"': entity = "&quot;"; break;
            case '\'': entity = "&#39;"; break;
            default: dst[d_idx++] = *src; break;
        }
        if (entity) {
            size_t ent_len = strlen(entity);
            if (d_idx + ent_len < dst_len - 1) {
                strcpy(dst + d_idx, entity);
                d_idx += ent_len;
            } else { break; }
        }
        src++;
    }
    dst[d_idx] = '\0';
}

// ==========================================================
// 🌐 SERVIDOR DNS (Hardened & Dinámico)
// ==========================================================
static void dns_server_task(void *pvParameters) {
    uint8_t data[128];
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    socklen_t socklen = sizeof(client_addr);
    
    s_dns_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (s_dns_socket_fd < 0) {
        vTaskDelete(NULL);
    }

    struct timeval tv;
    tv.tv_sec = 2; tv.tv_usec = 0;
    setsockopt(s_dns_socket_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(DNS_PORT);

    if (bind(s_dns_socket_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        close(s_dns_socket_fd);
        s_dns_socket_fd = -1;
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "🛡️ DNS Server iniciado");
    s_dns_running = true;

    while (s_dns_running) {
        int len = recvfrom(s_dns_socket_fd, data, sizeof(data), 0, (struct sockaddr *)&client_addr, &socklen);
        if (len < 0) continue; 
        if (!s_dns_running) break;

        if (len > 12) { 
            data[2] = 0x81; data[3] = 0x80; 
            data[6] = 0x00; data[7] = 0x01; 
            data[8] = 0x00; data[9] = 0x00;
            data[10] = 0x00; data[11] = 0x00;

            int idx = 12; 
            bool safe = true;
            while (idx < len) {
                uint8_t label_len = data[idx];
                if (label_len == 0) { idx++; break; }
                if (idx + label_len + 1 >= len) { safe = false; break; }
                idx += label_len + 1;
            }
            idx += 4;

            if (safe && idx + 16 <= sizeof(data)) {
                data[idx++] = 0xC0; data[idx++] = 0x0C;
                data[idx++] = 0x00; data[idx++] = 0x01; 
                data[idx++] = 0x00; data[idx++] = 0x01; 
                data[idx++] = 0x00; data[idx++] = 0x00; 
                data[idx++] = 0x00; data[idx++] = 0x3C; 
                data[idx++] = 0x00; data[idx++] = 0x04; 
                
                // CORRECCIÓN: Obtener IP real del AP
                uint32_t ip_addr = 0;
                esp_netif_t* netif_ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
                if (netif_ap) {
                    esp_netif_ip_info_t ip_info;
                    esp_netif_get_ip_info(netif_ap, &ip_info);
                    ip_addr = ip_info.ip.addr;
                } else {
                    IP4_ADDR((esp_ip4_addr_t*)&ip_addr, 192, 168, 4, 1); // Fallback
                }
                
                uint8_t* ip_bytes = (uint8_t*)&ip_addr;
                data[idx++] = ip_bytes[0];
                data[idx++] = ip_bytes[1];
                data[idx++] = ip_bytes[2];
                data[idx++] = ip_bytes[3];

                sendto(s_dns_socket_fd, data, idx, 0, (struct sockaddr *)&client_addr, socklen);
            }
        }
    }

    if (s_dns_socket_fd >= 0) {
        close(s_dns_socket_fd);
        s_dns_socket_fd = -1;
    }
    s_dns_task_handle = NULL;
    vTaskDelete(NULL);
}

static void stop_dns_server(void) {
    if (s_dns_running) {
        s_dns_running = false;
        if (s_dns_socket_fd >= 0) {
            shutdown(s_dns_socket_fd, 2); 
            close(s_dns_socket_fd);
            s_dns_socket_fd = -1;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ==========================================================
// 📄 HTML & WEB SERVER
// ==========================================================

static const char *HTML_HEAD = "<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'><title>Heladera IoT</title><style>body{font-family:'Segoe UI',sans-serif;padding:20px;background:#1a1a1a;color:#f0f0f0}input,select,button{width:100%;padding:12px;margin:8px 0;border-radius:6px;border:none;font-size:16px}input,select{background:#333;color:#fff;border:1px solid #444}button{background-color:#007bff;color:white;font-weight:bold;cursor:pointer}.btn-danger{background-color:#dc3545;margin-top:20px}h2{text-align:center;color:#fff}.card{background:#2d2d2d;padding:25px;border-radius:12px;max-width:400px;margin:auto}</style><script>function copySSID(){var e=document.getElementById('scan_result');var n=document.getElementById('ssid');''!==e.value&&(n.value=e.value)}function confirmReset(){return confirm('¿Seguro que querés borrar las claves?')}</script></head><body><div class='card'><h2>❄️ Heladera IoT</h2>";
static const char *HTML_FORM_START = "<form action='/save' method='post'><label>Redes Detectadas:</label><select id='scan_result' onchange='copySSID()'>";
static const char *HTML_FORM_END = "</select><br><a href='/scan' style='color:#4da3ff'>🔄 Recargar Lista</a><br><br><label>SSID:</label><input type='text' id='ssid' name='ssid'><label>Password:</label><input type='password' name='pass'><button type='submit'>💾 Guardar y Conectar</button></form>";
static const char *HTML_RESET_BTN = "<form action='/reset' method='post' onsubmit='return confirmReset()'><button type='submit' class='btn-danger'>⚠️ Borrar Credenciales</button></form></div></body></html>";

static char* perform_wifi_scan_safe(void) {
    wifi_scan_config_t scan_config = { .show_hidden = true };
    esp_err_t err = esp_wifi_scan_start(&scan_config, true);
    if (err != ESP_OK) return NULL;

    uint16_t ap_num = 0;
    esp_wifi_scan_get_ap_num(&ap_num);
    
    // CORRECCIÓN: Manejar 0 redes
    if (ap_num == 0) return strdup("<option value='' disabled>⚠️ No se encontraron redes</option>");
    if (ap_num > 15) ap_num = 15;

    wifi_ap_record_t *ap_records = (wifi_ap_record_t *)calloc(ap_num, sizeof(wifi_ap_record_t));
    if (!ap_records) return NULL;

    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_num, ap_records));

    size_t buf_size = (ap_num * 200) + 512;
    char *html_out = (char *)calloc(1, buf_size);
    if (!html_out) { free(ap_records); return NULL; }

    char *ptr = html_out;
    size_t remaining = buf_size;
    int w = snprintf(ptr, remaining, "<option value='' disabled selected>✅ %d Redes (Click para copiar)</option>", ap_num);
    if (w > 0) { ptr += w; remaining -= w; }

    char ssid_escaped[MAX_SSID_LEN * 6];
    for (int i = 0; i < ap_num; i++) {
        if (strlen((char *)ap_records[i].ssid) == 0) continue;
        escape_html((char *)ap_records[i].ssid, ssid_escaped, sizeof(ssid_escaped));
        w = snprintf(ptr, remaining, "<option value='%s'>%s (%d dBm)</option>", ssid_escaped, ssid_escaped, ap_records[i].rssi);
        if (w > 0 && w < remaining) { ptr += w; remaining -= w; } else break;
    }
    free(ap_records);
    return html_out;
}

static esp_err_t scan_get_handler(httpd_req_t *req) {
    char *list = perform_wifi_scan_safe();
    httpd_resp_send_chunk(req, HTML_HEAD, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, HTML_FORM_START, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, list ? list : "<option>Error al escanear</option>", HTTPD_RESP_USE_STRLEN);
    if (list) free(list);
    httpd_resp_send_chunk(req, HTML_FORM_END, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, HTML_RESET_BTN, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t root_get_handler(httpd_req_t *req) {
    httpd_resp_send_chunk(req, HTML_HEAD, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, HTML_FORM_START, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, "<option>-- Presione 'Recargar Lista' --</option>", HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, HTML_FORM_END, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, HTML_RESET_BTN, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t save_post_handler(httpd_req_t *req) {
    char buf[MAX_HTTP_RECV_BUF]; 
    int remaining = req->content_len;
    if (remaining >= sizeof(buf)) { httpd_resp_send_500(req); return ESP_FAIL; }
    
    memset(buf, 0, sizeof(buf));
    if (http_recv_full(req, buf, remaining) <= 0) {
        httpd_resp_send_500(req); return ESP_FAIL;
    }

    char ssid_raw[MAX_SSID_LEN * 3] = {0};
    char pass_raw[MAX_PASS_LEN * 3] = {0};
    char *p_ssid = strstr(buf, "ssid=");
    char *p_pass = strstr(buf, "pass=");
    
    if (p_ssid) {
        p_ssid += 5;
        char *end = strchr(p_ssid, '&');
        size_t len = end ? (size_t)(end - p_ssid) : strlen(p_ssid);
        safe_strcpy(ssid_raw, p_ssid, (len >= sizeof(ssid_raw)) ? sizeof(ssid_raw) : len + 1);
    }
    if (p_pass) {
        p_pass += 5;
        safe_strcpy(pass_raw, p_pass, sizeof(pass_raw));
    }

    char ssid_clean[MAX_SSID_LEN + 1] = {0};
    char pass_clean[MAX_PASS_LEN + 1] = {0};

    if (url_decode(ssid_raw, ssid_clean, sizeof(ssid_clean)) != 0 ||
        url_decode(pass_raw, pass_clean, sizeof(pass_clean)) != 0) {
        httpd_resp_send_500(req); return ESP_FAIL;
    }

    if (strlen(ssid_clean) > 0) {
        nvs_handle_t h;
        esp_err_t err = nvs_open("storage", NVS_READWRITE, &h);
        if (err == ESP_OK) {
            nvs_set_str(h, "wifi_ssid", ssid_clean);
            nvs_set_str(h, "wifi_pass", pass_clean);
            nvs_commit(h);
            nvs_close(h);
            httpd_resp_send(req, "<h1>Guardado. Reiniciando...</h1>", HTTPD_RESP_USE_STRLEN);
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_restart();
        } else {
            httpd_resp_send(req, "<h1 style='color:red'>Error NVS</h1>", HTTPD_RESP_USE_STRLEN);
        }
    } else {
        httpd_resp_send(req, "Error: SSID Vacio", HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

static esp_err_t reset_post_handler(httpd_req_t *req) {
    nvs_handle_t h;
    if (nvs_open("storage", NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_all(h); nvs_commit(h); nvs_close(h);
    }
    httpd_resp_send(req, "<h1 style='color:red'>Borrando...</h1>", HTTPD_RESP_USE_STRLEN);
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return ESP_OK;
}

static esp_err_t captive_portal_handler(httpd_req_t *req) {
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static void stop_webserver(void) {
    if (s_server) { httpd_stop(s_server); s_server = NULL; }
}

static void start_webserver(void) {
    stop_webserver(); 
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.max_uri_handlers = 12;
    config.recv_wait_timeout = HTTP_TIMEOUT_SEC;
    config.send_wait_timeout = HTTP_TIMEOUT_SEC;
    config.uri_match_fn = httpd_uri_match_wildcard;

    if (httpd_start(&s_server, &config) == ESP_OK) {
        httpd_uri_t root = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
        httpd_register_uri_handler(s_server, &root);
        httpd_uri_t scan = { .uri = "/scan", .method = HTTP_GET, .handler = scan_get_handler };
        httpd_register_uri_handler(s_server, &scan);
        httpd_uri_t save = { .uri = "/save", .method = HTTP_POST, .handler = save_post_handler };
        httpd_register_uri_handler(s_server, &save);
        httpd_uri_t reset = { .uri = "/reset", .method = HTTP_POST, .handler = reset_post_handler };
        httpd_register_uri_handler(s_server, &reset);
        httpd_uri_t catch_all = { .uri = "*", .method = HTTP_GET, .handler = captive_portal_handler };
        httpd_register_uri_handler(s_server, &catch_all);
    }
}

static void stop_softap_provisioning(void) {
    stop_dns_server();
    stop_webserver();
}

static void start_softap_provisioning(void) {
    ESP_LOGW(TAG, "Iniciando AP Provisioning");
    stop_softap_provisioning();

    // Ignorar error si WiFi no estaba iniciado
    esp_err_t err = esp_wifi_stop();
    if (err != ESP_ERR_WIFI_NOT_INIT && err != ESP_ERR_WIFI_NOT_STARTED) {
        ESP_ERROR_CHECK(err);
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    
    wifi_auth_mode_t auth_mode = WIFI_AUTH_WPA_WPA2_PSK;
    char safe_pass[64];
    safe_strcpy(safe_pass, AP_PASS_CONFIG, sizeof(safe_pass));

    if (strlen(safe_pass) < 8) {
        ESP_LOGE(TAG, "Password corta. Usando OPEN.");
        auth_mode = WIFI_AUTH_OPEN;
        memset(safe_pass, 0, sizeof(safe_pass));
    }

    wifi_config_t ap_config = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = strlen(AP_SSID),
            .max_connection = 4,
            .authmode = auth_mode,
            .channel = 1
        },
    };
    if (auth_mode != WIFI_AUTH_OPEN) {
        safe_strcpy((char*)ap_config.ap.password, safe_pass, sizeof(ap_config.ap.password));
    }

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    
    wifi_config_t sta_config = { .sta = { .scan_method = WIFI_FAST_SCAN } };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));

    ESP_ERROR_CHECK(esp_wifi_start());

    start_webserver();
    xTaskCreate(dns_server_task, "dns_server", 4096, NULL, 5, &s_dns_task_handle);
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
                ESP_LOGE(TAG, "Fallo conexion. AP.");
                start_softap_provisioning();
            }
        }
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Conectado! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        stop_softap_provisioning();
        esp_wifi_set_mode(WIFI_MODE_STA); // Apagar AP zombie
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
        ESP_LOGI(TAG, "Conectando a: %s", ssid);
        wifi_config_t wifi_config = {0};
        
        safe_strcpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
        safe_strcpy((char*)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password));
        
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());
    } else {
        start_softap_provisioning();
    }
}
