# 📋 Monitor Heladera IoT - Documentación Completa del Proyecto

## 📌 Descripción General

Sistema embebido profesional basado en **ESP-IDF** (ESP32) para monitoreo integral de una heladera industrial/comercial. Realiza mediciones en tiempo real de:

- **Temperatura** (interior heladera + electrónica del tablero)
- **Voltaje AC** (Red 220V)
- **Corriente AC** (Consumo de compresor)
- **Potencia Aparente** (VA)

Con compensación térmica automática y filtrado de ruido.

---

## 🏗️ Arquitectura del Proyecto

### Estructura de Directorios

```
monitor_heladera_iot/
├── CMakeLists.txt                 # Build configuration (ESP-IDF)
├── sdkconfig                       # Configuración del SDK guardada
├── sdkconfig.ci                    # Configuración CI (Continuous Integration)
├── build/                          # Artefactos compilados (generado)
├── components/                     # Componentes reutilizables
│   ├── ac_meter/                   # Driver medición de voltaje AC
│   │   ├── CMakeLists.txt
│   │   ├── ac_meter.c
│   │   └── include/
│   │       └── ac_meter.h
│   ├── ds18b20/                    # Driver sensor temperatura digital
│   │   ├── CMakeLists.txt
│   │   ├── ds18b20.c
│   │   └── include/
│   │       └── ds18b20.h
│   └── zmct103c/                   # Driver medición de corriente AC
│       ├── CMakeLists.txt
│       ├── zmct103c.c
│       └── include/
│           └── zmct103c.h
└── main/                           # Aplicación principal
    ├── CMakeLists.txt
    ├── Kconfig                     # Configuración de hardware (pines, valores)
    └── main.c                      # Lógica principal con 3 tareas FreeRTOS
```

---

## ⚙️ Stack Tecnológico

| Componente | Versión | Propósito |
|-----------|---------|----------|
| **ESP-IDF** | v5.x | Framework oficial ESP32 |
| **FreeRTOS** | Incluido en ESP-IDF | RTOS multitarea |
| **ADC API** | v5.x (adc_oneshot) | Lectura de canales analógicos |
| **GPIO Driver** | Estándar | Control de pines digitales |
| **1-Wire** | Software (bit banging) | Comunicación DS18B20 |

---

## 🔌 Mapeo de Hardware (Pines y Sensores)

### Sensores de Temperatura (GPIO)

| Sensor | Pin Recomendado | Protocolo | Función |
|--------|-----------------|-----------|---------|
| DS18B20 (Interior) | GPIO 32 | 1-Wire | Temp. dentro heladera |
| DS18B20 (Tablero) | GPIO 33 | 1-Wire | Temp. electrónica (compensación) |

**Nota:** Los GPIOs 34-39 son **solo entrada**, no usar para DS18B20.

### Sensores Eléctricos (ADC - Canal 1)

| Sensor | GPIO | Rango Input | Función | Atenuación |
|--------|------|------------|---------|-----------|
| ZMPT101B | GPIO 35 | 0-3.3V DC | Voltaje AC (220V reducido) | 11 dB |
| ZMCT103C | GPIO 34 | 0-3.3V DC | Corriente AC | 11 dB |

**Configuración ADC:**
- **Unidad:** ADC1 (para GPIO 32-39)
- **Bitwidth:** Predeterminado (12 bits en ESP32)
- **Atenuación:** ADC_ATTEN_DB_11 (máximo, para rango 0-3.3V)
- **Frecuencia Muestreo:** 2000 Hz (40 muestras por ciclo @50Hz)

### Conexiones de Referencia

#### ZMPT101B (Voltaje AC)
```
Red 220V AC
    ↓
[Divisor Resistivo 220V→3.3V]
    ↓
[Capacitor 100nF (0.1µF) filtro]
    ↓
GPIO 35 (ADC1_CH7)
```

**Fórmula divisor:** Si tienes R1=1M2Ω y R2=100kΩ:
- V_medido = 220V × (100k / (1M2 + 100k)) ≈ 18.3V pico
- Necesitas atenuación adicional o ajustar resistencias

#### ZMCT103C (Corriente AC)
```
Sensor ZMCT103C (transformador de corriente)
    ↓
[Resistencia de Carga 68Ω]
    ↓
GPIO 34 (ADC1_CH6)
```

---

## 📊 Componentes de Software

### 1. **ac_meter.c / ac_meter.h** - Medidor de Voltaje

**Propósito:** Medir voltaje RMS de la red AC.

**API Pública:**

```c
typedef struct {
    adc_channel_t channel;      // Canal ADC (ej: ADC_CHANNEL_7)
    adc_atten_t atten;          // Atenuación (ADC_ATTEN_DB_11)
    adc_bitwidth_t bitwidth;    // Resolución (ADC_BITWIDTH_DEFAULT)
    int fs_hz;                  // Freq. muestreo (ej: 2000)
    int window_ms;              // Ventana promedios (ej: 200ms)
} ac_meter_cfg_t;

typedef struct {
    float vline_rms;            // Voltaje RMS final (V)
    float raw_rms;              // RMS crudo (sin calibración)
} ac_meter_reading_t;

// Inicializar (recibe handle ADC compartido)
esp_err_t ac_meter_init(adc_oneshot_unit_handle_t adc_handle, 
                        const ac_meter_cfg_t *cfg);

// Leer última medición
esp_err_t ac_meter_read(ac_meter_reading_t *out);
```

**Internals:**
- Toma N muestras por ventana temporal
- Calcula RMS = √(promedio de cuadrados)
- Aplica factor de escala para convertir a voltios reales

---

### 2. **ds18b20.c / ds18b20.h** - Sensor Temperatura Digital

**Propósito:** Leer temperatura mediante protocolo 1-Wire (OneWire).

**API Pública:**

```c
typedef struct {
    gpio_num_t pin;             // GPIO del sensor
} ds18b20_t;

#define SENSOR_TEMP_ERROR -999.0f  // Código error

// Inicializar pin (OpenDrain)
void ds18b20_init(ds18b20_t *sensor, gpio_num_t pin);

// Leer temperatura (BLOQUEANTE ~750ms)
float ds18b20_read_temp(ds18b20_t *sensor);
// Retorna: temperatura en °C o SENSOR_TEMP_ERROR
```

**Características:**
- Protocolo 1-Wire: 1 pin para comunicación (data + alimentación)
- Tiempo de conversión: ~750ms (sensor mide internamente)
- Rango: -55°C a +125°C
- Resolución: 0.0625°C
- **Bloqueante:** No llamar en interrupt, usar en tarea

---

### 3. **zmct103c.c / zmct103c.h** - Sensor Corriente

**Propósito:** Medir corriente RMS mediante transformador de corriente.

**API Pública:**

```c
typedef struct {
    adc_channel_t adc_channel;  // Canal ADC (ej: ADC_CHANNEL_6)
    adc_atten_t adc_atten;      // Atenuación
    float burden_ohms;          // R de carga (ej: 68Ω)
    float ct_ratio;             // Relación trafo (ej: 1000:1)
    int sample_rate_hz;         // Freq. muestreo
    int cycles;                 // Ciclos promediar (ej: 10)
    int multisample;            // Oversampling SW (ej: 4)
} zmct103c_cfg_t;

typedef struct {
    adc_oneshot_unit_handle_t handle;  // Referencia ADC
    zmct103c_cfg_t cfg;                // Config guardada
} zmct103c_t;

// Inicializar
esp_err_t zmct103c_init(zmct103c_t *ctx, 
                        adc_oneshot_unit_handle_t handle,
                        const zmct103c_cfg_t *cfg);

// Leer corriente RMS (A)
esp_err_t zmct103c_read_irms(zmct103c_t *ctx, float *irms);
```

**Característica especial:**
- **Multisample:** Promedia múltiples muestras para reducir ruido
- **Detector ceros:** Sincroniza con ciclos AC para mejor precisión

---

## 🔧 Configuración (Kconfig - main/Kconfig)

Permite ajustar parámetros sin recompilar código C:

```kconfig
menu "Configuracion de Hardware"

    config GPIO_SENSOR_HELADERA
        int "GPIO Sensor Heladera (DS18B20)"
        default 32
        help
            Pin para sensor temperatura interior

    config GPIO_SENSOR_TABLERO
        int "GPIO Sensor Tablero (DS18B20)"
        default 33
        help
            Pin para sensor temperatura compensación

    config GPIO_AC_VOLTAGE
        int "GPIO Medicion Voltaje (ZMPT101B)"
        default 35
        help
            ADC1_CHANNEL_7. Requiere divisor de tensión.
            Conectar capacitor 100nF en paralelo.

    config GPIO_AC_CURRENT
        int "GPIO Medicion Corriente (ZMCT103C)"
        default 34
        help
            ADC1_CHANNEL_6. Transformador de corriente.

    config TEMP_COEFF_PPM
        int "Coeficiente de Correccion (ppm/C)"
        default 100
        help
            Ajuste por deriva térmica (típico 50-100 ppm/°C)

endmenu
```

**Cómo usar:**
```bash
idf.py menuconfig
# Navegar a "Configuracion de Hardware"
# Cambiar valores según tu hardware
# Guardar (salir con Q)
```

Los valores se guardan en `sdkconfig` y se reflejan en `build/config/sdkconfig.h` (ya incluido en main.c).

---

## 🎯 Lógica Principal (main.c)

### Estructura de Datos Compartida

```c
typedef struct {
    float temp_heladera;         // °C
    float temp_tablero;          // °C (ref. térmica)
    float voltaje_rms;           // V
    float corriente_rms;         // A (con filtro EMA)
    float potencia_aparente;     // VA (V × I)
} sistema_estado_t;

static sistema_estado_t g_estado;     // Variable global
static SemaphoreHandle_t g_mutex;     // Mutex para sincronización
```

**Por qué mutex:** Múltiples tareas acceden `g_estado` simultáneamente. Sin protección → race conditions → valores corruptos.

---

### Tarea 1: vTaskTermica - Lectura de Temperatura

```c
void vTaskTermica(void *pvParameters)
```

**Responsabilidades:**
1. Inicializar 2 sensores DS18B20 (pines configurables)
2. Leer temperatura cada 2 segundos
3. Guardar en `g_estado.temp_heladera` y `g_estado.temp_tablero`
4. Descartar valores de error (-999) para no ensuciar datos

**Pseudocódigo:**
```
Iniciar DS18B20_1 en GPIO 32
Iniciar DS18B20_2 en GPIO 33

BUCLE infinito:
    t1 = Leer DS18B20_1 (tarda ~750ms)
    t2 = Leer DS18B20_2 (tarda ~750ms)
    
    SI t1 válido: g_estado.temp_heladera = t1
    SI t2 válido: g_estado.temp_tablero = t2
    
    Esperar 2 segundos
```

**Prioridad:** 5 (baja)
**Núcleo:** 0
**Stack:** 4096 bytes

---

### Tarea 2: vTaskEnergia - Medición Eléctrica + Compensación Térmica

```c
void vTaskEnergia(void *pvParameters)
```

**Responsabilidades:**
1. Crear e inicializar **ADC Unit 1** (compartida para ambos sensores)
2. Configurar ac_meter (voltaje ZMPT101B)
3. Configurar zmct103c (corriente ZMCT103C)
4. **Aplicar compensación térmica** según temperatura tablero
5. Filtrar ruido (dead zone)
6. Actualizar `g_estado` cada 500ms

**Compensación Térmica Lineal:**
```
ΔT = T_actual - T_ref (25°C)
Factor_corrección = 1 + (ppm/1e6) × ΔT

V_final = V_medido × Factor_corrección
I_final = I_medido × Factor_corrección_I
```

**Ejemplo:**
- T_ref = 25°C
- ppm = 100 ppm/°C (resistencia metal film)
- T_actual = 35°C
- ΔT = +10°C
- Factor = 1 + (100/1e6) × 10 = 1.001 (+0.1%)
- Si V_medido = 220V → V_final = 220.22V

**Dead Zone (Filtro de Ruido):**
```
SI V < 9V      → V = 0V      (ruido de red sin carga)
SI I < 0.05A   → I = 0A      (ruido sensor)
```

**Filtro EMA para Corriente:**
```
EMA_new = α × I_medido + (1-α) × EMA_old
α = 0.20 (suaviza fluctuaciones rápidas)
```

**Pseudocódigo:**
```
Crear ADC Unit 1 (handle compartido)
Configurar ac_meter en GPIO 35
Configurar zmct103c en GPIO 34

BUCLE infinito:
    Leer voltaje → ac_meter_read()
    Leer corriente → zmct103c_read_irms()
    
    SI lecturas OK:
        Obtener T_tablero (con mutex)
        ΔT = T_actual - 25
        V_corregido = V × (1 + ppm×ΔT/1e6)
        I_corregido = I × factor_calib × filtro_EMA
        
        Aplicar dead zone
        
        Guardar en g_estado (con mutex):
            voltaje_rms = V_corregido
            corriente_rms = I_corregido
            potencia_aparente = V × I
    
    Esperar 500ms
```

**Prioridad:** 10 (ALTA - sensible a timing)
**Núcleo:** 1
**Stack:** 4096 bytes

**Notas:**
- Se ejecuta en núcleo 1 (el núcleo 0 puede estar ocupado con WiFi/BLE)
- Requiere más stack por operaciones float y math
- Bloqueante 750ms durante lectura DS18B20 (el API interno maneja delays)

---

### Tarea 3: vTaskReporte - Monitor Serial

```c
void vTaskReporte(void *pvParameters)
```

**Responsabilidades:**
1. Imprimir estado cada 5 segundos
2. Formato legible tipo tablero industrial
3. No afecta a mediciones

**Salida Típica:**
```
W (12345) HELADERA_IOT: ========================================
I (12345) HELADERA_IOT: ❄️  HELADERA:   4.5 °C  | 🌡️  TABLERO:  28.3 °C
I (12345) HELADERA_IOT: ⚡  TENSION:  220.1 V   | 🔌  CORRIENTE:  2.34 A
I (12345) HELADERA_IOT: 💡  POTENCIA:  515.4 VA
W (12345) HELADERA_IOT: ========================================
```

**Prioridad:** 1 (muy baja)
**Núcleo:** 0
**Stack:** 3072 bytes

---

### Función app_main() - Punto de Entrada

```c
void app_main(void)
```

**Secuencia de inicialización:**

1. **NVS Flash:** Inicializar memoria no volátil
   - Requerida por WiFi/Bluetooth (aunque no usamos aún)
   - Si está corrupta, borrar y reiniciar

2. **Logging:** Configurar niveles de log
   ```c
   esp_log_level_set("*", ESP_LOG_INFO);        // Default
   esp_log_level_set("AC_METER", ESP_LOG_WARN); // Solo errors graves
   esp_log_level_set("ZMCT103C", ESP_LOG_WARN);
   esp_log_level_set("DS18B20", ESP_LOG_WARN);
   ```

3. **Mutex:** Crear semáforo binario para sincronización

4. **Crear Tareas FreeRTOS:**
   ```c
   xTaskCreatePinnedToCore(
       vTaskTermica,          // Función tarea
       "Task_Clima",          // Nombre (debug)
       4096,                  // Stack size (bytes)
       NULL,                  // Parámetro (no usado)
       5,                     // Prioridad (0-24, mayor=+importante)
       NULL,                  // Handle (no guardamos)
       0                      // Núcleo (0 ó 1)
   );
   ```

**Orden de creación:**
- Termometría (Núcleo 0, Prioridad 5)
- Metrología (Núcleo 1, Prioridad 10) ← Más crítica
- Reporte (Núcleo 0, Prioridad 1)

---

## 📁 Sistema de Compilación (CMakeLists.txt)

### Root CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.5)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # Para IDE (VS Code IntelliSense)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(monitor_heladera_iot)
```

**Propósito:**
- Mínimo requerido para ESP-IDF
- `project()` llama a cmake del SDK que:
  - Busca componentes en `components/`
  - Busca app en `main/`
  - Compila todo

### main/CMakeLists.txt
```cmake
idf_component_register(
    SRCS "main.c"                      # Archivos fuente
    INCLUDE_DIRS "."                   # Dirs para #include
    REQUIRES ac_meter zmct103c ds18b20 # Dependencias (componentes)
             nvs_flash driver esp_timer
)
```

**SRCS:** Archivos `.c` a compilar
**REQUIRES:** Componentes de los que depende
- `ac_meter` - componente propio
- `zmct103c` - componente propio
- `ds18b20` - componente propio
- `nvs_flash` - del SDK (NVS)
- `driver` - del SDK (GPIO, ADC, etc)
- `esp_timer` - del SDK (timers)

### components/*/CMakeLists.txt

Estructura idéntica para cada componente:

```cmake
idf_component_register(
    SRCS "ac_meter.c"        # ó zmct103c.c / ds18b20.c
    INCLUDE_DIRS "include"
    REQUIRES esp_adc driver  # Dependencias del SDK
)
```

---

## 🚀 Compilación y Flasheo

### Comandos Básicos

```bash
# Configurar placa (ESP32 clásico)
idf.py set-target esp32

# Menuconfig (ajustar pines/valores)
idf.py menuconfig

# Compilar
idf.py build

# Flashear código (requiere placa conectada)
idf.py flash

# Monitor puerto serial (log + salida)
idf.py monitor

# Todo en uno: compilar + flashear + monitorear
idf.py build flash monitor

# Limpiar builds anteriores
idf.py fullclean
```

### Archivos Generados

```
build/
├── compile_commands.json    # Para IDE (inteligencia de código)
├── config/
│   └── sdkconfig.h          # Header con CONFIG_* defines
├── flash_args               # Parámetros para flasher
├── monitor_heladera_iot.elf # Ejecutable final
└── esp-idf/                 # Código compilado del SDK
```

---

## 📐 Constantes Clave de Calibración

```c
// main.c línea ~50-56

#define TEMP_COEFF_PPM      CONFIG_TEMP_COEFF_PPM // Desde Kconfig
#define TEMP_COEFF_PPM_I    120                   // ppm/°C corriente
#define CURRENT_CAL_FACTOR  0.41f                 // Factor escala
#define CURRENT_EMA_ALPHA   0.20f                 // Suavizado corriente
```

**Ajustes para tuning:**

| Parámetro | Rango Típico | Efecto |
|-----------|-------------|--------|
| `TEMP_COEFF_PPM` | 50-150 | Corrección voltaje por T° |
| `TEMP_COEFF_PPM_I` | 50-150 | Corrección corriente por T° |
| `CURRENT_CAL_FACTOR` | 0.3-0.5 | Escala corriente RMS |
| `CURRENT_EMA_ALPHA` | 0.05-0.5 | Mayor=Rápida, Menor=Suave |
| Dead Zone V | 9V | Ignora mediciones < 9V |
| Dead Zone I | 0.05A | Ignora mediciones < 50mA |

---

## 🔍 Sincronización y Thread-Safety

### Problema: Race Conditions

Sin protección, esto puede ocurrir:

```
Task 1: Lee g_estado.voltaje_rms           ← 220V (parcial)
Task 2: Escribe g_estado.voltaje_rms ← 0V (interrumpe)
Task 3: Lee g_estado.voltaje_rms           ← 0V (inconsistente!)
```

### Solución: Mutex

```c
// Tomar mutex (esperar si está tomado)
if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    // Zona crítica - acceso exclusivo a g_estado
    g_estado.voltaje_rms = v_final;
    g_estado.corriente_rms = i_final;
    // Liberar para otras tareas
    xSemaphoreGive(g_mutex);
}
```

**Timeout:** 100ms
- Si otra tarea ocupa mutex >100ms → abandonamos lectura
- Evita deadlocks (aunque raro aquí)

---

## 📊 Diagrama de Flujo Multitarea

```
                    app_main()
                        ↓
          (Crear NVS, Mutex, Logging)
                        ↓
        ┌───────────────┼───────────────┐
        ↓               ↓               ↓
   Task_Clima    Task_Metrologia  Task_Reporte
   (Núcleo 0)     (Núcleo 1)       (Núcleo 0)
   Pri 5          Pri 10           Pri 1
        ↓               ↓               ↓
   Leo DS18B20   Leo ADC + Calib   Leo g_estado
   cada 2s       cada 500ms         cada 5s
        ↓               ↓               ↓
   Actualizo    Aplico Compensación  Imprimo
   g_estado      Térmica + Filtro    Serial
   (con mutex)   Guardo g_estado
                 (con mutex)
                        ↓
            FreeRTOS Scheduler
       (Alterna ejecución por tiempo)
```

---

## ⚠️ Consideraciones Importantes

### 1. **Bloqueos Serios (Timeout)**

**DS18B20 es bloqueante ~750ms:**
```c
float t1 = ds18b20_read_temp(&s_heladera);  // ← ESPERA 750ms aquí
```

Si Task_Termica tiene prioridad alta, puede "starvar" (no dejar ejecutar) otras. 
**Solución actual:** Prioridad 5 (media-baja), así Task_Metrologia (Pri 10) siempre gana.

### 2. **ADC Compartida**

Ambos sensores (voltaje + corriente) usan **ADC Unit 1**:
```c
adc_oneshot_new_unit(&init_config, &adc_handle);
ac_meter_init(adc_handle, ...);        // Comparte handle
zmct103c_init(&zmct, adc_handle, ...);  // Comparte handle
```

ESP-IDF maneja mutex interno para ADC → no hay problemas.

### 3. **Protección de Lecturas Incompletas**

Riesgo: Leer g_estado a mitad de actualización
```c
if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    // Copia ATÓMICA de g_estado
    float v = g_estado.voltaje_rms;
    float i = g_estado.corriente_rms;
    xSemaphoreGive(g_mutex);
    // Ahora puedo usar v, i sin problemas
}
```

### 4. **Calibración Requiere Equipos**

Para calibración precisa:
- **Voltaje:** Multímetro AC profesional
- **Corriente:** Pinza amperimétrica AC
- **Temperatura:** Termómetro de referencia

Actualmente:
- Voltaje: Asume factor divisor resistivo correcto
- Corriente: `CURRENT_CAL_FACTOR` ajustable empíricamente
- Temperatura: DS18B20 de fábrica (±0.5°C típico)

---

## 📝 Cómo Extender el Proyecto

### Agregar Nueva Medición (Ej: Humedad)

1. **Crear componente:**
   ```bash
   mkdir components/dht11
   # Copiar structure de ds18b20
   ```

2. **Editar main.c:**
   ```c
   #include "dht11.h"
   
   typedef struct {
       // ... campos existentes ...
       float humedad;  // Nuevo campo
   } sistema_estado_t;
   ```

3. **Crear tarea:**
   ```c
   void vTaskHumedad(void *pvParameters) {
       dht11_t sensor;
       dht11_init(&sensor, CONFIG_GPIO_DHT11);
       while (1) {
           float hum = dht11_read_humidity(&sensor);
           if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
               if (hum != ERROR) g_estado.humedad = hum;
               xSemaphoreGive(g_mutex);
           }
           vTaskDelay(pdMS_TO_TICKS(2000));
       }
   }
   ```

4. **Lanzar en app_main:**
   ```c
   xTaskCreatePinnedToCore(vTaskHumedad, "Task_Humedad", 4096, NULL, 5, NULL, 0);
   ```

5. **Actualizar Kconfig:**
   ```kconfig
   config GPIO_DHT11
       int "GPIO DHT11"
       default 25
   ```

6. **Actualizar CMakeLists.txt main:**
   ```cmake
   idf_component_register(
       SRCS "main.c"
       INCLUDE_DIRS "."
       REQUIRES ac_meter zmct103c ds18b20 dht11  # ← Agregar
   )
   ```

### Enviar Datos a WiFi/Cloud

1. **Incluir componente WiFi:**
   ```c
   #include "esp_wifi.h"
   #include "esp_http_client.h"
   ```

2. **Nueva tarea para comunicación:**
   ```c
   void vTaskWiFi(void *pvParameters) {
       // Conectar WiFi
       // Enviar JSON con g_estado cada 60s a servidor
   }
   ```

---

## 🐛 Debugging

### Ver Logs en Vivo
```bash
idf.py monitor
# Ctrl+] para salir
```

### Filtrar por Tag
```bash
idf.py monitor | grep "HELADERA_IOT"
idf.py monitor --print_filter="HELADERA_IOT:V"
```

### Aumentar Verbosidad
```bash
idf.py menuconfig
# → Component config → Log output
#   Cambiar a DEBUG
```

### Breakpoints (JTAG)
Si tienes adaptador JTAG:
```bash
idf.py openocd
# En otra terminal:
idf.py gdb
```

---

## 📚 Referencias

- [ESP-IDF Official Docs](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [ADC Driver (v5.x)](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html)
- [GPIO Driver](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html)
- [FreeRTOS](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html)
- [DS18B20 Datasheet](https://www.analog.com/en/products/ds18b20.html)
- [ZMPT101B Datasheet](https://components101.com/modules/zmpt101b-ac-voltage-sensor-module)
- [ZMCT103C Datasheet](https://www.zmct103c.com/)

---

## 📄 Resumen de Archivos

| Archivo | Líneas | Propósito |
|---------|--------|----------|
| `main/main.c` | 281 | Lógica principal + 3 tareas |
| `main/CMakeLists.txt` | 5 | Config compilación app |
| `main/Kconfig` | ~30 | Parámetros ajustables |
| `components/ac_meter/ac_meter.c` | ~150 | Voltímetro RMS |
| `components/ac_meter/ac_meter.h` | ~25 | API voltímetro |
| `components/zmct103c/zmct103c.c` | ~150 | Amperímetro RMS |
| `components/zmct103c/zmct103c.h` | ~25 | API amperímetro |
| `components/ds18b20/ds18b20.c` | ~100 | Termómetro 1-Wire |
| `components/ds18b20/ds18b20.h` | ~20 | API termómetro |
| `CMakeLists.txt` | 5 | Config build root |

---

## ✅ Checklist para Recrear el Proyecto

- [ ] Clonar o crear estructura de directorios
- [ ] Copiar todos los `.c` y `.h` files
- [ ] Copiar `CMakeLists.txt` (root y subdirs)
- [ ] Copiar `main/Kconfig` con configuración de pines
- [ ] `idf.py set-target esp32`
- [ ] `idf.py menuconfig` → ajustar pines según hardware
- [ ] `idf.py build`
- [ ] Conectar ESP32 por USB
- [ ] `idf.py flash monitor`
- [ ] Verificar logs: mostrarán "HELADERA_IOT" con mediciones

---

**Documento generado:** 25 de diciembre de 2025
**Versión:** 1.0
**Autor:** Documentación automática

