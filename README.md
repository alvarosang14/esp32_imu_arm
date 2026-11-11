# Guide use ESP IDF

## 1. Install ESP

> Solo con la extension vale ya que este trae ya un docker con todo

Install esp32 manager

``` bash
echo "deb [trusted=yes] https://dl.espressif.com/dl/eim/apt/ stable main" | sudo tee /etc/apt/sources.list.d/espressif.list
sudo apt update

# cli + gui
#sudo apt install eim

# onli cli
sudo apt install eim-cli
```

Install dependencies project for esp32 using CLI.
+ Default directory: `/home/asantos/.espressif`
+ Install components ...

``` bash
# Automatic install
eim install

# Install with selection
#eim wizard

# Install a specidifc version
#eim install -i v5.4.2
```

> https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/linux-setup.html#offline-installation

> Esto genera eim_config.toml que es para repetir exactamente la misma instalacion sin romper nada
>   No va en el proyecto

## 2. Install extension ESP

``` bash
code --install-extension espressif.esp-idf-extension
```

OR

extensions.json
``` json
{
  "recommendations": [
    "espressif.esp-idf-extension"
  ]
}
```

> https://docs.espressif.com/projects/vscode-esp-idf-extension/en/latest/index.html

OR

Select profile clangd:
> https://docs.espressif.com/projects/vscode-esp-idf-extension/zh_CN/latest/additionalfeatures/esp-idf-profiles.html

+ The url is: https://raw.githubusercontent.com/espressif/vscode-esp-idf-extension/master/profiles/esp-idf-clangd.code-profile

## 3. Start project

+ Type ESP-IDF: New Project

Select sample_project and:
+ IDF target: Son las distintas familias de chips de Espressif. Cada una es un SoC diferente, con su CPU, periféricos y radios:

| Target | CPU | Radio | Para qué |
|---|---|---|---|
| **esp32** | 2× Xtensa LX6, 240 MHz | WiFi 4 + BT clásico + BLE | El original, el tuyo |
| **esp32s2** | 1× Xtensa LX7 | WiFi, **sin BT** | USB OTG nativo, bajo coste |

**`linux`** no es un chip. Compila tu aplicación para **ejecutarla en el PC** como un proceso normal. Sirve para tests unitarios y lógica sin hardware (los componentes de FreeRTOS, NVS, etc. tienen versión simulada), sin necesidad de flashear.

## 4. Estructura del proyecto

+ En componentes estan esas partes del codigo que se comunican con el hardware.
  + Pueden ser nuestros
  + Del propio esp, que creo que no hace falta ponerlos ahi ya que estan en el propio sdk
  + De terceros, se pone como dependencia en ...

+ Esta el sdk que nos permite compilar sin dependecias
  + En el caso de vscode, este mira en .devcontainer para leer un fichero de configuracion y un dockerfile.

## 5. Compile

El `.devconatiner` contiene un docker con las dependencias y asi no haria flata inatalar esp idf.

La extension instala los settings de vscode para ajustar los controles a ella.

+ En un componente va cualqueir cosa que se reutilice. Digamos que es un submodulo de esp
> Estructura, ficheros y demas para compilar: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/build-system.html

+ kconfig como los de kernel. Permite como `make menuconfig` y configurar parametros depends...
> kconfig: https://docs.espressif.com/projects/esp-idf-kconfig/en/latest/kconfiglib/language.html

## 6. Hardware

+ I2C: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2c.html

## 7. idf.py

``` bash
source ~/.espressif/tools/activate_idf_v6.1.sh
``` 

``` bash
idf.py build
```

> https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/tools/idf-py.html

## 8. Bluethoot

> https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/bluetooth/esp_bt_device.html
> https://docs.espressif.com/projects/esp-idf/en/v5.0/esp32/api-reference/bluetooth/esp_bt_main.html

+ Oficial example: https://github.com/espressif/esp-idf/tree/master/examples/bluetooth/bluedroid/classic_bt/bt_spp_acceptor

## 9. ADC

> ADC: https://docs.espressif.com/projects/esp-idf/en/release-v6.1/esp32/api-reference/peripherals/adc/index.html

### ADC continuo (`components/adc_wrapper`)

> https://docs.espressif.com/projects/esp-idf/en/release-v6.1/esp32/api-reference/peripherals/adc/adc_continuous.html

+ El ADC muestrea solo, por DMA (usa el I2S0), y guarda los resultados en un *pool* interno. Nosotros leemos frames de ese pool.
+ En ESP32 el modo continuo **solo funciona con ADC1** (GPIO32-39). El ADC2 lo usa el WiFi.
+ Frecuencia: entre **20 kHz y 2 MHz** (`SOC_ADC_SAMPLE_FREQ_THRES_LOW/HIGH`).
+ Cada muestra son 12 bits. Con `ADC_ATTEN_DB_12` el rango es ~0-3.1 V.

Flujo del driver:

| Paso | Funcion | Que hace |
|---|---|---|
| 1 | `adc_continuous_new_handle()` | Crea el driver: tamaño del pool (`max_store_buf_size`) y del frame (`conv_frame_size`) |
| 2 | `adc_continuous_config()` | Canales (patron), frecuencia y modo (`ADC_CONV_SINGLE_UNIT_1`) |
| 3 | `adc_continuous_start()` | Empieza a convertir |
| 4 | `adc_continuous_read()` + `adc_continuous_parse_data()` | Lee bytes crudos y los pasa a `{unit, channel, raw_data, valid}` |
| 5 | `adc_continuous_stop()` + `adc_continuous_deinit()` | Para y libera |

Uso del wrapper:

``` c
#include "adc_wrapper.h"

adc_channel_t ch[] = {ADC_CHANNEL_6, ADC_CHANNEL_7}; // GPIO34, GPIO35
adc_setup(ch, 2, 20000);

adc_continuous_data_t data[ADC_MAX_SAMPLES];
uint32_t n;
if (adc_read(data, &n, 100) == ESP_OK) {
    for (int i = 0; i < n; i++) {
        if (data[i].valid) {
            ESP_LOGI("adc", "ch%d = %lu", data[i].channel, data[i].raw_data);
        }
    }
}
```

Cuidado:
+ `adc_read()` devuelve `ESP_ERR_INVALID_STATE` si el pool se llenó: estamos leyendo más lento de lo que convierte. Lee más a menudo o baja la frecuencia.
+ Los valores son crudos. Para pasarlos a mV hay que calibrar (`adc_cali_raw_to_voltage()`).
+ El driver no es thread-safe: usarlo desde una sola tarea.

## 10. Programa (`main/main.c`)

+ 3 IMU BNO055: dos en el I2C0 (ADR a GND = `0x28`, ADR a 3V3 = `0x29`) y uno en el I2C1 (`0x28`).
+ 4 sensores de presion por ADC continuo (DMA): GPIO36, GPIO39, GPIO34, GPIO35 (A0-A3 de la FireBeetle).
+ Todo se manda por Bluetooth clasico (SPP), nombre `BT_IMU_ARM`.

Pines I2C (se cambian con `idf.py menuconfig` -> *esp32_imu_arm board*):

| Bus | SDA | SCL | IMU |
|---|---|---|---|
| I2C0 | 21 | 22 | 0 y 1 |
| I2C1 | 25 | 26 | 2 |

Canales ADC1 de los sensores de presion (mismo menu de `menuconfig`):

| Sensor | Canal | GPIO | Pin FireBeetle |
|---|---|---|---|
| 0 | 0 | 36 | A0 |
| 1 | 3 | 39 | A1 |
| 2 | 6 | 34 | A2 |
| 3 | 7 | 35 | A3 |

> En la FireBeetle el GPIO32 y el GPIO33 no salen a los pines, y A4 (GPIO15) es ADC2: no vale para el modo continuo.

Tareas FreeRTOS:

| Tarea | Que hace |
|---|---|
| `adc_task` | Lee los frames del DMA y guarda la ultima muestra de cada canal |
| `imu_task` | Cada 20 ms (`SAMPLE_PERIOD_MS`) lee el euler de los 3 IMU y publica la ultima lectura |
| `bt_task` | Cada 20 ms (`SEND_PERIOD_MS`) coge la ultima lectura y manda una linea por Bluetooth |

Linea que se manda (CSV, 50 Hz):

```
h0,r0,p0,h1,r1,p1,h2,r2,p2,adc0,adc1,adc2,adc3
```

+ `h,r,p`: heading, roll, pitch en **1/16 de grado** (dividir entre 16). Ya llevan restada la referencia que se toma al arrancar (`struct_init()`).
+ `adc`: valor crudo de 12 bits (0-4095).

Cuidado:
+ Al arrancar hay que dejar el brazo quieto ~1 s: ahi se toma la referencia.
+ Al restar el heading de dos IMU el resultado puede saltar ±360: hay que normalizarlo en el receptor.
