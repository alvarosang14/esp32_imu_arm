#include <stdio.h>

#include "adc_wrapper.h"
#include "bno055_wrapper/bno055_wrapper.h"
#include "bt_wrapper.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PRESSURE_COUNT   4
#define ADC_FREQ_HZ      20000  // ESP32 minimum: 5 kHz per channel
#define SAMPLE_PERIOD_MS 20     // 50 Hz over Bluetooth

// ========================== ADCS AND GLOBAL VARIABLES ==========================

static const char *TAG = "main";

static const adc_channel_t pressure_ch[PRESSURE_COUNT] = {
    ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7};

static volatile uint16_t pressure[PRESSURE_COUNT];

// ========================== TASKS ==========================

static void adc_task(void *arg) {
    static adc_continuous_data_t data[ADC_MAX_SAMPLES];
    uint32_t n;

    while (1) {
        if (adc_read(data, &n, 100) != ESP_OK) {
            continue;
        }

        for (int i = 0; i < n; i++) {
            for (int ch = 0; ch < PRESSURE_COUNT; ch++) {
                if (data[i].valid && data[i].channel == pressure_ch[ch]) {
                    pressure[ch] = data[i].raw_data;
                }
            }
        }
    }
}

static void imu_task(void *arg) {
    struct bno055_euler_t euler[BNO055_COUNT];
    char msg[128];
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        s32 comres = BNO055_SUCCESS;
        for (int id = 0; id < BNO055_COUNT; id++)
            comres += bno055_read_euler(id, &euler[id]);

        /* Only fresh data is sent: a failed read skips this sample */
        if (comres == BNO055_SUCCESS) {
            snprintf(msg, sizeof(msg), "%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u\n",
                     euler[0].h, euler[0].r, euler[0].p,
                     euler[1].h, euler[1].r, euler[1].p,
                     euler[2].h, euler[2].r, euler[2].p,
                     pressure[0], pressure[1], pressure[2], pressure[3]);
            bt_send(msg);
        } else {
            ESP_LOGW(TAG, "IMU read failed, sample skipped");
        }

        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

// ========================== MAIN ==========================

void app_main(void) {
    // BNO055
    if (bno055_init_a() != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "A BNO055 is not responding (check SDA/SCL of both buses and the ADR pins)");
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
    struct_init();

    // ADC
    esp_err_t err_adc = adc_setup(pressure_ch, PRESSURE_COUNT, ADC_FREQ_HZ);
    if (err_adc == ESP_FAIL) {
        ESP_LOGE(TAG, "ADC ERROR");
        return;
    }
    
    // BT
    bt_setup();

    // TASKS
    xTaskCreate(adc_task, "adc", 4096, NULL, 5, NULL);
    xTaskCreate(imu_task, "imu", 4096, NULL, 5, NULL);
}
