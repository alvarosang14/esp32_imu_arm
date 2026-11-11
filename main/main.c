#include <stdbool.h>
#include <stdio.h>

#include "adc_wrapper.h"
#include "bno055_wrapper/bno055_wrapper.h"
#include "bt_wrapper.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define PRESSURE_COUNT   4
#define ADC_FREQ_HZ      20000  // ESP32 minimum: 5 kHz per channel
#define SAMPLE_PERIOD_MS 20     // 50 Hz IMU reading
#define SEND_PERIOD_MS   20     // 50 Hz over Bluetooth

// ========================== ADCS AND GLOBAL VARIABLES ==========================

static const char *TAG = "main";
static const adc_channel_t pressure_ch[PRESSURE_COUNT] = {
    CONFIG_BOARD_PRESSURE_CH0, CONFIG_BOARD_PRESSURE_CH1,
    CONFIG_BOARD_PRESSURE_CH2, CONFIG_BOARD_PRESSURE_CH3};
static volatile uint16_t pressure[PRESSURE_COUNT];
static QueueHandle_t euler_queue;

// ========================== TASKS ==========================

static void adc_task(void *arg) {
    static adc_continuous_data_t data[ADC_MAX_SAMPLES];
    uint32_t n;

    while (1) {
        if (adc_read(data, &n, 100) != ESP_OK) {
            continue;
        }

        // Samples of all channels come mixed: store each one in its sensor slot
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
    TickType_t last_wake = xTaskGetTickCount();
    bool failing = false;  // log only when the state changes, not every sample

    while (1) {
        s32 comres = BNO055_SUCCESS;
        for (int id = 0; id < BNO055_COUNT; id++) {
            comres += bno055_read_euler(id, &euler[id]);
        }

        /* Only fresh data is published: a failed read keeps the previous sample */
        if (comres == BNO055_SUCCESS) {
            xQueueOverwrite(euler_queue, euler);
            if (failing) {
                ESP_LOGI(TAG, "IMU read recovered");
            }
            failing = false;
        } else {
            if (!failing) {
                ESP_LOGW(TAG, "IMU read failing, samples skipped");
            }
            failing = true;
        }

        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

// Task on top of a task bt_send only queues the message
static void bt_task(void *arg) {
    struct bno055_euler_t euler[BNO055_COUNT];
    char msg[128];
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        /* Peek does not empty the mailbox: nothing is sent until the first IMU sample */
        if (xQueuePeek(euler_queue, euler, 0) == pdTRUE) {
            snprintf(msg, sizeof(msg), "%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u\n",
                     euler[0].h, euler[0].r, euler[0].p,
                     euler[1].h, euler[1].r, euler[1].p,
                     euler[2].h, euler[2].r, euler[2].p,
                     pressure[0], pressure[1], pressure[2], pressure[3]);
            bt_send(msg);
        }

        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SEND_PERIOD_MS));
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
    if (err_adc != ESP_OK) {
        ESP_LOGE(TAG, "ADC ERROR");
        return;
    }
    
    // BT
    bt_setup();

    // TASKS
    euler_queue = xQueueCreate(1, sizeof(struct bno055_euler_t[BNO055_COUNT]));
    xTaskCreate(adc_task, "adc", 4096, NULL, 5, NULL);
    xTaskCreate(imu_task, "imu", 4096, NULL, 5, NULL);
    xTaskCreate(bt_task, "bt", 4096, NULL, 5, NULL);
}
