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
#define SERIAL_PERIOD_MS 1000   // 1 Hz over serial
#define REF_DELAY_MS     3000   // arm still: let the fusion settle before the reference
#define IMU_MAX_FAILS    10     // consecutive failed reads (200 ms) before re-init

// ========================== ADCS AND GLOBAL VARIABLES ==========================

static const char *TAG = "main";

// ADC
static const adc_channel_t pressure_ch[PRESSURE_COUNT] = {
    CONFIG_BOARD_PRESSURE_CH0, CONFIG_BOARD_PRESSURE_CH1,
    CONFIG_BOARD_PRESSURE_CH2, CONFIG_BOARD_PRESSURE_CH3};

// I2C: [bus] -> pins and address of each IMU on it
static const bno055_bus_t i2c_bus[BNO055_I2C_BUS_COUNT] = {
    {CONFIG_BNO055_I2C0_SDA, CONFIG_BNO055_I2C0_SCL,
     {CONFIG_BNO055_ADDR1, CONFIG_BNO055_ADDR2}},
    {CONFIG_BNO055_I2C1_SDA, CONFIG_BNO055_I2C1_SCL,
     {CONFIG_BNO055_ADDR1, CONFIG_BNO055_ADDR2}},
};

// Datas
static volatile uint16_t pressure[PRESSURE_COUNT];
static QueueHandle_t imu_queue;

// One reading of all the IMUs
typedef struct {
    struct bno055_quaternion_t quat[BNO055_COUNT];
} imu_sample_t;

// ========================== HELPERS ==========================

/* IMU_x_y: quaternion w,x,y,z relative to the boot position (1 = 16384) */
static void convert_to_string(char* msg, size_t len, const imu_sample_t* imu) {
    const struct bno055_quaternion_t* q = imu->quat;
    snprintf(msg, len,
             "INFO:\n"
             "\tIMU_0_0: %d,%d,%d,%d\n"
             "\tIMU_0_1: %d,%d,%d,%d\n"
             "\tIMU_1_0: %d,%d,%d,%d\n"
             "\tIMU_1_1: %d,%d,%d,%d\n"
             "\tADC: %u,%u,%u,%u\n",
             q[0].w, q[0].x, q[0].y, q[0].z, q[1].w, q[1].x, q[1].y, q[1].z,
             q[2].w, q[2].x, q[2].y, q[2].z, q[3].w, q[3].x, q[3].y, q[3].z,
             pressure[0], pressure[1], pressure[2], pressure[3]);
}

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
    imu_sample_t imu;
    TickType_t last_wake = xTaskGetTickCount();
    bool failing = false;  // log only when the state changes, not every sample
    int fails[BNO055_COUNT] = {0};

    while (1) {
        s32 comres = BNO055_SUCCESS;
        for (int id = 0; id < BNO055_COUNT; id++) {
            if (bno055_read_quat(id, &imu.quat[id]) == BNO055_SUCCESS) {
                fails[id] = 0;
                bno055_save_calib_if_ready(id);
                continue;
            }

            /* Stuck bus or sensor reset by itself: set it up again. NDOF is
             * absolute, so the boot reference is still valid afterwards */
            comres = BNO055_ERROR;
            if (++fails[id] >= IMU_MAX_FAILS) {
                bno055_recover(id);
                fails[id] = 0;
            }
        }

        /* Only fresh data is published: a failed read keeps the previous sample */
        if (comres == BNO055_SUCCESS) {
            xQueueOverwrite(imu_queue, &imu);
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
    imu_sample_t imu;
    char msg[320];
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        /* Peek does not empty the mailbox: nothing is sent until the first IMU sample */
        if (xQueuePeek(imu_queue, &imu, 0) == pdTRUE) {
            convert_to_string(msg, sizeof(msg), &imu);
            bt_send(msg);
        }

        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SEND_PERIOD_MS));
    }
}

static void serial_task(void* arg) {
    imu_sample_t imu;
    char msg[320];
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        if (xQueuePeek(imu_queue, &imu, 0) == pdTRUE) {
            convert_to_string(msg, sizeof(msg), &imu);
            printf("%s", msg);
        }

        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SERIAL_PERIOD_MS));
    }
}

// ========================== MAIN ==========================

void app_main(void) {
    // BNO055
    if (bno055_init_a(i2c_bus) != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "A BNO055 is not responding (check SDA/SCL of both buses and the ADR pins)");
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(REF_DELAY_MS));
    if (struct_init() != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Could not take the BNO055 reference");
        return;
    }

    // ADC
    esp_err_t err_adc = adc_setup(pressure_ch, PRESSURE_COUNT, ADC_FREQ_HZ);
    if (err_adc != ESP_OK) {
        ESP_LOGE(TAG, "ADC ERROR");
        return;
    }
    
    // BT
    bt_setup();
    esp_log_level_set("BT_HCI", ESP_LOG_ERROR);   // sniff/unsniff are not errors
    esp_log_level_set("BT_APPL", ESP_LOG_ERROR);

    // TASKS
    imu_queue = xQueueCreate(1, sizeof(imu_sample_t));
    xTaskCreate(adc_task, "adc", 4096, NULL, 5, NULL);
    xTaskCreate(imu_task, "imu", 4096, NULL, 5, NULL);
    xTaskCreate(bt_task, "bt", 4096, NULL, 5, NULL);
    xTaskCreate(serial_task, "serial", 4096, NULL, 5, NULL);
}
