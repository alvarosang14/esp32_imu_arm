#include <stdio.h>

#include "bno055_wrapper/bno055_wrapper.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "main";

void app_main(void) {
    if (bno055_init_a() != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "BNO055 no responde (revisa SDA=%d SCL=%d y la ADR)",
                 CONFIG_BOARD_I2C_SDA, CONFIG_BOARD_I2C_SCL);
        return;
    }

    /* El BNO055 tarda en dar datos de fusion validos tras cambiar de modo */
    vTaskDelay(pdMS_TO_TICKS(1000));
    struct_init();

    struct bno055_accel_t accel;
    struct bno055_gyro_t gyro;
    struct bno055_euler_t euler;

    while (1) {
        bno055_read(&accel, &gyro);
        bno055_read_euler(&euler);

        ESP_LOGI(TAG,
                 "accel=[%6d %6d %6d] gyro=[%6d %6d %6d] euler=[%6d %6d %6d]",
                 accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z, euler.h,
                 euler.p, euler.r);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
