#include "bno055_wrapper/bno055_wrapper.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include "BNO055/bno055.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/i2c_types.h"
#include "nvs.h"
#include "nvs_flash.h"

#define BNO055_I2C_TIMEOUT_MS 100
#define BNO055_I2C_FREQ_HZ 100000  // BNO055 stretches the clock: 400 kHz timed out
#define BNO055_I2C_WR_BUF_LEN 32

#define BNO055_QUAT_ONE      16384.0f
#define BNO055_REF_TIMEOUT_MS 5000  // max wait for the gyros before the reference
#define BNO055_REF_RETRIES    50

/* CALIB_STAT register: sys<<6 | gyro<<4 | accel<<2 | mag, each 0 (none) .. 3 (full) */
#define BNO055_CALIB_GYRO(c) (((c) >> 4) & 0x03)
#define BNO055_CALIB_MAG(c)  ((c) & 0x03)

#define BNO055_NVS_NAMESPACE "bno055"

static const char* TAG = "bno055";

static i2c_master_bus_handle_t s_i2c_bus[BNO055_I2C_BUS_COUNT];
static i2c_master_dev_handle_t s_i2c_dev[BNO055_COUNT];

/*----------------------------------------------------------------------------*
 *  struct bno055_t parameters can be accessed by using BNO055
 *  BNO055_t having the following parameters
 *  Bus write function pointer: BNO055_WR_FUNC_PTR
 *  Bus read function pointer: BNO055_RD_FUNC_PTR
 *  Burst read function pointer: BNO055_BRD_FUNC_PTR
 *  Delay function pointer: delay_msec
 *  I2C address: dev_addr
 *  Chip id of the sensor: chip_id
 *
 *  The Bosch API only keeps one active sensor, so a single bno055_t is shared
 *  and dev_addr carries the id of the IMU (index of s_i2c_dev) to read.
 *---------------------------------------------------------------------------*/
static struct bno055_t bno055;

static struct bno055_quaternion_t quat_init[BNO055_COUNT];
static bool calib_stored[BNO055_COUNT];  // a profile exists in NVS (loaded or saved)

/* The Bosch API addresses by register: write = [reg_addr | payload] */
static s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 len) {
    if (s_i2c_dev[dev_addr] == NULL || len > (BNO055_I2C_WR_BUF_LEN - 1)) {
        return BNO055_ERROR;
    }

    u8 buf[BNO055_I2C_WR_BUF_LEN];
    buf[0] = reg_addr;
    for (u8 i = 0; i < len; i++) {
        buf[i + 1] = reg_data[i];
    }

    esp_err_t err = i2c_master_transmit(s_i2c_dev[dev_addr], buf, (size_t)len + 1,
                                        BNO055_I2C_TIMEOUT_MS);

    return (err == ESP_OK) ? BNO055_SUCCESS : BNO055_ERROR;
}

static s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 len) {
    if (s_i2c_dev[dev_addr] == NULL) {
        return BNO055_ERROR;
    }

    esp_err_t err = i2c_master_transmit_receive(s_i2c_dev[dev_addr], &reg_addr, 1,
                                               reg_data, (size_t)len,
                                               BNO055_I2C_TIMEOUT_MS);

    return (err == ESP_OK) ? BNO055_SUCCESS : BNO055_ERROR;
}

static void BNO055_delay_msec(u32 msec) {
    if (msec == 0) {
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(msec) + 1);
}

static esp_err_t BNO055_i2c_bus_init(const bno055_bus_t bus[BNO055_I2C_BUS_COUNT]) {
    if (s_i2c_bus[I2C_NUM_0] != NULL) {
        return ESP_OK;
    }

    for (int port = 0; port < BNO055_I2C_BUS_COUNT; port++) {
        i2c_master_bus_config_t bus_cfg = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = port,
            .sda_io_num = bus[port].sda,
            .scl_io_num = bus[port].scl,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };

        esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_i2c_bus[port]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2c_new_master_bus %d: %s", port, esp_err_to_name(err));
            return err;
        }

        /* id = port * BNO055_IMUS_PER_BUS + slot */
        for (int slot = 0; slot < BNO055_IMUS_PER_BUS; slot++) {
            int id = port * BNO055_IMUS_PER_BUS + slot;
            i2c_device_config_t dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = bus[port].addr[slot],
                .scl_speed_hz = BNO055_I2C_FREQ_HZ,
            };

            err = i2c_master_bus_add_device(s_i2c_bus[port], &dev_cfg, &s_i2c_dev[id]);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "i2c_master_bus_add_device %d: %s", id, esp_err_to_name(err));
                return err;
            }
        }
    }

    return ESP_OK;
}

// ========================== CALIBRATION (NVS) ==========================

/* Calibration profile of one IMU, saved in NVS so it survives a reboot */
typedef struct {
    struct bno055_accel_offset_t accel;
    struct bno055_mag_offset_t mag;
    struct bno055_gyro_offset_t gyro;
} bno055_calib_profile_t;

static void calib_key(u8 id, char *key, size_t len) {
    snprintf(key, len, "cal%u", id);
}

/* Writes the saved profile of `id` (if there is one) into the sensor.
 * The Bosch write functions switch to CONFIG mode and back by themselves */
static void calib_load(u8 id) {
    nvs_handle_t nvs;
    if (nvs_open(BNO055_NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        ESP_LOGI(TAG, "imu%d: no saved calibration", id);
        return;
    }

    bno055_calib_profile_t prof;
    size_t len = sizeof(prof);
    char key[8];
    calib_key(id, key, sizeof(key));
    esp_err_t err = nvs_get_blob(nvs, key, &prof, &len);
    nvs_close(nvs);

    if (err != ESP_OK || len != sizeof(prof)) {
        ESP_LOGI(TAG, "imu%d: no saved calibration", id);
        return;
    }

    bno055.dev_addr = id;
    s32 comres = BNO055_SUCCESS;
    comres += bno055_write_accel_offset(&prof.accel);
    comres += bno055_write_mag_offset(&prof.mag);
    comres += bno055_write_gyro_offset(&prof.gyro);
    calib_stored[id] = (comres == BNO055_SUCCESS);
    ESP_LOGI(TAG, "imu%d: saved calibration loaded (comres=%ld)", id, (long)comres);
}

static s32 calib_read_stat(u8 id, u8 *calib) {
    return BNO055_I2C_bus_read(id, BNO055_CALIB_STAT_ADDR, calib, 1);
}

static s32 calib_save(u8 id) {
    bno055_calib_profile_t prof;
    s32 comres = BNO055_SUCCESS;

    /* The offset registers are only valid when read in CONFIG mode.
     * Going back to NDOF restarts the fusion, but it is absolute (gravity +
     * calibrated mag), so it comes back to the same orientation */
    bno055.dev_addr = id;
    comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
    comres += bno055_read_accel_offset(&prof.accel);
    comres += bno055_read_mag_offset(&prof.mag);
    comres += bno055_read_gyro_offset(&prof.gyro);
    comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
    if (comres != BNO055_SUCCESS) {
        return BNO055_ERROR;
    }

    nvs_handle_t nvs;
    if (nvs_open(BNO055_NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) {
        return BNO055_ERROR;
    }
    char key[8];
    calib_key(id, key, sizeof(key));
    esp_err_t err = nvs_set_blob(nvs, key, &prof, sizeof(prof));
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);

    ESP_LOGI(TAG, "imu%d: calibration saved (%s)", id, esp_err_to_name(err));
    return (err == ESP_OK) ? BNO055_SUCCESS : BNO055_ERROR;
}

void bno055_save_calib_if_ready(u8 id) {
    u8 calib;
    if (calib_stored[id] || calib_read_stat(id, &calib) != BNO055_SUCCESS) {
        return;
    }

    /* Gyro and mag calibrate on their own with normal use. Accel is not
     * required: it needs special poses and barely affects the NDOF orientation */
    if (BNO055_CALIB_GYRO(calib) == 3 && BNO055_CALIB_MAG(calib) == 3) {
        calib_stored[id] = (calib_save(id) == BNO055_SUCCESS);
    }
}

// ========================== INIT ==========================

static s32 imu_setup(u8 id) {
    s32 comres = BNO055_SUCCESS;

    bno055.dev_addr = id;
    comres += bno055_init(&bno055);
    comres += bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
    calib_load(id);
    // Accel + gyro + mag: the mag keeps the heading from drifting
    comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);

    return comres;
}

s32 bno055_init_a(const bno055_bus_t bus[BNO055_I2C_BUS_COUNT]) {
    if (BNO055_i2c_bus_init(bus) != ESP_OK) {
        return BNO055_ERROR;
    }

    if (nvs_flash_init() != ESP_OK) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    bno055.bus_write = BNO055_I2C_bus_write;
    bno055.bus_read = BNO055_I2C_bus_read;
    bno055.delay_msec = BNO055_delay_msec;

    s32 comres = BNO055_SUCCESS;
    for (u8 id = 0; id < BNO055_COUNT; id++) {
        s32 imu_comres = imu_setup(id);
        ESP_LOGI(TAG, "init imu%d comres=%ld chip_id=0x%02x", id, (long)imu_comres,
                 bno055.chip_id);
        comres += imu_comres;
    }

    return comres;
}

s32 bno055_recover(u8 id) {
    i2c_master_bus_reset(s_i2c_bus[id / BNO055_IMUS_PER_BUS]);
    s32 comres = imu_setup(id);
    ESP_LOGW(TAG, "imu%d: recovered (comres=%ld)", id, (long)comres);
    return comres;
}

// ========================== QUATERNION ==========================

/* A real orientation is a unit quaternion. A corrupted I2C read, or a sensor
 * that reset itself (CONFIG mode reads 0,0,0,0), is far from it */
static bool quat_valid(const struct bno055_quaternion_t *q) {
    float n = ((float)q->w * q->w + (float)q->x * q->x + (float)q->y * q->y +
               (float)q->z * q->z) / (BNO055_QUAT_ONE * BNO055_QUAT_ONE);
    return n > 0.9f && n < 1.1f;
}

static BNO055_RETURN_FUNCTION_TYPE quat_read_raw(u8 id, struct bno055_quaternion_t *q) {
    bno055.dev_addr = id;
    if (bno055_read_quaternion_wxyz(q) != BNO055_SUCCESS || !quat_valid(q)) {
        return BNO055_ERROR;
    }
    return BNO055_SUCCESS;
}

/* Quaternion "subtraction": out = q * ref^-1 (ref is unit, so ref^-1 = conjugate).
 * Inputs and output in BNO055 units (1 = 16384) */
static void quat_diff(const struct bno055_quaternion_t *q,
                      const struct bno055_quaternion_t *ref,
                      struct bno055_quaternion_t *out) {
    const float s = BNO055_QUAT_ONE;
    float qw = q->w / s, qx = q->x / s, qy = q->y / s, qz = q->z / s;
    float rw = ref->w / s, rx = -ref->x / s, ry = -ref->y / s, rz = -ref->z / s;

    float w = qw * rw - qx * rx - qy * ry - qz * rz;
    float x = qw * rx + qx * rw + qy * rz - qz * ry;
    float y = qw * ry - qx * rz + qy * rw + qz * rx;
    float z = qw * rz + qx * ry - qy * rx + qz * rw;

    // Renormalize: the raw values are not exactly unit
    float n = sqrtf(w * w + x * x + y * y + z * z);
    out->w = (s16)lroundf(w / n * s);
    out->x = (s16)lroundf(x / n * s);
    out->y = (s16)lroundf(y / n * s);
    out->z = (s16)lroundf(z / n * s);
}

s32 struct_init(void) {
    /* The gyro calibrates by itself in a few seconds with the sensor still.
     * Taking the reference before that would let it drift away */
    TickType_t start = xTaskGetTickCount();
    for (u8 id = 0; id < BNO055_COUNT; id++) {
        u8 calib = 0;
        while ((calib_read_stat(id, &calib) != BNO055_SUCCESS ||
                BNO055_CALIB_GYRO(calib) != 3) &&
               (xTaskGetTickCount() - start) < pdMS_TO_TICKS(BNO055_REF_TIMEOUT_MS)) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (BNO055_CALIB_GYRO(calib) != 3) {
            ESP_LOGW(TAG, "imu%d: gyro not calibrated, reference taken anyway", id);
        }
    }

    s32 comres = BNO055_SUCCESS;
    for (u8 id = 0; id < BNO055_COUNT; id++) {
        s32 imu_comres = BNO055_ERROR;
        for (int i = 0; i < BNO055_REF_RETRIES && imu_comres != BNO055_SUCCESS; i++) {
            imu_comres = quat_read_raw(id, &quat_init[id]);
            if (imu_comres != BNO055_SUCCESS) {
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
        if (imu_comres != BNO055_SUCCESS) {
            ESP_LOGE(TAG, "imu%d: no valid reference", id);
        }
        comres += imu_comres;
    }

    return comres;
}

BNO055_RETURN_FUNCTION_TYPE bno055_read_quat(u8 id, struct bno055_quaternion_t *quat) {
    struct bno055_quaternion_t raw;

    if (quat_read_raw(id, &raw) != BNO055_SUCCESS) {
        return BNO055_ERROR;
    }
    quat_diff(&raw, &quat_init[id], quat);

    return BNO055_SUCCESS;
}
