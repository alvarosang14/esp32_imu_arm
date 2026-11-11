#include "bno055_wrapper/bno055_wrapper.h"
#include "BNO055/bno055.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BNO055_I2C_TIMEOUT_MS 100
#define BNO055_I2C_FREQ_HZ 400000
#define BNO055_I2C_WR_BUF_LEN 32

static const char* TAG = "bno055";

#define BNO055_I2C_BUS_COUNT 2

/* Two IMUs share bus 0 (different ADR) and the third one is alone on bus 1 */
static const i2c_port_num_t imu_port[BNO055_COUNT] = {I2C_NUM_0, I2C_NUM_0, I2C_NUM_1};
static const u8 imu_addr[BNO055_COUNT] = {BNO055_I2C_ADDR1, BNO055_I2C_ADDR2, BNO055_I2C_ADDR1};

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
static u8 power_mode = BNO055_INIT_VALUE;

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

static esp_err_t BNO055_i2c_bus_init(void) {
    if (s_i2c_bus[I2C_NUM_0] != NULL) {
        return ESP_OK;
    }

    const int sda[BNO055_I2C_BUS_COUNT] = {CONFIG_BOARD_I2C_SDA, CONFIG_BOARD_I2C1_SDA};
    const int scl[BNO055_I2C_BUS_COUNT] = {CONFIG_BOARD_I2C_SCL, CONFIG_BOARD_I2C1_SCL};

    for (int port = 0; port < BNO055_I2C_BUS_COUNT; port++) {
        i2c_master_bus_config_t bus_cfg = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = port,
            .sda_io_num = sda[port],
            .scl_io_num = scl[port],
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };

        esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_i2c_bus[port]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2c_new_master_bus %d: %s", port, esp_err_to_name(err));
            return err;
        }
    }

    for (int id = 0; id < BNO055_COUNT; id++) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = imu_addr[id],
            .scl_speed_hz = BNO055_I2C_FREQ_HZ,
        };

        esp_err_t err = i2c_master_bus_add_device(s_i2c_bus[imu_port[id]], &dev_cfg, &s_i2c_dev[id]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2c_master_bus_add_device %d: %s", id, esp_err_to_name(err));
            return err;
        }
    }

    return ESP_OK;
}

static struct bno055_accel_t accel_init[BNO055_COUNT];
static struct bno055_gyro_t gyro_init[BNO055_COUNT];
static struct bno055_euler_t euler_init[BNO055_COUNT];

// Raw read, nothing subtracted - used by struct_init() and bno055_read()
static s32 bno055_read_raw(struct bno055_accel_t *accel_out,
                           struct bno055_gyro_t *gyro_out) {
    s32 comres = BNO055_SUCCESS;
    comres += bno055_read_accel_xyz(accel_out);
    comres += bno055_read_gyro_xyz(gyro_out);
    return comres;
}

void struct_init(void) {
    for (u8 id = 0; id < BNO055_COUNT; id++) {
        bno055.dev_addr = id;
        bno055_read_raw(&accel_init[id], &gyro_init[id]);
        bno055_read_euler_hrp(&euler_init[id]); // raw too, nothing subtracted
    }
}

s32 bno055_init_a(void) {
    s32 comres = BNO055_ERROR;

    if (BNO055_i2c_bus_init() != ESP_OK) {
        return BNO055_ERROR;
    }

    bno055.bus_write = BNO055_I2C_bus_write;
    bno055.bus_read = BNO055_I2C_bus_read;
    bno055.delay_msec = BNO055_delay_msec;
    power_mode = BNO055_POWER_MODE_NORMAL;

    comres = BNO055_SUCCESS;
    for (u8 id = 0; id < BNO055_COUNT; id++) {
        bno055.dev_addr = id;

        comres += bno055_init(&bno055);
        comres += bno055_set_power_mode(power_mode);

#if CONFIG_BNO055_OPMODE_NDOF
        comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
#else
        // Inertial measurement unit.
        // Reads accel,gyro and fusion data
        comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_IMUPLUS);
#endif
        comres += bno055_set_euler_unit(BNO055_EULER_UNIT_DEG);

        ESP_LOGI(TAG, "init imu%d comres=%ld chip_id=0x%02x", id, (long)comres,
                 bno055.chip_id);
    }

    return comres;
}

s32 bno055_read(u8 id, struct bno055_accel_t *accel_out,
                struct bno055_gyro_t *gyro_out) {
    s32 comres = BNO055_SUCCESS;

    bno055.dev_addr = id;

    /*  Raw accel X, Y and Z data can read from the register
     * page - page 0
     * register - 0x08 to 0x0D*/
    comres += bno055_read_accel_xyz(accel_out);

    /*  Raw gyro X, Y and Z data can read from the register
     * page - page 0
     * register - 0x14 to 0x19*/
    comres += bno055_read_gyro_xyz(gyro_out);

    gyro_out->x = gyro_out->x - gyro_init[id].x;
    gyro_out->y = gyro_out->y - gyro_init[id].y;
    gyro_out->z = gyro_out->z - gyro_init[id].z;

    accel_out->x = accel_out->x - accel_init[id].x;
    accel_out->y = accel_out->y - accel_init[id].y;
    accel_out->z = accel_out->z - accel_init[id].z;

    return comres;
}

s32 bno055_convert_double(u8 id, struct bno055_accel_double_t *accel_out,
                          struct bno055_gyro_double_t *gyro_out) {
    s32 comres = BNO055_SUCCESS;

    bno055.dev_addr = id;

    /*  API used to read accel data output as double  - m/s2 and mg
     * float functions also available in the BNO055 API */
    comres += bno055_convert_double_accel_xyz_msq(accel_out);

    /*  API used to read gyro data output as double  - dps and rps
     * float functions also available in the BNO055 API */
    comres += bno055_convert_double_gyro_xyz_dps(gyro_out);

    return comres;
}

BNO055_RETURN_FUNCTION_TYPE bno055_read_euler(u8 id, struct bno055_euler_t *euler) {
    BNO055_RETURN_FUNCTION_TYPE comres = BNO055_SUCCESS;

    bno055.dev_addr = id;
    comres += bno055_read_euler_hrp(euler);

    euler->h = euler->h - euler_init[id].h;
    euler->p = euler->p - euler_init[id].p;
    euler->r = euler->r - euler_init[id].r;

    return comres;
}

s32 bno055_stop(void) {
    s32 comres = BNO055_SUCCESS;

    power_mode = BNO055_POWER_MODE_SUSPEND;
    for (u8 id = 0; id < BNO055_COUNT; id++) {
        bno055.dev_addr = id;
        comres += bno055_set_power_mode(power_mode);
    }

    return comres;
}