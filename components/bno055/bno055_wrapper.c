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

static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_i2c_dev = NULL;

/*----------------------------------------------------------------------------*
 *  struct bno055_t parameters can be accessed by using BNO055
 *  BNO055_t having the following parameters
 *  Bus write function pointer: BNO055_WR_FUNC_PTR
 *  Bus read function pointer: BNO055_RD_FUNC_PTR
 *  Burst read function pointer: BNO055_BRD_FUNC_PTR
 *  Delay function pointer: delay_msec
 *  I2C address: dev_addr
 *  Chip id of the sensor: chip_id
 *---------------------------------------------------------------------------*/
static struct bno055_t bno055;
static u8 power_mode = BNO055_INIT_VALUE;

/* La API de Bosch direcciona por registro: escritura = [reg_addr | payload]  */
static s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 len) {
    (void)dev_addr; // ignore

    if (s_i2c_dev == NULL || len > (BNO055_I2C_WR_BUF_LEN - 1)) {
        return BNO055_ERROR;
    }

    u8 buf[BNO055_I2C_WR_BUF_LEN];
    buf[0] = reg_addr;
    for (u8 i = 0; i < len; i++) {
        buf[i + 1] = reg_data[i];
    }

    esp_err_t err = i2c_master_transmit(s_i2c_dev, buf, (size_t)len + 1,
                                        BNO055_I2C_TIMEOUT_MS);

    return (err == ESP_OK) ? BNO055_SUCCESS : BNO055_ERROR;
}

static s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 len) {
    (void)dev_addr; // ignore

    if (s_i2c_dev == NULL) {
        return BNO055_ERROR;
    }

    esp_err_t err = i2c_master_transmit_receive(s_i2c_dev, &reg_addr, 1,
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
    if (s_i2c_dev != NULL) {
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CONFIG_BOARD_I2C_SDA,
        .scl_io_num = CONFIG_BOARD_I2C_SCL,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus: %s", esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CONFIG_BNO055_I2C_ADDR,
        .scl_speed_hz = BNO055_I2C_FREQ_HZ,
    };

    err = i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_i2c_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device: %s", esp_err_to_name(err));
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
        return err;
    }

    return ESP_OK;
}

static struct bno055_accel_t accel_init;
static struct bno055_gyro_t gyro_init;
static struct bno055_euler_t euler_init;

// Lectura cruda, sin restar nada — la usa struct_init() y bno055_read()
static s32 bno055_read_raw(struct bno055_accel_t *accel_out,
                           struct bno055_gyro_t *gyro_out) {
    s32 comres = BNO055_SUCCESS;
    comres += bno055_read_accel_xyz(accel_out);
    comres += bno055_read_gyro_xyz(gyro_out);
    return comres;
}

void struct_init(void) {
    bno055_read_raw(&accel_init, &gyro_init);
    bno055_read_euler_hrp(&euler_init); // raw también, sin restar
}

s32 bno055_init_a(void) {
    s32 comres = BNO055_ERROR;

    if (BNO055_i2c_bus_init() != ESP_OK) {
        return BNO055_ERROR;
    }

    bno055.bus_write = BNO055_I2C_bus_write;
    bno055.bus_read = BNO055_I2C_bus_read;
    bno055.delay_msec = BNO055_delay_msec;
    bno055.dev_addr = CONFIG_BNO055_I2C_ADDR;

    comres = bno055_init(&bno055);

    power_mode = BNO055_POWER_MODE_NORMAL;
    comres += bno055_set_power_mode(power_mode);

#if CONFIG_BNO055_OPMODE_NDOF
    comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
#else
    // Inertial measurement unit.
    // Reads accel,gyro and fusion data
    comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_IMUPLUS);
#endif
    comres += bno055_set_euler_unit(BNO055_EULER_UNIT_DEG);

    ESP_LOGI(TAG, "init comres=%ld chip_id=0x%02x", (long)comres,
             bno055.chip_id);

    return comres;
}

s32 bno055_read(struct bno055_accel_t *accel_out,
                struct bno055_gyro_t *gyro_out) {
    s32 comres = BNO055_SUCCESS;

    /*  Raw accel X, Y and Z data can read from the register
     * page - page 0
     * register - 0x08 to 0x0D*/
    comres += bno055_read_accel_xyz(accel_out);

    /*  Raw gyro X, Y and Z data can read from the register
     * page - page 0
     * register - 0x14 to 0x19*/
    comres += bno055_read_gyro_xyz(gyro_out);

    gyro_out->x = gyro_out->x - gyro_init.x;
    gyro_out->y = gyro_out->y - gyro_init.y;
    gyro_out->z = gyro_out->z - gyro_init.z;

    accel_out->x = accel_out->x - accel_init.x;
    accel_out->y = accel_out->y - accel_init.y;
    accel_out->z = accel_out->z - accel_init.z;

    return comres;
}

s32 bno055_convert_double(struct bno055_accel_double_t *accel_out,
                          struct bno055_gyro_double_t *gyro_out) {
    s32 comres = BNO055_SUCCESS;

    /*  API used to read accel data output as double  - m/s2 and mg
     * float functions also available in the BNO055 API */
    comres += bno055_convert_double_accel_xyz_msq(accel_out);

    /*  API used to read gyro data output as double  - dps and rps
     * float functions also available in the BNO055 API */
    comres += bno055_convert_double_gyro_xyz_dps(gyro_out);

    return comres;
}

BNO055_RETURN_FUNCTION_TYPE bno055_read_euler(struct bno055_euler_t *euler) {
    BNO055_RETURN_FUNCTION_TYPE comres = BNO055_SUCCESS;
    comres += bno055_read_euler_hrp(euler);

    euler->h = euler->h - euler_init.h;
    euler->p = euler->p - euler_init.p;
    euler->r = euler->r - euler_init.r;

    return comres;
}

s32 bno055_stop(void) {
    s32 comres = BNO055_SUCCESS;

    power_mode = BNO055_POWER_MODE_SUSPEND;
    comres += bno055_set_power_mode(power_mode);

    return comres;
}