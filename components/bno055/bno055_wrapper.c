#include "bno055_wrapper/bno055_wrapper.h"
#include "BNO055/bno055.h"
#include "i2c.h"

#define BNO055_I2C_TIMEOUT_MS 100

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

static s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 len) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
        &hi2c1, (uint16_t)(dev_addr << 1), reg_addr, I2C_MEMADD_SIZE_8BIT,
        reg_data, len, BNO055_I2C_TIMEOUT_MS);

    return (status == HAL_OK) ? BNO055_SUCCESS : BNO055_ERROR;
}

static s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 len) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        &hi2c1, (uint16_t)(dev_addr << 1), reg_addr, I2C_MEMADD_SIZE_8BIT,
        reg_data, len, BNO055_I2C_TIMEOUT_MS);

    return (status == HAL_OK) ? BNO055_SUCCESS : BNO055_ERROR;
}

static void BNO055_delay_msec(u32 msec) { HAL_Delay(msec); }

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

    bno055.bus_write = BNO055_I2C_bus_write;
    bno055.bus_read = BNO055_I2C_bus_read;
    bno055.delay_msec = BNO055_delay_msec;
    bno055.dev_addr = BNO055_I2C_ADDR1;

    comres = bno055_init(&bno055);

    power_mode = BNO055_POWER_MODE_NORMAL;
    comres += bno055_set_power_mode(power_mode);

    // Inertial measurement unit.
    // Reads accel,gyro and fusion data
    comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_IMUPLUS);
    comres += bno055_set_euler_unit(BNO055_EULER_UNIT_DEG);

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