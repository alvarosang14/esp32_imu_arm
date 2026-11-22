// bno055_wrapper.h
#ifndef BNO055_WRAPPER_H
#define BNO055_WRAPPER_H

#include "BNO055/bno055.h"

#define BNO055_I2C_BUS_COUNT 2
#define BNO055_IMUS_PER_BUS  2

/* Number of IMUs. `id` = bus * BNO055_IMUS_PER_BUS + slot, from 0 to BNO055_COUNT - 1 */
#define BNO055_COUNT (BNO055_I2C_BUS_COUNT * BNO055_IMUS_PER_BUS)

/* One I2C bus: its pins and the address of each IMU on it */
typedef struct {
    int sda;
    int scl;
    u8 addr[BNO055_IMUS_PER_BUS];
} bno055_bus_t;

/* Starts the I2C buses and the BNO055_COUNT sensors in NDOF mode */
s32 bno055_init_a(const bno055_bus_t bus[BNO055_I2C_BUS_COUNT]);

/* Takes the reference orientation that bno055_read_quat() subtracts.
 * Call it with the arm still: waits for the gyros to be calibrated first */
s32 struct_init(void);

/* Fusion quaternion relative to the struct_init() reference: q * q_init^-1.
 * 1 = 2^14 (16384); it is 16384,0,0,0 in the reference position.
 * A failed or invalid read returns an error and leaves `quat` untouched */
BNO055_RETURN_FUNCTION_TYPE bno055_read_quat(u8 id, struct bno055_quaternion_t *quat);

/* Resets the bus of `id` and sets the sensor up again (after repeated failures) */
s32 bno055_recover(u8 id);

/* Saves the calibration of `id` in NVS the first time gyro and mag are fully
 * calibrated. Does nothing once a profile exists (loaded at boot or saved) */
void bno055_save_calib_if_ready(u8 id);

#endif
