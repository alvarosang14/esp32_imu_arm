// bno055_wrapper.h
#ifndef BNO055_WRAPPER_H
#define BNO055_WRAPPER_H

#include "BNO055/bno055.h"

/* Number of IMUs. `id` goes from 0 to BNO055_COUNT - 1 */
#define BNO055_COUNT 3

/* Starts both I2C buses and the BNO055_COUNT sensors */
s32 bno055_init_a(void);

/* Takes the reference reading that bno055_read()/bno055_read_euler() subtract */
void struct_init(void);

s32 bno055_read(u8 id, struct bno055_accel_t *accel_out,
                struct bno055_gyro_t *gyro_out);

s32 bno055_convert_double(u8 id, struct bno055_accel_double_t *accel_out,
                          struct bno055_gyro_double_t *gyro_out);

/* Euler in 1/16 of a degree */
BNO055_RETURN_FUNCTION_TYPE bno055_read_euler(u8 id, struct bno055_euler_t *euler);

/* Suspends all the sensors */
s32 bno055_stop(void);

#endif