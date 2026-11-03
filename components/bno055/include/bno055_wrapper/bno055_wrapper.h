// bno055_wrapper.h
#ifndef BNO055_WRAPPER_H
#define BNO055_WRAPPER_H

#include "BNO055/bno055.h"

s32 bno055_init_a(void);

/* Toma la lectura de referencia que bno055_read()/bno055_read_euler() restan */
void struct_init(void);

s32 bno055_read(struct bno055_accel_t *accel_out,
                struct bno055_gyro_t *gyro_out);

s32 bno055_convert_double(struct bno055_accel_double_t *accel_out,
                          struct bno055_gyro_double_t *gyro_out);

BNO055_RETURN_FUNCTION_TYPE bno055_read_euler(struct bno055_euler_t *euler);

s32 bno055_stop(void);

#endif