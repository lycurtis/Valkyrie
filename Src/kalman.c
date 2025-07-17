/*
 * kalman.c
 *
 *  Created on: Jul 13, 2025
 *      Author: cly
 */

#include "kalman.h"

/* Complementary Filter
 * angle = α * (angle + gyro * dt) + (1 - α) * accel_angle;
 * */

