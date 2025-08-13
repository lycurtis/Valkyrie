#pragma once
#include <stdint.h>
#include "complementary.h"  // for comp struct

extern comp imu_CA;
extern float roll;
extern float pitch;
extern volatile uint16_t ibus_channels[6];
