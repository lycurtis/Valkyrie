#include "globals.h"

comp imu_CA = {0};
float roll  = 0.0f;
float pitch = 0.0f;
volatile uint16_t ibus_channels[6] = {0};
