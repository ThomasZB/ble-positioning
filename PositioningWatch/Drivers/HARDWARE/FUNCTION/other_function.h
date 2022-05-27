#ifndef __OTHER_FUNCTION_H
#define __OTHER_FUNCTION_H

#include "stdint.h"


uint8_t get_displacement_data(short* accel, float* euler_angle, float* displacement);
int get_rssi_aod_from_char(char *c, char** next_start, int* rssi, float* angle1, float* angle2);

#endif

