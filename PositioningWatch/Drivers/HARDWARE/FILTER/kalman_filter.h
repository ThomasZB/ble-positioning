#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

#include "main.h"


float kalman_filter_rssi(int rssi);
float kalman_filter_aod1(float aod_data);
float kalman_filter_aod2(float aod_data);
float kalman_filter_aod3(float aod_data);
float kalman_filter_aod(int ant_id, float aod_data);
float kalman_filter_rssi_to_distance(int rssi);


#endif


