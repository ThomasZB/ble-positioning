/**
 * @file my_gui.h
 * @author hang chen (thomaszb@qq.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __MY_GUI_H
#define __MY_GUI_H

#include "stdint.h"


#define IMU_MAX_LEN 200

extern char num[3];


void my_gui_start(void);
uint8_t imu_set_step(unsigned long step_count);
void set_rssi_data(uint8_t ant_num, int rssi, float distance);
void set_aod_data(uint8_t ant_num, float pitch_angle, float yaw_angle);





#endif

