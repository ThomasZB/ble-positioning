/**
 * @file nrf_reg.h
 * @author your name (you@domain.com)
 * @brief 寄存器的相关定义
 * @version 0.1
 * @date 2022-04-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __NRF_REG_H
#define __NRF_REG_H

#include "stdint.h"

#define RADIO_BASS_ADD 0x40001000

#define DEFMODE (*(uint32_t *)(RADIO_BASS_ADD+0x900))



#endif