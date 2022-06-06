#ifndef __FIR_FILTER_H
#define __FIR_FILTER_H

#include "stdint.h"

void aod_fir_filter_init(void);
float aod_fir_filter(uint8_t ant_id, float angle);


#endif