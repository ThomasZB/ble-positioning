#include "fir_filter.h"
#include "fdacoefs.h"
#include "arm_math.h"


static float32_t firStateF32_1[BL + 16 - 1];
static float32_t firStateF32_2[BL + 16 - 1];
static float32_t firStateF32_3[BL + 16 - 1];
static arm_fir_instance_f32 fir_handle1;
static arm_fir_instance_f32 fir_handle2;
static arm_fir_instance_f32 fir_handle3;


void aod_fir_filter_init(void){
	arm_fir_init_f32(&fir_handle1, BL, (float32_t*)B, firStateF32_1, 1);
	arm_fir_init_f32(&fir_handle2, BL, (float32_t*)B, firStateF32_2, 1);
	arm_fir_init_f32(&fir_handle3, BL, (float32_t*)B, firStateF32_3, 1);
}


float aod_fir_filter(uint8_t ant_id, float angle){
	float output = 0;
	if (ant_id == 1){
		arm_fir_f32(&fir_handle1, &angle, &output, 1);
	}
	else if (ant_id == 2){
		arm_fir_f32(&fir_handle2, &angle, &output, 1);
	}
	else if (ant_id == 3){
		arm_fir_f32(&fir_handle3, &angle, &output, 1);
	}
	return output;
}