#include "other_function.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "arm_math.h"

int get_rssi_aod_from_char(char *c, char** next_start, int* rssi, float* angle1, float* angle2){
	char *str_pos = NULL;
	uint8_t id;
	
	if (c != NULL){
		str_pos = strstr(c, "id:");
	}
	else{
		str_pos = strstr(*next_start, "id:");
	}
	
	/* 是需要的信号 */
	if (str_pos != NULL){
		/* 得到天线id */
		id = atoi(str_pos+3);
		/* 得到rssi */
		str_pos = strstr(c, "rssi:");
		*rssi = atoi(str_pos+5);
		/* 得到角度 */
		str_pos = strstr(c, "angle:");
		sscanf((str_pos+6), "%f %f", angle1, angle2);
		
		*next_start = strchr(str_pos, '\n');
		
		return id;
	}
	
	return 0;
}



uint8_t get_displacement_data(short* accel, float* euler_angle, float* displacement){
	/* 设置为静态变量，避免频繁进出栈 */
	static float a[2] = {0};	/* 偏航 */
	static float b[2] = {0};	/* 俯仰 */
	static float c[2] = {0};	/* 横滚 */
	static float v[3] = {0};
	static short new_accel[3] = {0};
	static float delta_t = 0.01;
	static float acc[3] = {0};
	
	arm_sin_cos_f32(euler_angle[2], &a[0], &a[1]);
	arm_sin_cos_f32(euler_angle[0], &b[0], &b[1]);
	arm_sin_cos_f32(euler_angle[1], &c[0], &c[1]);
	
	/* 新滚仰航 */
	new_accel[0] = (a[1]*b[1])*accel[0] + (a[1]*b[0]*c[0]-a[0]*c[1])*accel[1] + (a[0]*c[0]+a[1]*b[0]*c[1])*accel[2];
	new_accel[1] = (a[0]*b[1])*accel[0] + (a[1]*c[1]+a[0]*b[0]*c[0])*accel[1] + (a[0]*b[0]*c[1]-a[1]*c[0])*accel[2];
	new_accel[2] = (-b[0])*accel[0] + (b[1]*c[0])*accel[1] + (b[1]*c[1])*accel[2];
	
	/* 新航仰滚 */
//	new_accel[0] = (a[1]*b[1])*accel[0] + (-a[0]*b[1])*accel[1] + (b[0])*accel[2];
//	new_accel[1] = (a[0]*c[1]+a[1]*b[0]*c[0])*accel[0] + (a[1]*c[1]-a[0]*b[0]*c[0])*accel[1] + (-b[1]*c[0])*accel[2];
//	new_accel[2] = (a[0]*c[0]-a[1]*b[0]*c[1])*accel[0] + (a[1]*c[0]+a[0]*b[0]*c[1])*accel[1] + (b[1]*c[1])*accel[2];
	
	
//	printf("%.1f, %.1f, %.1f\r\n", euler_angle[0],euler_angle[1],euler_angle[2]);
//	printf("%d, %d, %d\r\n", new_accel[0],new_accel[1],new_accel[2]);
	acc[0] = 0.00059762*new_accel[0] - 0.3978;
	acc[1] = 0.00059762*new_accel[1] + 0.1812;
	acc[2] = 0.00059762*new_accel[2]-9.7914 - 0.4208;
	//printf("%.4f %.4f %.4f\r\n", acc[0],acc[1],acc[2]);
	v[0] = v[0] + acc[0] * delta_t;
	v[1] = v[1] + acc[1] * delta_t;
	v[2] = v[2] + acc[2] * delta_t;
	
	/* 得到位移 */
	displacement[0] = displacement[0] + v[0]*delta_t;
	displacement[1] = displacement[1] + v[1]*delta_t;
	displacement[2] = displacement[2] + v[2]*delta_t;
}
	