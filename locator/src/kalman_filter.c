#include "kalman_filter.h"



/**
 * @brief			：使用卡尔曼滤波对rssi进行滤波
 * 
 * @param rssi 		：接收到的信号强度
 * @return float 	：rssi
 */
float kalman_filter_rssi(int rssi){
	/* 基本参数 */
	static float 	A				= -64.0867		;	/* RSSI参数（可以线性回归，可以1m处信号强度代替） */
	static float	rssi_forecast	= 0				;
	static float	n				= 0.4518		;	/* 传播因子 */
	static float	P				= 10			;	/* 协方差矩阵 */
	static float	K				= 0				;	/* 卡尔曼滤波参数 */ 
	
	/* 超参数 */
	static float	R				= 0.05			;	/* 观测噪声方差 */
	static float	Q				= 0.01	;	/* 过程噪声方差 */

	
	/* 预测lgd */
	rssi_forecast = rssi_forecast;
	
	/* 预测协方差矩阵 */
	P = P + Q;
	
	/* 更新卡尔曼滤波参数 */
	K = P / (P+R);
	
	/* 得到结果值 */
	rssi_forecast = rssi_forecast  + K*(rssi-rssi_forecast);
	
	/* 更新协方差矩阵 */
	P = (1-K)*P;
	
	return rssi_forecast;
}