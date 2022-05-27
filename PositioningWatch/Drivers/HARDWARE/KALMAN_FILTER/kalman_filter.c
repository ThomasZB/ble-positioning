#include "kalman_filter.h"
#include "math.h"

/**
 * @brief			：使用卡尔曼滤波对rssi进行滤波
 * 
 * @param rssi 		：接收到的信号强度
 * @return float 	：距离
 */
float kalman_filter_rssi_to_distance(int rssi){
	/* 基本参数 */
	static int 		A				= -65			;	/* RSSI参数（可以线性回归，可以1m处信号强度代替） */
	static float	lgd_measure	 	= 0				;	/* 距离测量值 */
	static float 	lgd 			= 0				;	/* 距离预测值 */
	static int		last_rssi		= 0				;	/* 保留上一次的rssi用于预测 */
	static int 		drssi 			= 0				;	/* 差分rssi */
	static float	n				= 3				;	/* 传播因子 */
	static float	Q_bias			= 0.01			;	/* 偏差 */
	static float	P[2][2]			= {{1,0},
										{0,1}}	;		/* 协方差矩阵 */
	static float	K[2]			= {0,0}			;	/* 卡尔曼滤波参数 */ 
	
	/* 超参数 */
	static float	R				= 0.01			;	/* 观测噪声方差 */
	static float	Q[2][2]			= {{0.04,0},
										{0,0.02}}	;	/* 过程噪声方差 */

	
	/* 预测lgd */
	lgd = lgd + Q_bias - (1.0*drssi/(10*n));
	
	/* 预测协方差矩阵 */
	P[0][0] = P[0][0] + P[1][0] + P[0][1] + P[1][1] + Q[0][0];
	P[0][1] = P[0][1] + P[1][1] + Q[0][1];
	P[1][0] = P[1][0] + P[1][1] + Q[1][0];
	P[1][1] = P[1][1] + Q[1][1];
	
	/* 更新卡尔曼滤波参数 */
	K[0] = P[0][0] / (P[0][0]+R);
	K[1] = P[1][0] / (P[0][0]+R);
	
	/* 得到测量 */
	lgd_measure = 1.0*A/(10*n) - 1.0*rssi/(10*n);
	
	/* 得到结果值 */
	lgd 	= lgd + K[0]*(lgd_measure-lgd);
	Q_bias 	= Q_bias + K[1]*(lgd_measure-lgd);
	
	/* 更新协方差矩阵 */
	P[0][0] = (1-K[0]) * P[0][0];
	P[0][1] = (1-K[0]) * P[0][1];
	P[1][0] = P[1][0] - K[1]*P[0][0];
	P[1][1] = P[1][1] - K[1]*P[0][1];
	
	/* 更新差分rssi */
	drssi = rssi - last_rssi;
	last_rssi = rssi;
	
	return pow(10, lgd);
}



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