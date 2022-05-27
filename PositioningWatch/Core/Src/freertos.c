/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "usart.h"
#include "adc.h"
#include "lcd.h"
#include "lvgl.h"
#include "lv_port_disp.h"
#include "demos/lv_demos.h"
#include "demos/widgets/lv_demo_widgets.h"
#include "semphr.h"
#include "key.h"
#include "lv_port_indev.h"
#include "mpu6050.h"
#include "stdlib.h"
#include "kalman_filter.h"
#include "other_function.h"
#include "math.h"
#include "my_gui.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern SemaphoreHandle_t KeyPressedSemaphore;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SysStatusTask */
osThreadId_t SysStatusTaskHandle;
const osThreadAttr_t SysStatusTask_attributes = {
    .name = "SysStatusTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for NordicComTask */
osThreadId_t NordicComTaskHandle;
const osThreadAttr_t NordicComTask_attributes = {
    .name = "NordicComTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LittleVGLTask */
osThreadId_t LittleVGLTaskHandle;
const osThreadAttr_t LittleVGLTask_attributes = {
    .name = "LittleVGLTask",
    .stack_size = 2048 * 4,
    .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for KeyGetTask */
osThreadId_t KeyGetTaskHandle;
const osThreadAttr_t KeyGetTask_attributes = {
    .name = "KeyGetTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for MPU6050Test */
osThreadId_t MPU6050TestHandle;
const osThreadAttr_t MPU6050Test_attributes = {
    .name = "MPU6050Test",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void SysStatusTask_Func(void *argument);
void NordicTask(void *argument);
void LittleVGLTaskFunc(void *argument);
void KeyGetTaskFunc(void *argument);
void MPU6050TestFunc(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* creation of SysStatusTask */
    SysStatusTaskHandle = osThreadNew(SysStatusTask_Func, NULL, &SysStatusTask_attributes);

    /* creation of NordicComTask */
    NordicComTaskHandle = osThreadNew(NordicTask, NULL, &NordicComTask_attributes);

    /* creation of LittleVGLTask */
    LittleVGLTaskHandle = osThreadNew(LittleVGLTaskFunc, NULL, &LittleVGLTask_attributes);

    /* creation of KeyGetTask */
    KeyGetTaskHandle = osThreadNew(KeyGetTaskFunc, NULL, &KeyGetTask_attributes);

    /* creation of MPU6050Test */
    MPU6050TestHandle = osThreadNew(MPU6050TestFunc, NULL, &MPU6050Test_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
    /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    for(;;)
    {
        osDelay(100);
    }
    /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_SysStatusTask_Func */
uint8_t relu100(float in) {
    if (in > 100) {
        return 100;
    } else if (in < 0) {
        return 0;
    } else
    {
        return in;
    }
}
/**
* @brief Function implementing the SysStatusTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SysStatusTask_Func */
__weak void SysStatusTask_Func(void *argument)
{
    /* USER CODE BEGIN SysStatusTask_Func */
    uint8_t electicity;
    float voltage;
    /* Infinite loop */
    for(;;)
    {
        voltage = 2.0*adc_getvoltage();
        electicity = relu100((voltage-3.45)/0.0075);
        printf("current voltage %.2f\r\n", voltage);
        printf("electicity:%d\r\n", electicity);
        osDelay(FREERTOS_S(50));
    }
    /* USER CODE END SysStatusTask_Func */
}

/* USER CODE BEGIN Header_NordicTask */
/**
* @brief Function implementing the NordicComTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_NordicTask */
void NordicTask(void *argument)
{
    /* USER CODE BEGIN NordicTask */
	static int rssi = 0;
    static uint16_t data_len;
	static float rssi_filter;
	static float distance = 0;
	static float angle1, angle2;
	static char** next_start;
	uint8_t ant_id;
    /* Infinite loop */

    for(;;)
    {
        if (uart_data.len > 0) {
			/* 判断是否有数据没接收完 */
			if (uart_data.data[uart_data.len-1] == '\n'){
				if (uart_data.data[uart_data.len-2] == '\r'){
					/* 清空数据缓冲 */
					data_len = uart_data.len;
					uart_data.len = 0;
					UART_Start_Receive_IT(&huart1, uart_data.data, 1);
					HAL_UART_Transmit(&huart3, (uint8_t *)uart_data.data, data_len, 1000);
					
					/* 循环接收数据到结束 */
					ant_id = get_rssi_aod_from_char((char*)uart_data.data, next_start, &rssi, &angle1, &angle2);
					while (1){
						if (ant_id != 0) {
							rssi_filter = kalman_filter_rssi(rssi);
							distance = pow(10, (-42.9149-rssi_filter)/24.092);
//							angle1 = angle1*57.28578;
//							angle2 = angle2*57.28578;
							set_rssi_data(1, rssi_filter, distance);
							set_aod_data(ant_id, angle1, angle2);
						}
						else {
							break;
						}
						
						ant_id = get_rssi_aod_from_char(NULL, next_start, &rssi, &angle1, &angle2);
					}
				}
			}


//			data_len = uart_data.len;
//			uart_data.len = 0;
//			UART_Start_Receive_IT(&huart1, uart_data.data, 1);
//			HAL_UART_Transmit(&huart3, (uint8_t *)uart_data.data, data_len, 1000);



			/* 处理完数据 */
        }
        osDelay(FREERTOS_MS(40));
    }
    /* USER CODE END NordicTask */
}

/* USER CODE BEGIN Header_LittleVGLTaskFunc */
/**
* @brief Function implementing the LittleVGLTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LittleVGLTaskFunc */
void LittleVGLTaskFunc(void *argument)
{
    /* USER CODE BEGIN LittleVGLTaskFunc */
    lv_init();
    lv_port_disp_init();
    lv_port_indev_init();
    //lv_demo_keypad_encoder();
	my_gui_start();
    /* Infinite loop */
    for(;;)
    {
        lv_timer_handler();
        osDelay(FREERTOS_MS(32));
    }
    /* USER CODE END LittleVGLTaskFunc */
}

/* USER CODE BEGIN Header_KeyGetTaskFunc */
/**
* @brief Function implementing the KeyGetTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_KeyGetTaskFunc */
void KeyGetTaskFunc(void *argument)
{
    /* USER CODE BEGIN KeyGetTaskFunc */
    KeyPressedSemaphore = xSemaphoreCreateBinary();
    /* Infinite loop */
    for(;;)
    {
        xSemaphoreTake(KeyPressedSemaphore, portMAX_DELAY);
        osDelay(FREERTOS_MS(4));
        /* 根据输入的按键进行判断 */
        if (whitch_key_input == KEY_Pin) {
            if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == 0) {
                key_status = 1;
                printf("key pressed\r\n");
            }
        } else if (whitch_key_input == E5E8_K_Pin) {
            if (HAL_GPIO_ReadPin(E5E8_K_GPIO_Port, E5E8_K_Pin) == 0) {
                e5e8_status.key = 1;
                encoder_handler(0, 1);
                printf("e5e8 key pressed\r\n");
            }
        } else if (whitch_key_input == E5E8_A_Pin) {
            if (HAL_GPIO_ReadPin(E5E8_A_GPIO_Port, E5E8_A_Pin) == 0) {
                /* 根据另一个引脚的高低电平判断是往哪边旋转 */
                if (HAL_GPIO_ReadPin(E5E8_B_GPIO_Port, E5E8_B_Pin) == 1) {
                    e5e8_status.left = 1;
                    encoder_handler(1, 0);
                    printf("e5e8 turn left\r\n");
                } else {
                    e5e8_status.right = 1;
                    encoder_handler(-1, 0);
                    printf("e5e8 turn right\r\n");
                }
            }
        }
    }
    /* USER CODE END KeyGetTaskFunc */
}

/* USER CODE BEGIN Header_MPU6050TestFunc */
/**
* @brief Function implementing the MPU6050Test thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MPU6050TestFunc */
void MPU6050TestFunc(void *argument)
{
    /* USER CODE BEGIN MPU6050TestFunc */
    static uint8_t remain_num = 0;
	static unsigned long time_stamp = 0;
	static short accel[3];
	static float euler_angle[3];			//欧拉角
    static short temp;						//温度
	static float displacement[3] = {0};
	int ret;
	
	static unsigned long step_count = 0;
	
	mpu6050_Init();
    osDelay(8);
    while(mpu_dmp_init())
    {
        printf("MPU6050 Error\r\n");
        osDelay(2000);
    }
    printf("mpu6050 init succeed!\r\n");
    /* Infinite loop */
    for(;;)
    {
		/*********** 处理完fifo数据 ***********/
//		do {
//			i = my_mpu_dmp_get_data(euler_angle, accel, &time_stamp, &remain_num);
//			if (i==0) {
//				get_displacement_data(accel, euler_angle, displacement);
//			}
//			else {
//				printf("mpu6050 data get failed (-%d)\r\n", i);
//				//osDelay(FREERTOS_MS(1000));
//			}
//		}while(remain_num);
//		printf("%f, %f, %f\r\n", displacement[0],displacement[1],displacement[2]);
		
		ret = dmp_get_pedometer_step_count(&step_count);
		if (ret) {
			printf("mpu6050 data get failed (-%d)\r\n", ret);
		}
		else {
			imu_set_step(step_count);
		}
		
        osDelay(FREERTOS_S(5));
    }
    /* USER CODE END MPU6050TestFunc */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

