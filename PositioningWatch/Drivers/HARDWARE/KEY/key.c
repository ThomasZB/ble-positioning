#include "key.h"
#include "main.h"
#include "freertos.h"
#include "task.h"
#include "stdio.h"
#include "semphr.h"

volatile uint16_t whitch_key_input = 100;
volatile uint8_t key_status = 0;
e5e8_status_type e5e8_status = {0, 0, 0};
SemaphoreHandle_t KeyPressedSemaphore = NULL;


/* 外部中断目前只用来处理按键，就加在这里吧 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	whitch_key_input = GPIO_Pin;
	if (KeyPressedSemaphore != NULL){
		xSemaphoreGiveFromISR(KeyPressedSemaphore, &xHigherPriorityTaskWoken);
	}
}