/**
 * @file main.c
 * @author hang chen (thomaszb.cn)
 * @brief 
 * @version 0.1
  * @date 2022-04-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "zephyr.h"
#include "mybluetooth.h"
#include "myuart.h"
#include "direction_finding.h"
#include "uart_client.h"
#include "nrf_reg.h"
#include "arm_math.h"
#include "math.h"


/* 创建和32通信线程，优先级较低 */
K_THREAD_DEFINE(ble2stm32_thread_id, BLE2STM32_STACKSIZE, ble2stm32_thread, NULL, NULL,
		NULL, BLE2STM32_PRIORITY+1, 0, 0);

/* 创建接收基站1周期性广播线程 */
K_THREAD_DEFINE(perodic_adv_thread1_id, PERODIC_ADV_STACKSIZE, perodic_adv_thread1, NULL, NULL,
		NULL, PERODIC_ADV_PRIORITY, 0, 0);

// /* 创建接收基站2周期性广播线程 */
// K_THREAD_DEFINE(perodic_adv_thread2_id, PERODIC_ADV_STACKSIZE, perodic_adv_thread2, NULL, NULL,
// 		NULL, PERODIC_ADV_PRIORITY, 0, 0);

// /* 创建接收基站3周期性广播线程 */
// K_THREAD_DEFINE(perodic_adv_thread3_id, PERODIC_ADV_STACKSIZE, perodic_adv_thread3, NULL, NULL,
// 		NULL, PERODIC_ADV_PRIORITY, 0, 0);


void main(void)
{
	int err;
	int i=0 ;
	
	/* 串口初始化 */
 	err = uart_init();
	k_msleep(1000);
    printk("locater started\r\n");
    if (err){
		printk("uart init failed, errcode: %d\r\n", err);
	}

	/* 使能蓝牙协议栈 */
	err = bt_enable(NULL);
	if (err){
		printk("ble init failed, errcode: %d\r\n", err);
	}
	printk("ble init success!\r\n");
	ble_had_been_inited = 1;

	/* 使能扫描 */
	scan_init();
	bt_scan_enable();

	/* 使能direction finding */
	direction_finding_init();

    /* 发送蓝牙就绪信号量 */
	k_sem_give(&my_ble_ready);

	/* 开始AOD测量 */
	while (1){
		/* 使能扫描，若未连接，进行5ms扫描 */
		if (current_conn == NULL){
			bt_switch_conn();
			//bt_scan_enable();
			k_msleep(500);
		}
		if (gatt_had_been_find && current_conn){
			//bt_uart_client_read();
			//bt_uart_client_send("hello", 5);
		}
		if (i++ == 20){
			printk("Hello world\r\n");
		}
		k_msleep(2000);
	}
}






