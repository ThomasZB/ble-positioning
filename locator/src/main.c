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


void main(void)
{
	int err;
	
	/* 串口初始化 */
	err = uart_init();
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

	/* 开始AOD测量 */
	while (1){
        per_adv_found = 0;
		/* 使能扫描，若未连接，进行5ms扫描 */
		if (current_conn == NULL){
			bt_switch_conn();
			bt_scan_enable();
			k_msleep(5);
			bt_scan_disable();
		}
		bt_switch_df();
        bt_scan_enable();
		
		/* 找到对应的AOD广播设备 */
		wait_central_adv();

		/* 进行同步 */
		create_sync_handle();
		printk("Waiting for periodic sync...\r\n");
        err = k_sem_take(&sem_per_sync, K_MSEC(sync_create_timeout_ms));
        if (err) {
            printk("failed waitting (err %d)\r\n", err);
            err = delete_sync();
            if (err){
                return;
            }
            continue;
        }
        printk("success. Periodic sync established.\r\n");

		/* 接收CTE信息 */
		err = enable_cte_rx();
        if (err) {
            printk("failed waitting (err %d)\r\n", err);
            err = delete_sync();
            if (err){
                return;
            }
            continue;
        }
        printk("success. Periodic sync established.\r\n");
		
		/* 停止一次采集，准备下次采集 */
		bt_scan_disable();
		wait_sync_lost();
		k_msleep(2000);
		printk("hello world\r\n");
	}
}
