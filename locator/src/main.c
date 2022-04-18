/****
 * 
 * 
 * */
#include "zephyr.h"
#include "mybluetooth.h"
#include "myuart.h"
#include "uart_profile.h"

static K_SEM_DEFINE(ble_init_ok, 0, 1);



void main(void)
{
	int err;
	
	/* 串口初始化 */
	uart_init();

	/* 使能蓝牙协议栈 */
	err = bt_enable(NULL);
	if (err){
		printk("ble init failed, errcode: %d\r\n", err);
	}
	printk("ble init success!\r\n");
	ble_had_been_inited = 1;

	/* 使能扫描 */
	bt_scan_enable();

	while (1){
		printk("hello world\r\n");
		NRFX_DELAY_US(2000000);
	}
}
