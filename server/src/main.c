/****
 * 
 * 
 * */
#include <zephyr.h>
#include "mybluetooth.h"
#include "myuart.h"
#include "uart_profile.h"


K_THREAD_DEFINE(ble_write_thread_id, BT_UART_STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, BT_UART_PRIORITY, 0, 0);

void main(void)
{
	int err;
	
	NRFX_DELAY_US(1000000);
	uart_init();

	err = bt_enable(NULL);
	if (err){
		printk("ble init faild\r\n");
	}
	/* 发送初始化成功的消息 */
	k_sem_give(&ble_init_ok);
	printk("ble init success!\r\n");
	ble_had_been_inited = 1;

	/* 开始广播 */
	bt_advertising_start();
	while (1){
		printk("hello world\r\n");
		k_sleep(K_MSEC(2000));
	}
}


