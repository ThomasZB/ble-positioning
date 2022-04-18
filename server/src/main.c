/****
 * 
 * 
 * */
#include <zephyr.h>
#include "mybluetooth.h"
#include "myuart.h"
#include "uart_profile.h"

static K_SEM_DEFINE(ble_init_ok, 0, 1);


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
		NRFX_DELAY_US(2000000);
	}
}
