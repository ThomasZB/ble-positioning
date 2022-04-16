/****
 * 
 * 
 * */
#include <zephyr.h>
#include "mybluetooth.h"
#include "myuart.h"


void main(void)
{
	int err;
	
	NRFX_DELAY_US(1000000);
	uart_init();

	err = bt_enable(NULL);
	if (err){
		printk("ble init faild\r\n");
	}
	printk("ble init success!\r\n");
	
	bt_advertising_start();
	
	while (1){
		printk("hello world\r\n");
		NRFX_DELAY_US(2000000);
	}
}
