/****
 * 
 * 
 * */
#include "zephyr.h"
#include "mybluetooth.h"
#include "myuart.h"
#include "uart_profile.h"
#include "direction_finding.h"


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
	scan_init();

	/* 使能direction finding */
	direction_finding_init();
	

	/* 开始AOD测量 */
	while (1){
		/* 找到对应的AOD广播设备 */
		bt_scan_enable();
		wait_central_adv();

		/* 进行同步 */
		create_sync_handle();
		wait_sync();

		/* 接收CTE信息 */
		enable_cte_rx();
		
		/* 停止一次采集，准备下次采集 */
		bt_scan_disable();
		wait_sync_lost();
	}
}
