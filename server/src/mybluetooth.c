#include "mybluetooth.h"
#include "uart_profile.h"


/* 蓝牙初始化完成标志 */
uint8_t ble_had_been_inited = 0;


/* 广播发送的信息 */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
}; 

/* 对方扫描回应的信息 */
static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, UART_TP_UUID_VAL),
};

/* 扫描参数 */
const struct bt_le_scan_param scan_param = {
		.type       = BT_HCI_LE_SCAN_PASSIVE,
		.options    = BT_LE_SCAN_OPT_NONE,
		.interval   = 0x00A0,
		.window     = 0x0050,
	};


/**
 * @brief 蓝牙开始广播 
 */
void bt_advertising_start(void){
	int err;
	/* Start advertising */
	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
    if (err) {
		printk("Advertising failed to start (err %d)\r\n", err);
		return;
	}
	printk("Advertising started!\r\n");
}


/**
 * @brief 初始化并开始扫描 
 */
void bt_scan_enable(void){
	int err;

	err = bt_le_scan_start(&scan_param, scan_cb);
	if (err) {
		printk("Starting scanning failed (err %d)\n", err);
		return;
	}
}

/**
 * @brief 扫描回调函数，每次扫描到东西都会进入到里面
 * 
 * @param addr 		：广播方地址
 * @param rssi 		：接收功率
 * @param adv_type 	：类型，被动扫描还是主动扫描（可以用来判断是广播数据还是扫描响应数据）
 * @param buf 		：接收的广播数据
 */
static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
		    struct net_buf_simple *buf)
{
	static uint8_t name_buf[11] = {0};
	if (adv_type == BT_HCI_LE_SCAN_PASSIVE){
		if (buf->len > 10){
			if ((uint8_t)buf->data[3] == 11){
				memcpy(name_buf, &(buf->data[5]), 10);
				printk("name is: %s \r\n", name_buf);
				printk("rssi is: %d \r\n", rssi);
			}
		}
	}
}
