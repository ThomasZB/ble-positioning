#include "mybluetooth.h"
#include "uart_profile.h"

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
