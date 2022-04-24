/**
 * @file uart_client.c
 * @author hang chen (thomaszb.cn)
 * @brief 用于解析串口透传服务
 * @version 0.1
 * @date 2022-04-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "uart_client.h"
#include "myuart.h"
#include "bluetooth/services/nus.h"

bt_uart_client bt_uart_client_handle;

int get_uart_service(struct bt_gatt_dm *dm){
    const struct bt_gatt_dm_attr *gatt_service_attr =
			bt_gatt_dm_service_get(dm);
	const struct bt_gatt_service_val *gatt_service =
			bt_gatt_dm_attr_service_val(gatt_service_attr);
	const struct bt_gatt_dm_attr *gatt_chrc;
	const struct bt_gatt_dm_attr *gatt_desc;

    /* 解析服务UUID */
    printk("finded service!\r\n");
    printk("uuid_type: %d\r\n", gatt_service->uuid->type);
    if (bt_uuid_cmp(gatt_service->uuid, BT_UUID_NUS_SERVICE)) {
		return -ENOTSUP;
	}
    printk("Getting handles from NUS service.!\r\n");

	/* 解析TX特征 */
	gatt_chrc = bt_gatt_dm_char_by_uuid(dm, BT_UUID_NUS_TX);
	if (!gatt_chrc) {
		printk("Missing NUS TX characteristic.\r\n");
		return -EINVAL;
	}
	gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, BT_UUID_NUS_TX);
	if (!gatt_desc) {
		printk("Missing NUS TX value descriptor in characteristic.\r\n");
		return -EINVAL;
	}
	printk("Found handle for NUS TX characteristic.\r\n");
    bt_uart_client_handle.tx_handle = gatt_desc->handle;

	/* 解析RX特征 */
	gatt_chrc = bt_gatt_dm_char_by_uuid(dm, BT_UUID_NUS_RX);
	if (!gatt_chrc) {
		printk("Missing NUS RX characteristic.\r\n");
		return -EINVAL;
	}
	gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, BT_UUID_NUS_RX);
	if (!gatt_desc) {
		printk("Missing NUS RX value descriptor in characteristic.\r\n");
		return -EINVAL;
	}
	printk("Found handle for NUS RX characteristic.\r\n");
    bt_uart_client_handle.rx_handle = gatt_desc->handle;

    bt_uart_client_handle.conn = bt_gatt_dm_conn_get(dm);
    return 0;
}



int bt_uart_subscribe_receive(void)
{
	int err;

	if (atomic_test_and_set_bit(&bt_uart_client_handle.state, BT_UART_SEND_STATUS_ENABLED)){
		return -EALREADY;
	}

	bt_uart_client_handle.tx_notif_params.notify = uart_tx_notify_cb;             /* 对方发notify的回调函数 */
	bt_uart_client_handle.tx_notif_params.value = BT_GATT_CCC_NOTIFY;
	bt_uart_client_handle.tx_notif_params.value_handle = bt_uart_client_handle.tx_handle;
	atomic_set_bit(bt_uart_client_handle.tx_notif_params.flags, 
                    BT_GATT_SUBSCRIBE_FLAG_VOLATILE);

    /* 订阅属性通知 */
	err = bt_gatt_subscribe(bt_uart_client_handle.conn, &bt_uart_client_handle.tx_notif_params);
	if (err) {
		printk("Subscribe failed (err %d)\r\n", err);
		atomic_clear_bit(&bt_uart_client_handle.state, BT_UART_TX_NOTIF_ENABLED);
	} else {
		printk("[SUBSCRIBED]\r\n");
	}

	return err;
}


uint8_t uart_tx_notify_cb(struct bt_conn *conn,
			struct bt_gatt_subscribe_params *params,
			const void *data, uint16_t length){
	if (!data) {
		printk("[UNSUBSCRIBED]\r\n");
		params->value_handle = 0;
		atomic_clear_bit(&bt_uart_client_handle.state, BT_UART_TX_NOTIF_ENABLED);
		return BT_GATT_ITER_STOP;
	}

	printk("[NOTIFICATION] data %p length %u\r\n", data, length);

	return BT_GATT_ITER_CONTINUE;
}