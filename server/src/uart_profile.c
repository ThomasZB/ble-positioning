#include "uart_profile.h"
#include "myuart.h"
#include "mybluetooth.h"


K_FIFO_DEFINE(fifo_uart_rx_data);
K_SEM_DEFINE(ble_init_ok, 0, 1);

/* UART Service Declaration */
BT_GATT_SERVICE_DEFINE(us_svc,
BT_GATT_PRIMARY_SERVICE(BT_UUID_UART_TP),
	BT_GATT_CHARACTERISTIC(BT_UUID_UART_TX,
			       BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       NULL, NULL, NULL),
    BT_GATT_CCC(bt_uart_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),   /* 只要发送有通知都需要配合这个使用 */
	BT_GATT_CHARACTERISTIC(BT_UUID_UART_RX,
			       BT_GATT_CHRC_WRITE |
			       BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       NULL, ble_uart_receive_cb, NULL),
);



ssize_t ble_uart_receive_cb(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  const void *buf,
			  uint16_t len,
			  uint16_t offset,
			  uint8_t flags){

    if (ble_had_been_inited == 1){
        uart_tx(uart, buf, len, SYS_FOREVER_MS);
    }
    return len;
}


void bt_uart_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				  uint16_t value){
    printk("notification has been changed\r\n");

}


int bt_uart_send(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
	struct bt_gatt_notify_params params = {0};
	const struct bt_gatt_attr *attr = &us_svc.attrs[2];

	params.attr = attr;
	params.data = data;
	params.len = len;
	params.func = NULL;

	if (!conn) {
		//printk("Notification send to all connected peers\r\n");
		return bt_gatt_notify_cb(NULL, &params);
	} else if (bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)) {
		return bt_gatt_notify_cb(conn, &params);
	} else {
		return -EINVAL;
	}
}


void ble_write_thread(void)
{
	/* Don't go any further until BLE is initialized */
	k_sem_take(&ble_init_ok, K_FOREVER);
    //printk("write thread started!\r\n");
	for (;;) {
		/* Wait indefinitely for data to be sent over bluetooth */
		struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data,
						     K_FOREVER);
        //printk("data len: %d \r\n", buf->len);

		if (bt_uart_send(current_conn, buf->data, buf->len)) {
			//printk("Failed to send data over BLE connection\r\n");
		}

		k_free(buf);
	}
}





/**
 * @breif: 串口回调函数，目前没用
 * @param
 */
void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data){
	static bool buf_release;
	struct uart_data_t *buf;

	switch (evt->type) {
	case UART_RX_RDY:
		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data);
		buf->len = (uint16_t)evt->data.rx.len;
		buf_release = false;
//        printk("buf_len:%d\r\n", buf->len);

        k_fifo_put(&fifo_uart_rx_data, buf);
        buf_release = true;
//        uart_rx_disable(uart);
		// if (buf->len == UART_BUF_SIZE) {
		// 	k_fifo_put(&fifo_uart_rx_data, buf);
		// } else if ((evt->data.rx.buf[buf->len - 1] == '\n') ||
		// 	  (evt->data.rx.buf[buf->len - 1] == '\r')) {
		// 	k_fifo_put(&fifo_uart_rx_data, buf);
		// 	current_buf = evt->data.rx.buf;
		// 	buf_release = true;
		// 	uart_rx_disable(uart);
		// }
		break;
	default:
		break;
	}
}