#include "uart_profile.h"
#include "myuart.h"
#include "mybluetooth.h"

/* UART Service Declaration */
BT_GATT_SERVICE_DEFINE(us_svc,
BT_GATT_PRIMARY_SERVICE(BT_UUID_UART_TP),
	BT_GATT_CHARACTERISTIC(BT_UUID_UART_TX,
			       BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       NULL, NULL, NULL),
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
    //printk("Received data, handle %d, conn %p", attr->handle, (void *)conn);
    if (ble_had_been_inited == 1){
        uart_tx(uart, buf, sizeof(buf), SYS_FOREVER_MS);
    }
    return len;
}

