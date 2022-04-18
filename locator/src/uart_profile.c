#include "uart_profile.h"
#include "myuart.h"
#include "mybluetooth.h"
#include "bluetooth/conn.h"


/* 串口透传服务注册 */
// BT_GATT_SERVICE_DEFINE(us_svc,
// BT_GATT_PRIMARY_SERVICE(BT_UUID_UART_TP),
// 	BT_GATT_CHARACTERISTIC(BT_UUID_UART_TX,
// 			       BT_GATT_CHRC_NOTIFY,
// 			       BT_GATT_PERM_READ,
// 			       NULL, NULL, NULL),
// 	BT_GATT_CHARACTERISTIC(BT_UUID_UART_RX,
// 			       BT_GATT_CHRC_WRITE |
// 			       BT_GATT_CHRC_WRITE_WITHOUT_RESP,
// 			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
// 			       NULL, ble_uart_receive_cb, NULL),
// );


/**
 * @brief 串口透传接收服务回调函数，当接收到数据时会自动调用该函数（目前还有很大的问题）
 * 
 * @param conn 		：具体的连接
 * @param attr 		：对方地址
 * @param buf 		：数据
 * @param len 		：数据长度
 * @param offset 	：
 * @param flags 	：
 * @return ssize_t 	：
 */
ssize_t ble_uart_receive_cb(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  const void *buf,
			  uint16_t len,
			  uint16_t offset,
			  uint8_t flags){
    if (ble_had_been_inited == 1){
        uart_tx(uart, buf, sizeof(buf), SYS_FOREVER_MS);
    }
    return len;
}

