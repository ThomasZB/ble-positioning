#ifndef __UART_PROFILE_H
#define __UART_PROFILE_H

#include <drivers/uart.h>
#include "bluetooth/uuid.h"
#include "bluetooth/gatt.h"


/* 串口透传服务UUID */
#define UART_TP_UUID_VAL BT_UUID_128_ENCODE(0x64f499cc,0x3411, 0x461d, 0x8055, 0x333d0bc7d33c)

/* 发送特征UUID */
#define UART_TX_UUID_VAL BT_UUID_128_ENCODE(0x64f499cd,0x3411, 0x461d, 0x8055, 0x333d0bc7d33c)

/* 接收特征UUID */
#define UART_RX_UUID_VAL BT_UUID_128_ENCODE(0x64f499ce,0x3411, 0x461d, 0x8055, 0x333d0bc7d33c)

#define BT_UUID_UART_TP        BT_UUID_DECLARE_128(UART_TP_UUID_VAL)
#define BT_UUID_UART_RX        BT_UUID_DECLARE_128(UART_TX_UUID_VAL)
#define BT_UUID_UART_TX        BT_UUID_DECLARE_128(UART_RX_UUID_VAL)


#define BT_UART_STACKSIZE 256
#define BT_UART_PRIORITY 7

extern struct k_sem ble_init_ok;

/* 串口透传状态 */
enum bt_uart_send_status {
	/* 发送通知使能 */
	BT_UART_SEND_STATUS_ENABLED,
	/* 发送通知停止使能 */
	BT_UART_SEND_STATUS_DISABLED,
};


void bt_uart_ccc_cfg_changed(const struct bt_gatt_attr *, uint16_t);
ssize_t ble_uart_receive_cb(struct bt_conn*, const struct bt_gatt_attr*, const void*, uint16_t, uint16_t, uint8_t);


#endif