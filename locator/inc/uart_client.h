/**
 * @file uart_client.h
 * @author hang chen (thomaszb.cn)
 * @brief 
 * @version 0.1
 * @date 2022-04-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __UART_CLIENT_H
#define __UART_CLIENT_H

#include "mybluetooth.h"

/* 蓝牙串口标志 */
enum bt_uart_send_status {
	/* 发送通知已启用 */
	BT_UART_SEND_STATUS_ENABLED,
	/* 发送通知未启用 */
	BT_UART_SEND_STATUS_DISABLED,

    BT_UART_INITIALIZED,
	BT_UART_TX_NOTIF_ENABLED,
	BT_UART_RX_WRITE_PENDING
};

typedef struct{
    /* 连接对象 */
    struct bt_conn *conn;
    /* 内部状态 */
    atomic_t state;

    uint16_t tx_handle;
    uint16_t rx_handle;
    /* 发送特征描述 */
	struct bt_gatt_subscribe_params tx_notif_params; 
    /* 接收特征描述 */
    struct bt_gatt_write_params rx_write_params;
}bt_uart_client;


int bt_uart_subscribe_receive(void);
int get_uart_service(struct bt_gatt_dm *dm);


uint8_t uart_tx_notify_cb(struct bt_conn *, struct bt_gatt_subscribe_params *, const void *, uint16_t);
#endif