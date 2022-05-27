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


/* 串口透传服务UUID */
#define UART_TP_UUID_VAL BT_UUID_128_ENCODE(0x4fafc201,0x1fb5, 0x459e, 0x8fcc, 0xc5c9c331914b)

/* 发送特征UUID */
#define UART_TX_UUID_VAL BT_UUID_128_ENCODE(0x92d84e59,0x057e, 0x43a4, 0xb050, 0xb90b54293f50)

/* 接收特征UUID */
#define UART_RX_UUID_VAL BT_UUID_128_ENCODE(0xf9aacb93,0x0c3a, 0x49e3, 0x91df, 0xa527f45cb27c)

#define BT_UUID_UART_TP        BT_UUID_DECLARE_128(UART_TP_UUID_VAL)
#define BT_UUID_UART_RX        BT_UUID_DECLARE_128(UART_TX_UUID_VAL)
#define BT_UUID_UART_TX        BT_UUID_DECLARE_128(UART_RX_UUID_VAL)


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
    uint16_t tx_ccc;
    /* 发送特征描述 */
	struct bt_gatt_subscribe_params tx_notif_params; 
    /* 接收特征描述 */
    struct bt_gatt_write_params rx_write_params;
}bt_uart_client;


int bt_uart_subscribe_receive(void);
int get_uart_service(struct bt_gatt_dm *dm);
int bt_uart_client_send(const uint8_t *data, uint16_t len);


void bt_uart_client_send_cb(struct bt_conn *, uint8_t err, struct bt_gatt_write_params *);
uint8_t uart_tx_notify_cb(struct bt_conn *, struct bt_gatt_subscribe_params *, const void *, uint16_t);


#endif