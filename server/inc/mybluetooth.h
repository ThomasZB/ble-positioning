#ifndef __MYBLUETOOTH_H
#define __MYBLUETOOTH_H

#include <drivers/uart.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

extern uint8_t ble_had_been_inited;
extern struct bt_conn *current_conn;

void bt_scan_enable(void);
void bt_advertising_start(void);
void connected_cb(struct bt_conn *conn, uint8_t err);
void disconnected_cb(struct bt_conn *conn, uint8_t reason);
void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type, struct net_buf_simple *buf);


#endif