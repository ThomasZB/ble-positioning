#ifndef __MYBLUETOOTH_H
#define __MYBLUETOOTH_H

#include "drivers/uart.h"
#include "bluetooth/bluetooth.h"
#include "bluetooth/hci.h"
#include "bluetooth/scan.h"

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

extern uint8_t ble_had_been_inited;


int bt_scan_enable(void);
void connected_cb(struct bt_conn *conn, uint8_t err);
void disconnected_cb(struct bt_conn *conn, uint8_t reason);
void scan_filter_match_cb(struct bt_scan_device_info*, struct bt_scan_filter_match*, bool);


#endif