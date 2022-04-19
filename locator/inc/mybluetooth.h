#ifndef __MYBLUETOOTH_H
#define __MYBLUETOOTH_H

#include "drivers/uart.h"
#include "bluetooth/bluetooth.h"
#include "bluetooth/hci.h"
#include "bluetooth/scan.h"

#define PEER_NAME_LEN_MAX 30
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

extern bool scan_enabled;
extern uint8_t ble_had_been_inited;


void scan_init(void);
int bt_scan_enable(void);
bool data_cb(struct bt_data *data, void *user_data);
void connected_cb(struct bt_conn *conn, uint8_t err);
void disconnected_cb(struct bt_conn *conn, uint8_t reason);
void scan_cb(const struct bt_le_scan_recv_info*, struct net_buf_simple*);
void scan_filter_match_cb(const struct bt_le_scan_recv_info*, struct net_buf_simple*);


#endif