/**
 * @file mybluetooth.h
 * @author hang chen (thomaszb.cn)
 * @brief 
 * @version 0.1
 * @date 2022-04-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __MYBLUETOOTH_H
#define __MYBLUETOOTH_H

#include "drivers/uart.h"
#include "bluetooth/bluetooth.h"
#include "bluetooth/hci.h"
#include "bluetooth/scan.h"
#include "bluetooth/conn.h"
#include <bluetooth/gatt_dm.h>
#include "uart_profile.h"

#define PEER_NAME_LEN_MAX 30
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)


extern bool scan_enabled;
extern uint8_t ble_had_been_inited;
extern struct bt_conn *current_conn;

/* 外部调用函数 */
int scan_init(void);
void bt_switch_df(void);
int bt_scan_enable(void);
int bt_scan_disable(void);
void bt_switch_conn(void);
const char *phy2str(uint8_t phy);
void gatt_service_discover(struct bt_conn *conn);

/* 内部调用（回调函数） */
bool data_cb(struct bt_data *data, void *user_data);
void connected_cb(struct bt_conn *conn, uint8_t err);
void discovery_error_cb(struct bt_conn*, int, void*);
void discovery_complete_cb(struct bt_gatt_dm*, void*);
void scan_connecting_error_cb(struct bt_scan_device_info*);
void disconnected_cb(struct bt_conn *conn, uint8_t reason);
void discovery_service_not_found_cb(struct bt_conn*, void*);
void scan_connecting_cb(struct bt_scan_device_info*, struct bt_conn*);
void mtu_exchange_cb(struct bt_conn*, uint8_t, struct bt_gatt_exchange_params*);
void scan_filter_match_cb(struct bt_scan_device_info*, struct bt_scan_filter_match *, bool);


/**
 * @brief 将广播窗转化为ms
 *
 * @param interval 	：广播窗大小，单位0.625ms
 * @return uint32_t ：广播窗大小，单位ms
 */
static inline uint32_t adv_interval_to_ms(uint16_t interval)
{
	return (uint32_t)(interval*1.25);
}


#endif