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

int scan_init(void);
int bt_scan_enable(void);
int bt_scan_disable(void);
const char *phy2str(uint8_t phy);
bool data_cb(struct bt_data *data, void *user_data);
void connected_cb(struct bt_conn *conn, uint8_t err);
void scan_connecting_error_cb(struct bt_scan_device_info*);
void disconnected_cb(struct bt_conn *conn, uint8_t reason);
void scan_connecting_cb(struct bt_scan_device_info*, struct bt_conn*);
void scan_filter_match_cb(struct bt_scan_device_info*, struct bt_scan_filter_match *, bool);


/**
 * @brief 将广播窗转化为ms
 *
 * @param interval 	：广播窗大小，单位0.625ms
 * @return uint32_t ：广播窗大小，单位ms
 */
static inline uint32_t adv_interval_to_ms(uint16_t interval)
{
	return interval * 5 / 4;
}


#endif