#ifndef __MYBLUETOOTH_H
#define __MYBLUETOOTH_H

#include <drivers/uart.h>
#include <bluetooth/bluetooth.h>

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

extern uint8_t ble_had_been_inited;

void bt_advertising_start(void);

#endif