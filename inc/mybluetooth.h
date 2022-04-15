#ifndef __MYBLUETOOTH_H
#define __MYBLUETOOTH_H

#include <bluetooth/bluetooth.h>

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

void bt_advertising_start(void);

#endif