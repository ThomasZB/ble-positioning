#ifndef __MYCONN_H
#define __MYCONN_H

#include <bluetooth/bluetooth.h>


void connected(struct bt_conn *conn, uint8_t err);
void disconnected(struct bt_conn *conn, uint8_t reason);

#endif