#include "myconn.h"
#include "myuart.h"
#include <bluetooth/conn.h>

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
};


void connected(struct bt_conn *conn, uint8_t err)
{

	if (err) {
		printk("Connection failed (err %u)", err);
		return;
	}

	printk("Connected!!\r\n");

	current_conn = bt_conn_ref(conn);
}


void disconnected(struct bt_conn *conn, uint8_t reason)
{

	printk("Disconnected (reason %u)\r\n", reason);

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
}