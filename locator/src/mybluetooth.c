#include "mybluetooth.h"
#include "uart_profile.h"
#include "myuart.h"


/* 蓝牙初始化完成标志 */
uint8_t ble_had_been_inited = 0;

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

/* 连接相关回调函数注册 */
BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected_cb,
	.disconnected = disconnected_cb,
};

/* 扫描相关回调函数初始化，注册在函数里面 */
BT_SCAN_CB_INIT(scan_cb, scan_filter_match_cb, NULL,
		NULL, NULL);

/* 扫描参数 */
const struct bt_le_scan_param my_scan_param = {
	.type       = BT_HCI_LE_SCAN_PASSIVE,
	.options    = BT_LE_SCAN_OPT_NONE,
	.interval   = 0x00A0,
	.window     = 0x0050,
};



/**
 * @brief 初始化并开始扫描 
 * 
 * @return int :错误类型
 */
int bt_scan_enable(void){
	int err;
	/* 参数定义在函数内以节省空间 */
	struct bt_scan_init_param scan_param_init = {
		.connect_if_match = 0,
	};
	scan_param_init.scan_param = &my_scan_param;

	/* 初始化扫描参数 */
	bt_scan_init(&scan_param_init);
	bt_scan_cb_register(&scan_cb);

	/* 添加并启动过滤器 */
	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_NAME, "ble_thomas");
	if (err) {
		printk("Scanning filters cannot be set (err %d)", err);
		return err;
	}
	err = bt_scan_filter_enable(BT_SCAN_NAME_FILTER, true);
	if (err) {
		printk("Filters cannot be turned on (err %d)", err);
		return err;
	}

	/* 开始扫描 */
	bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
	if (err) {
		printk("Starting scanning failed (err %d)\n", err);
		return err;
	}
	return 0;
}


/**
 * @brief 连接成功回调函数
 * 
 * @param conn 	:连接句柄
 * @param err 	:连接错误
 */
void connected_cb(struct bt_conn *conn, uint8_t err)
{

	if (err) {
		printk("Connection failed (err %u)", err);
		return;
	}

	printk("Connected!!\r\n");

	current_conn = bt_conn_ref(conn);
}


/**
 * @brief 断开连接回调函数
 * 
 * @param conn 		:连接句柄
 * @param reason 	:断开原因
 */
void disconnected_cb(struct bt_conn *conn, uint8_t reason)
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

/**
 * @brief 扫描匹配成功的回调函数
 * 
 * @param device_info 	: 设备信息
 * @param filter_match 	：匹配类型
 * @param connectable 	：是否可以连接
 */
void scan_filter_match_cb(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match,
			      bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

	printk("Filters matched. Address: %s connectable: %d", addr, connectable);
}
