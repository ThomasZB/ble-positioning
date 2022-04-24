/**
 * @file mybluetooth.c
 * @author hang chen (thomaszb.cn)
 * @brief 
 * @version 0.1
 * @date 2022-04-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "direction_finding.h"
#include "mybluetooth.h"
#include "myuart.h"
#include "uart_client.h"
#include "bluetooth/services/nus.h"


/* 蓝牙初始化完成标志 */
bool scan_enabled = 0;
uint8_t ble_had_been_inited = 0;

/* 连接句柄 */
struct bt_conn *current_conn = NULL;

/* 连接相关回调函数注册 */
BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected_cb,
	.disconnected = disconnected_cb,
};

/* 扫描相关回调函数初始化，注册在函数里面 */
BT_SCAN_CB_INIT(scan_callbacks, scan_filter_match_cb, NULL,
		scan_connecting_error_cb, scan_connecting_cb);

/* 服务发现回调函数 */
const struct bt_gatt_dm_cb discovery_cb = {
	.completed         = discovery_complete_cb,
	.service_not_found = discovery_service_not_found_cb,
	.error_found       = discovery_error_cb,
};

/* 扫描参数 */
const struct bt_le_scan_param my_scan_param = {
	.type       = BT_HCI_LE_SCAN_ACTIVE,
	.options    = BT_LE_SCAN_OPT_NONE,
	.interval   = 0x00A0,
	.window     = 0x0050,
};
struct bt_scan_init_param my_scan_init_param = {
		.scan_param = &my_scan_param,
		.connect_if_match = 1,
};


/**
 * @brief 初始化扫描，主要是注册回调函数和过滤器初始化
 * 
 * @return int	：错误类型 
 */
int scan_init(void){
	int err;

	/* 初始化扫描参数，回调函数 */
	bt_scan_init(&my_scan_init_param);
	bt_scan_cb_register(&scan_callbacks);

	/* 添加过滤器 */
	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_NAME, "ble_thomas");
	if (err) {
		printk("Scanning filters cannot be set (err %d)\r\n", err);
		return err;
	}
	
	/* 使能过滤器 */
	err = bt_scan_filter_enable(BT_SCAN_NAME_FILTER, true);
	if (err) {
		printk("Filters cannot be turned on (err %d)\r\n", err);
		return err;
	}

	printk("Scan module initialized\r\n");
	return 0;
}


/**
 * @brief 如果没有开始扫描，则启动扫描
 * 	
 * @return int	：错误类型 
 */
int bt_scan_enable(void){
	int err;
	
	if (!scan_enabled) {
		printk("Start scanning...\r\n");
		/* 开始扫描 */
		err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
		if (err) {
			printk("Starting scanning failed (err %d)\r\n", err);
			return err;
		}
		scan_enabled = true;
		printk("Scanning successfully started!\r\n");
	}
	return 0;
}


/**
 * @brief 停止扫描
 * 
 * @return int	：错误类型 
 */
int bt_scan_disable(void){
	int err;

	err = bt_scan_stop();
	if ((!err) && (err != -EALREADY)) {
		printk("failed (err %d)\r\n", err);
		return err;
	}
	printk("Scan disabled.\r\n");
	
	scan_enabled = false;
	return 0;
}


/**
 * @brief 开始寻找服务
 * 
 * @param conn 	：连接句柄
 */
void gatt_service_discover(struct bt_conn *conn)
{
	int err;

	if (conn != current_conn) {
		return;
	}

	err = bt_gatt_dm_start(conn, BT_UUID_NUS_SERVICE, &discovery_cb, NULL);

	if (err) {
		printk("could not start the discovery procedure, error "
			"code: %d\r\n", err);
	}
}


/**
 * @brief 连接成功回调函数，在该函数中进行连接参数的更行
 * 
 * @param conn 	:连接句柄
 * @param err 	:连接错误
 */
void connected_cb(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err %u)\r\n", err);
		return;
	}
	printk("Connected!!\r\n");

	/**************** 初始化连接参数 ****************/
	static struct bt_gatt_exchange_params exchange_params;
	/* mtu交换 */
	exchange_params.func = mtu_exchange_cb;
	err = bt_gatt_exchange_mtu(conn, &exchange_params);
	if (err) {
		printk("MTU exchange failed (err %d)\r\n", err);
	}
	printk("MTU exchange succeed\r\n");
	
	gatt_service_discover(conn);

	/* 关闭自动连接 */
	bt_scan_disable();
	if (err){
		printk("Stop failed (%d) \r\n", err);
	}
}


/**
 * @brief mtu交换的回调函数
 * 
 * @param conn 		：连接句柄
 * @param err 		：交换结果
 * @param params 	：交换的回调函数
 */
void mtu_exchange_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
	if (!err) {
		printk("MTU exchange done\r\n");
	} else {
		printk("MTU exchange failed (err %" PRIu8 ")\r\n", err);
	}
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
	bt_conn_unref(current_conn);
	current_conn = NULL;
}


/**
 * @brief 匹配数据的回调函数，bt_data_parse函数将数据分块，回调函数解析这块数据
 * 
 * @param data 			：传入数据
 * @param user_data 	：解析完毕保存数据
 * @return true 		：成功
 * @return false 		：失败
 */
bool data_cb(struct bt_data *data, void *user_data)
{
	char *name = user_data;
	uint8_t len;

	switch (data->type) {
	case BT_DATA_NAME_SHORTENED:
	case BT_DATA_NAME_COMPLETE:
		len = MIN(data->data_len, PEER_NAME_LEN_MAX - 1);
		memcpy(name, data->data, len);
		name[len] = '\0';
		return false;
	default:
		return true;
	}
}


/**
 * @brief ble协议显示
 * 
 * @param phy 			：传入参数，phy标志
 * @return const char* 	：返回名称
 */
const char *phy2str(uint8_t phy)
{
	switch (phy) {
	case 0: return "No packets";
	case BT_GAP_LE_PHY_1M: return "LE 1M";
	case BT_GAP_LE_PHY_2M: return "LE 2M";
	case BT_GAP_LE_PHY_CODED: return "LE Coded";
	default: return "Unknown";
	}
}


/**
 * @brief 扫描回调函数中如果匹配成功的回调函数（弱定义，此应用中将在direction_finding.c里面重定义）
 * 
 * @param device_info 
 * @param filter_match 
 * @param connectable 
 */
__weak void scan_filter_match_cb(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match,
			      bool connectable)
{
	char le_addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(device_info->recv_info->addr, le_addr, sizeof(le_addr));

	printk("[DEVICE]: %s, RSSI %i, connectable: %d \r\n", le_addr, device_info->recv_info->rssi, connectable);
}


/**
 * @brief 连接失败回调函数
 * 
 * @param device_info 
 */
void scan_connecting_error_cb(struct bt_scan_device_info *device_info)
{
	printk("Connecting failed\r\n");
}


/**
 * @brief 连接成功回调函数
 * 
 * @param device_info 
 */
void scan_connecting_cb(struct bt_scan_device_info *device_info,
			    struct bt_conn *conn)
{
	current_conn = bt_conn_ref(conn);
}


/**
 * @brief 切换为连接模式，找到对应设备并建立连接
 * 
 */
void bt_switch_conn(void){
	/* 打开自动连接 */
	printk("switch to connected mode\r\n");
	bt_scan_conn_modify(true);
}


/**
 * @brief 切换为扫描CTE模式，不进行连接
 * 
 * @return int ：切换是否成功
 */
void bt_switch_df(void){
	/* 关闭自动连接 */
	printk("switch to direction finding mode... \r\n");
	bt_scan_conn_modify(false);
}


/**
 * @brief 服务发现成功回到函数
 * 
 * @param dm 		：服务发现句柄
 * @param context 	：传入参数
 */
void discovery_complete_cb(struct bt_gatt_dm *dm,
			       void *context)
{
	printk("Service discovery completed\r\n");
	
	get_uart_service(dm);
    bt_uart_subscribe_receive();

    bt_gatt_dm_data_release(dm);
}


/**
 * @brief 服务发现未找到服务回调函数
 * 
 * @param conn 		：连接句柄
 * @param context 	：传入参数
 */
void discovery_service_not_found_cb(struct bt_conn *conn,
					void *context)
{
	printk("Service not found\r\n");
}


/**
 * @brief 服务发现错误回调函数
 * 
 * @param conn 		：连接句柄
 * @param err 		：出错类型
 * @param context 	：传入参数
 */
void discovery_error_cb(struct bt_conn *conn,
			    int err,
			    void *context)
{
	printk("Error while discovering GATT database: (%d)\r\n", err);
}
