#include "mybluetooth.h"
#include "uart_profile.h"
#include "myuart.h"


/* 蓝牙初始化完成标志 */
bool scan_enabled = 0;
uint8_t ble_had_been_inited = 0;


static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

/* 连接相关回调函数注册 */
BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected_cb,
	.disconnected = disconnected_cb,
};

/* 扫描相关回调函数初始化，注册在函数里面 */
static struct bt_le_scan_cb scan_callbacks = {
	.recv = scan_cb,
};

/* 扫描参数 */
const struct bt_le_scan_param my_scan_param = {
	.type       = BT_HCI_LE_SCAN_PASSIVE,
	.options    = BT_LE_SCAN_OPT_NONE,
	.interval   = 0x00A0,
	.window     = 0x0050,
};



/**
 * @brief 初始化扫描，主要是注册回调函数
 */
void scan_init(void){
	/* 初始化扫描参数 */
	printk("Scan callbacks register...\r\n");
	bt_le_scan_cb_register(&scan_callbacks);
	printk("success.\r\n");
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
		err = bt_le_scan_start(&my_scan_param, NULL);
		if (err) {
			printk("failed (err %d)\r\n", err);
			return err;
		}
		printk("success\r\n");
		scan_enabled = true;
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
		printk("Connection failed (err %u)\r\n", err);
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
 * @brief 扫描的回调函数，这里我用该函数做过滤器
 * 
 * @param info 	：广播设备信息（是否可连接、功率）
 * @param buf 	：广播数据
 */
void scan_cb(const struct bt_le_scan_recv_info *info,
		      struct net_buf_simple *buf)
{
	char name[PEER_NAME_LEN_MAX];
	(void)memset(name, 0, sizeof(name));

	/* 找到name数据 */
	bt_data_parse(buf, data_cb, name);

	/* 比较name */
	if (!strcmp(name, "ble_thomas")){
		scan_filter_match_cb(info, buf);
	}

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


static const char *phy2str(uint8_t phy)
{
	switch (phy) {
	case 0: return "No packets";
	case BT_GAP_LE_PHY_1M: return "LE 1M";
	case BT_GAP_LE_PHY_2M: return "LE 2M";
	case BT_GAP_LE_PHY_CODED: return "LE Coded";
	default: return "Unknown";
	}
}

static inline uint32_t adv_interval_to_ms(uint16_t interval)
{
	return interval * 5 / 4;
}

/**
 * @brief 扫描回调函数中如果匹配成功的回调函数
 * 
 * @param info 	：广播设备的信息
 * @param buf	：广播发出的数据
 */
void scan_filter_match_cb(const struct bt_le_scan_recv_info *info,
		      struct net_buf_simple *buf)
{
	char le_addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

	printk("[DEVICE]: %s, AD evt type %u, Tx Pwr: %i, RSSI %i C:%u S:%u "
	       "D:%u SR:%u E:%u Prim: %s, Secn: %s, Interval: 0x%04x (%u ms), "
	       "SID: %u\r\n",
	       le_addr, info->adv_type, info->tx_power, info->rssi,
	       (info->adv_props & BT_GAP_ADV_PROP_CONNECTABLE) != 0,
	       (info->adv_props & BT_GAP_ADV_PROP_SCANNABLE) != 0,
	       (info->adv_props & BT_GAP_ADV_PROP_DIRECTED) != 0,
	       (info->adv_props & BT_GAP_ADV_PROP_SCAN_RESPONSE) != 0,
	       (info->adv_props & BT_GAP_ADV_PROP_EXT_ADV) != 0, phy2str(info->primary_phy),
	       phy2str(info->secondary_phy), info->interval, adv_interval_to_ms(info->interval),
	       info->sid);
}


