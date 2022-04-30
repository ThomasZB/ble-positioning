/**
 * @file direction_finding.c
 * @author hang chen (thomaszb.cn)
 * @brief AOD定位的源文件
 * @version 0.1
 * @date 2022-04-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "direction_finding.h"
#include "mybluetooth.h"
#include "bluetooth/hci.h"
#include "myuart.h"

/* 定义信号量 */
K_SEM_DEFINE(sem_per_adv, 0, 1);        /* 用于找设备（周期广播）的信号量 */
K_SEM_DEFINE(sem_per_sync, 0, 1);       /* 用于同步的信号量 */
K_SEM_DEFINE(sem_per_sync_lost, 0, 1);  /* 同步丢失的信号量 */

/* 是否找到周期广播的信号 */
volatile bool per_adv_found;
static struct bt_le_per_adv_sync *sync_handle;


static uint8_t per_sid;
static bt_addr_le_t per_addr;
uint32_t sync_create_timeout_ms;

void state_changed_cb(struct bt_le_per_adv_sync *sync,
			      const struct bt_le_per_adv_sync_state_info *info){
                      printk("this is state changed cb\r\n");
                  }
void biginfo_cb(struct bt_le_per_adv_sync *sync, const struct bt_iso_biginfo *biginfo){
    printk("this is biginfo cb\r\n");
}

 struct bt_le_per_adv_sync_cb sync_callbacks = {
	.synced = sync_cb,
	.term = term_cb,
	.recv = recv_cb,
	.cte_report_cb = cte_recv_cb,
    .state_changed = state_changed_cb,
    .biginfo = biginfo_cb,
    };

void direction_finding_init(void){
    /* 定义在函数内部以节省空间 */
   
   
    printk("Periodic Advertising callbacks register...\r\n");
	bt_le_per_adv_sync_cb_register(&sync_callbacks);
	printk("success.\r\n");
}



/**
 * @brief 等待广播消息
 * 
 * @return int ：0，成功；其他值失败
 */
int wait_central_adv(void){
    int err;
    printk("Waiting for periodic advertising...\r\n");
    err = k_sem_take(&sem_per_adv, K_FOREVER);
    if (err) {
        printk("failed (err %d)\r\n", err);
        return err;
    }
    printk("success. Found periodic advertising.\r\n");
    return 0;
}


/**
 * @brief 创建一个同步的句柄
 */
void create_sync_handle(void){
    static struct bt_le_per_adv_sync_param sync_create_param;
	int err;

	printk("Creating Periodic Advertising Sync...\r\n");
	bt_addr_le_copy(&sync_create_param.addr, &per_addr);
	sync_create_param.options = 0;
	sync_create_param.sid = per_sid;
	sync_create_param.skip = 0;
	sync_create_param.timeout = 0xa;
	err = bt_le_per_adv_sync_create(&sync_create_param, &sync_handle);
	if (err) {
		printk("failed (err %d)\r\n", err);
		return;
	}
	printk("success.\r\n");
}

int delete_sync(void)
{
	int err;

	printk("Deleting Periodic Advertising Sync...\r\n");
	err = bt_le_per_adv_sync_delete(sync_handle);
	if (err) {
		printk("failed (err %d)\r\n", err);
		return err;
	}
	printk("success\r\n");

	return 0;
}

/**
 * @brief 等待广播消息
 * 
 * @return int ：0，成功；其他值失败
 */
int wait_sync(void){
    int err;
    printk("Waiting for periodic sync...\r\n");
    err = k_sem_take(&sem_per_sync, K_MSEC(sync_create_timeout_ms));
    if (err) {
        printk("failed waitting (err %d)\r\n", err);
        err = delete_sync();
        return err;
    }
    printk("success. Periodic sync established.\r\n");
    return 0;
}


/**
 * @brief 等待同步丢失
 * 
 * @return int 0等待成功；其他，错误码
 */
int wait_sync_lost(void){
    int err;
    printk("Waiting for periodic sync lost...\r\n");
    err = k_sem_take(&sem_per_sync_lost, K_FOREVER);
    if (err) {
        printk("failed (err %d)\n", err);
        return err;
    }
    printk("Periodic sync lost.\n");
    return 0;
}


/**
 * @brief 使能CTE接收
 * 
 */
int enable_cte_rx(void){
    int err;

	static struct bt_df_per_adv_sync_cte_rx_param cte_rx_params = {
		.max_cte_count = 1,
		.cte_types = BT_DF_CTE_TYPE_AOD_2US,
        // .slot_durations = 0x02,
        // .ant_ids=NULL,
        // .num_ant_ids=1,
	};

	printk("Enable receiving of CTE...\r\n");
    
	err = bt_df_per_adv_sync_cte_rx_enable(sync_handle, &cte_rx_params);
	if (err) {
		printk("failed (err %d)\r\n", err);
		return err;
	}
	printk("success. CTE receive enabled.\r\n");
    return 0;
}


/**
 * @brief CTE信息包转化为人可见的
 * 
 * @param type          ：信息包类型定义
 * @return const char*  ：信息包返回值
 */
static const char *cte_type2str(uint8_t type)
{
	switch (type) {
	case BT_DF_CTE_TYPE_AOA: return "AOA";
	case BT_DF_CTE_TYPE_AOD_1US: return "AOD 1 [us]";
	case BT_DF_CTE_TYPE_AOD_2US: return "AOD 2 [us]";
	case BT_DF_CTE_TYPE_NONE: return "";
	default: return "Unknown";
	}
}


/**
 * @brief 
 * 
 * @param status 
 * @return const char* 
 */
static const char *packet_status2str(uint8_t status)
{
	switch (status) {
	case BT_DF_CTE_CRC_OK: return "CRC OK";
	case BT_DF_CTE_CRC_ERR_CTE_BASED_TIME: return "CRC not OK, CTE Info OK";
	case BT_DF_CTE_CRC_ERR_CTE_BASED_OTHER: return "CRC not OK, Sampled other way";
	case BT_DF_CTE_INSUFFICIENT_RESOURCES: return "No resources";
	default: return "Unknown";
	}
}


/**
 * @brief 没有匹配的回调函数，即不是连接对象，进行判断是否是周期广播信号（进行定位）
 * 
 * @param device_info 	：广播设备信息
 * @param connectable 	：是否可连接
 */
void scan_filter_not_match_cb(struct bt_scan_device_info *device_info,
				bool connectable)
{
	char le_addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(device_info->recv_info->addr, le_addr, sizeof(le_addr));

    /* 找到周期广播信号 */
    if (!per_adv_found && device_info->recv_info->interval) {
		sync_create_timeout_ms =
			adv_interval_to_ms(device_info->recv_info->interval) * SYNC_CREATE_TIMEOUT_INTERVAL_NUM;
		per_adv_found = true;
		per_sid = device_info->recv_info->sid;
		bt_addr_le_copy(&per_addr, device_info->recv_info->addr);
        /* 打印信息 */
	    printk("[DEVICE]: %s, RSSI %i, sync_create_timeout_ms: %d \r\n", le_addr, device_info->recv_info->rssi, (int)adv_interval_to_ms);
        /* 发送信号 */
		k_sem_give(&sem_per_adv);
	}
}


/**
 * @brief 同步成功的回调函数
 * 
 * @param sync ：同步的句柄？
 * @param info ：广播方的信息
 */
void sync_cb(struct bt_le_per_adv_sync *sync,
		    struct bt_le_per_adv_sync_synced_info *info)
{
	char le_addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

	printk("PER_ADV_SYNC[%u]: [DEVICE]: %s synced, "
	       "Interval 0x%04x (%u ms), PHY %s\r\n",
	       bt_le_per_adv_sync_get_index(sync), le_addr, info->interval,
	       adv_interval_to_ms(info->interval), phy2str(info->phy));
 
	k_sem_give(&sem_per_sync);
}


/**
 * @brief 同步停止的信号量
 * 
 * @param sync ：同步的句柄
 * @param info ：广播方信息
 */
void term_cb(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_term_info *info)
{
	char le_addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

	printk("PER_ADV_SYNC[%u]: [DEVICE]: %s sync terminated\r\n",
	       bt_le_per_adv_sync_get_index(sync), le_addr);

	k_sem_give(&sem_per_sync_lost);
}


/**
 * @brief 周期广播的接收回调函数
 * 
 * @param sync  ：同步的句柄
 * @param info  ：广播方的信息
 * @param buf   ：接收的数据
 */
void recv_cb(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_recv_info *info,
		    struct net_buf_simple *buf)
{
	static char data_str[MY_GAP_ADV_MAX_DATA_LEN];
	char le_addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
	bin2hex(buf->data, buf->len, data_str, sizeof(data_str));

	printk("PER_ADV_SYNC[%u]: [DEVICE]: %s, tx_power %i, "
	       "RSSI %i, CTE %s, data length %u, data: %s\r\n",
	       bt_le_per_adv_sync_get_index(sync), le_addr, info->tx_power,
	       info->rssi, cte_type2str(info->cte_type), buf->len, data_str);
}


/**
 * @brief CTE采样的回调函数
 * 
 * @param sync      ：同步句柄
 * @param report    ：CTE采样报告
 */
void cte_recv_cb(struct bt_le_per_adv_sync *sync,
			struct bt_df_per_adv_sync_iq_samples_report const *report)
{
	printk("CTE[%u]: samples count %d, cte type %s, slot durations: %u [us], "
	       "packet status %s, RSSI %i\r\n",
	       bt_le_per_adv_sync_get_index(sync), report->sample_count,
	       cte_type2str(report->cte_type), report->slot_durations,
	       packet_status2str(report->packet_status), report->rssi);
}



