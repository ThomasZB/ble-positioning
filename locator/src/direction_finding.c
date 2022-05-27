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
#include "math.h"
#include "stdio.h"
#include "arm_math.h"


/* 定义信号量 */
K_SEM_DEFINE(sem_per_adv, 0, 1);        /* 用于找设备（周期广播）的信号量 */
K_SEM_DEFINE(sem_per_sync, 0, 1);       /* 用于同步的信号量 */
K_SEM_DEFINE(sem_per_sync_lost, 0, 1);  /* 同步丢失的信号量 */
K_SEM_DEFINE(sem_rssi_ready, 0, 1);  /* 同步丢失的信号量 */
K_SEM_DEFINE(sem_aod_ready, 0, 1);  /* 同步丢失的信号量 */

/* 是否找到周期广播的信号 */
volatile bool per_adv_found;
static struct bt_le_per_adv_sync *sync_handle;

/* 用于信号同步 */
static uint8_t per_sid;
static bt_addr_le_t per_addr;
uint32_t sync_create_timeout_ms;

/* 全局变量 */
double pi = 3.1416;
int8_t rssi_data = 0;
float pitch = 0;
float yaw = 0;

/* CTE信号接收相关回调函数 */
 struct bt_le_per_adv_sync_cb sync_callbacks = {
	.synced = sync_cb,
	.term = term_cb,
	.recv = sync_recv_cb,
	.cte_report_cb = cte_recv_cb,
};




/**
 * @brief 初始化df
 * 
 */
void direction_finding_init(void){
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
	sync_create_param.options = BT_LE_PER_ADV_SYNC_OPT_SYNC_ONLY_CONST_TONE_EXT;
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


/**
 * @brief 删除同步相关句柄（同步出现错误时调用）
 * 
 * @return int 0，删除成功；其他，错误码
 */
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
 * @return int ：0，成功；其他，错误码
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
 * @return int 0，使能成功，其他，错误码
 */
int enable_cte_rx(void){
    int err;
	const struct bt_df_per_adv_sync_cte_rx_param cte_rx_params = {
		.max_cte_count = 5,
		.cte_types = BT_DF_CTE_TYPE_AOD_1US|BT_DF_CTE_TYPE_AOD_2US,
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
void sync_recv_cb(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_recv_info *info,
		    struct net_buf_simple *buf)
{
	static char data_str[MY_GAP_ADV_MAX_DATA_LEN];
	char le_addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
	bin2hex(buf->data, buf->len, data_str, sizeof(data_str));

	// printk("PER_ADV_SYNC[%u]: [DEVICE]: %s, tx_power %i, "
	//        "RSSI %i, CTE %s, data length %u, data: %s\r\n",
	//        bt_le_per_adv_sync_get_index(sync), le_addr, info->tx_power,
	//        info->rssi, cte_type2str(info->cte_type), buf->len, data_str);
    //printk("rssi:%d\n", info->rssi);
	printk("RSSI %i, CTE %s, data length %u, data: %s\r\n", info->rssi, cte_type2str(info->cte_type), buf->len, data_str);
    rssi_data = info->rssi;
    k_sem_give(&sem_rssi_ready);
}


float my_abs(float a){
    if (a<0){
        return -a;
    }
    return a;
}

#define PID2 (0.1885)
const float lambda[] = {0.124680,0.124576,0.124473,0.124369,0.124266,0.124163,0.124061,0.123958,0.123856,0.123753,0.123651,0.123448,0.123346,0.123245,0.123143,0.123042,0.122941,0.122841,0.122740,0.122640,0.122539,0.122439,0.122339,0.122240,0.122140,0.122040,0.121941,0.121842,0.121743,0.121644,0.121546,0.121447,0.121349,0.121251,0.121153,0.121055,0.120957,0.124784,0.123549,0.120860};
struct bt_df_per_adv_sync_iq_samples_report my_report;
/**
 * @brief CTE采样的回调函数
 * 
 * @param sync      ：同步句柄
 * @param report    ：CTE采样报告
 */
void cte_recv_cb(struct bt_le_per_adv_sync *sync,
			struct bt_df_per_adv_sync_iq_samples_report const *report)
{
    
	// printk("CTE[%u]: samples count %d, cte type %s, slot durations: %u [us], "
	//        "packet status %s, RSSI %i\r\n",
	//        bt_le_per_adv_sync_get_index(sync), report->sample_count,
	//        cte_type2str(report->cte_type), report->slot_durations,
	//        packet_status2str(report->packet_status), report->rssi);
    
	printk("CTE[%u]: samples count %d, cte type %s\r\n", bt_le_per_adv_sync_get_index(sync), report->sample_count, cte_type2str(report->cte_type));

    my_report = *report;
     k_sem_give(&sem_aod_ready);
}


/**
 * @brief rssi和角度信息发送到stm32f412的线程
 * 
 */
void ble2stm32_thread(void)
{
    uint8_t i;
    uint8_t index;
    static float sig_refer_angle[8] = {0};
    static float refer_t_num[8] = {-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5};
    static float sig_refer_angle_avg;
    static float delta_phi_1us;
    static float delta_phi_4us;
    static float ant1_angle[11] = {0};
    static float ant2_angle[11] = {0};
    static float ant3_angle[11] = {0};
    static float delta_angle_avg1 = 0;
    static float delta_angle_avg2 = 0;
    static float temp;
    static char send_data[100] = {0};

    
	for (;;) {
        /* 等待两个信息的信号量 */
        k_sem_take(&sem_rssi_ready, K_FOREVER);
        k_sem_take(&sem_aod_ready, K_FOREVER);

        if (my_report.sample_count == 45){
            /* 获取参考信号相位 */
            for (i=0; i<8; i++){
                sig_refer_angle[i] = atan2(my_report.sample[i].q, my_report.sample[i].i);
                if (sig_refer_angle[i] < 0){
                    sig_refer_angle[i] = sig_refer_angle[i] + 2*pi;
                }
                if (i>0){
                    while (sig_refer_angle[i] < sig_refer_angle[i-1]){
                        sig_refer_angle[i] = sig_refer_angle[i] + 2*pi;
                    }
                }
            }
            /* 线性回归计算斜率 */
            arm_mean_f32(sig_refer_angle, 8, &sig_refer_angle_avg);                     /* 计算均值 */
            arm_offset_f32(sig_refer_angle, -sig_refer_angle_avg, sig_refer_angle, 8);  /* 对信号偏移均值 */
            arm_dot_prod_f32(sig_refer_angle, refer_t_num, 8, &delta_phi_1us);          /* 点积坐标 */
            delta_phi_1us = delta_phi_1us/42 - pi;                                      /* 计算斜率的结果 */
            delta_phi_4us = fmod(delta_phi_1us*4, 2*pi);

            /* 提取天线1信号相位 */
            for (i=0; i<11; i++){
                index = 9+i*3;
                ant1_angle[i] = atan2(my_report.sample[index].q, my_report.sample[index].i);
                if (ant1_angle[i] < 0){
                    ant1_angle[i] = ant1_angle[i] + 2*pi;
                }
            }

            /* 提取天线2信号相位 */
            for (i=0; i<11; i++){
                index = 8+i*3;
                ant2_angle[i] = atan2(my_report.sample[index].q, my_report.sample[index].i);
                if (ant2_angle[i] < 0){
                    ant2_angle[i] = ant2_angle[i] + 2*pi;
                }
            }

            /* 提取天线3信号相位 */
            for (i=0; i<11; i++){
                index = 10+i*3;
                ant3_angle[i] = atan2(my_report.sample[index].q, my_report.sample[index].i);
                if (ant3_angle[i] < 0){
                    ant3_angle[i] = ant3_angle[i] + 2*pi;
                }
            }

            /* 计算1-2相位差 */
            delta_angle_avg1 = 0;
            for (i=0; i<11; i++){
                temp = ant1_angle[i] - (ant2_angle[i]+delta_phi_4us);
                if (my_abs(temp) > pi){
                    if (ant1_angle[i] < pi){
                        temp = ant1_angle[i]+2*pi - (ant2_angle[i]+delta_phi_4us);
                    }
                    else{
                        temp = ant1_angle[i] - (ant2_angle[i]+2*pi+delta_phi_4us);
                    }
                }
                delta_angle_avg1 += temp;
            }
            delta_angle_avg1 = (delta_angle_avg1/11);

            /* 计算1-3相位差 */
            delta_angle_avg2 = 0;
            for (i=0; i<11; i++){
                temp = ant1_angle[i]+delta_phi_4us - ant3_angle[i];
                if (my_abs(temp) > pi){
                    if (ant3_angle[i] > pi){
                        temp = ant1_angle[i]+2*pi+delta_phi_4us - ant3_angle[i];
                    }
                    else{
                        temp = ant1_angle[i]+delta_phi_4us - (ant3_angle[i]+2*pi);
                    }
                }
                delta_angle_avg2 += temp;
            }
            delta_angle_avg2 = (delta_angle_avg2/11);

            /* 计算俯仰角和偏航角 */
            pitch = atan(delta_angle_avg1/delta_angle_avg2);
            yaw = atan(sqrt(delta_angle_avg1*delta_angle_avg1+delta_angle_avg2*delta_angle_avg2)/(PID2/(lambda[my_report.chan_idx])));
           
        }

		/* 发送接收的数据 */
        sprintf(send_data, "id:%d\nrssi:%d\nangle:%f %f\r\n", 1, rssi_data, pitch, yaw);
        printk("%s", send_data);
	}
}



