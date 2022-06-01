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
#include "zephyr.h"
#include "uart_client.h"
#include "kalman_filter.h"


/* 定义信号量 */
K_SEM_DEFINE(sem_per_adv_success, 0, 1);        /* 用于找设备（周期广播）的信号量 */
K_SEM_DEFINE(sem_per_adv1, 0, 1);        /* 用于找设备（周期广播）的信号量 */
K_SEM_DEFINE(sem_per_adv2, 0, 1);        /* 用于找设备（周期广播）的信号量 */
K_SEM_DEFINE(sem_per_adv3, 0, 1);        /* 用于找设备（周期广播）的信号量 */
K_SEM_DEFINE(sem_per_sync1, 0, 1);       /* 用于同步的信号量 */
K_SEM_DEFINE(sem_per_sync2, 0, 1);       /* 用于同步的信号量 */
K_SEM_DEFINE(sem_per_sync3, 0, 1);       /* 用于同步的信号量 */
K_SEM_DEFINE(sem_per_sync_lost1, 0, 1);  /* 同步丢失的信号量 */
K_SEM_DEFINE(sem_per_sync_lost2, 0, 1);  /* 同步丢失的信号量 */
K_SEM_DEFINE(sem_per_sync_lost3, 0, 1);  /* 同步丢失的信号量 */
K_SEM_DEFINE(sem_rssi_ready, 0, 1);     /* 同步丢失的信号量 */
K_SEM_DEFINE(sem_aod_ready, 0, 1);      /* 同步丢失的信号量 */
K_SEM_DEFINE(my_ble_ready, 0, 1);        /* 用于找设备（周期广播）的信号量 */

/* 是否找到周期广播的信号 */
volatile bool per_adv1_found;
volatile bool per_adv2_found;
volatile bool per_adv3_found;
static struct bt_le_per_adv_sync *sync_handle1;
static struct bt_le_per_adv_sync *sync_handle2;
static struct bt_le_per_adv_sync *sync_handle3;

/* 用于信号同步 */
static uint8_t per_sid1;
static uint8_t per_sid2;
static uint8_t per_sid3;
static bt_addr_le_t per_addr1;
static bt_addr_le_t per_addr2;
static bt_addr_le_t per_addr3;
uint32_t sync1_create_timeout_ms;
uint32_t sync2_create_timeout_ms;
uint32_t sync3_create_timeout_ms;
static uint8_t current_sync = 0;
static uint8_t current_sem = 0;
static uint8_t current_needed_sync = 0; /* 需要建立同步的基站 */

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
int wait_central_adv(uint8_t base_num){
    int err=0;
    printk("Waiting for base%d periodic advertising...\r\n", base_num);
    if (base_num == 1){
        err = k_sem_take(&sem_per_adv1, K_FOREVER);
    }
    else if (base_num == 2){
        err = k_sem_take(&sem_per_adv2, K_FOREVER);
    }
    else if (base_num == 3){
        err = k_sem_take(&sem_per_adv3, K_FOREVER);
    }

    if (err) {
        printk("wait periodic failed (err %d)\r\n", err);
        return err;
    }
    printk("success. Found periodic advertising.\r\n");
    return 0;
}


/**
 * @brief 创建一个同步的句柄
 */
int create_sync_handle(uint8_t sync_num){
    static struct bt_le_per_adv_sync_param sync_create_param1;
    static struct bt_le_per_adv_sync_param sync_create_param2;
    static struct bt_le_per_adv_sync_param sync_create_param3;
	int err=0;
    printk("Create Adv Sync%d\r\n", sync_num);
    if (sync_num == 1){
        bt_addr_le_copy(&sync_create_param1.addr, &per_addr1);
        sync_create_param1.options = BT_LE_PER_ADV_SYNC_OPT_SYNC_ONLY_CONST_TONE_EXT;
        sync_create_param1.sid = per_sid1;
        sync_create_param1.skip = 6;
        sync_create_param1.timeout = 0xa;
        err = bt_le_per_adv_sync_create(&sync_create_param1, &sync_handle1);  
    }
    else if (sync_num == 2){
        bt_addr_le_copy(&sync_create_param2.addr, &per_addr2);
        sync_create_param2.options = BT_LE_PER_ADV_SYNC_OPT_SYNC_ONLY_CONST_TONE_EXT;
        sync_create_param2.sid = per_sid2;
        sync_create_param2.skip = 6;
        sync_create_param2.timeout = 0xa;
        err = bt_le_per_adv_sync_create(&sync_create_param2, &sync_handle2);
    }
    else if (sync_num == 3){
        bt_addr_le_copy(&sync_create_param3.addr, &per_addr3);
        sync_create_param3.options = BT_LE_PER_ADV_SYNC_OPT_SYNC_ONLY_CONST_TONE_EXT;
        sync_create_param3.sid = per_sid3;
        sync_create_param3.skip = 6;
        sync_create_param3.timeout = 0xa;
        err = bt_le_per_adv_sync_create(&sync_create_param3, &sync_handle3);
    }
    return err;
}


/**
 * @brief 删除同步相关句柄（同步出现错误时调用）
 * 
 * @return int 0，删除成功；其他，错误码
 */
int delete_sync(uint8_t sync_num)
{
	int err=0;

#ifdef MYDIRECT_DEBUG
	printk("Deleting Periodic Advertising Sync%d...\r\n", sync_num);
#endif
    if (sync_num == 1){
        err = bt_le_per_adv_sync_delete(sync_handle1);
        sync_handle1 = NULL;
    }
    else if (sync_num == 2){
        err = bt_le_per_adv_sync_delete(sync_handle2);
        sync_handle2 = NULL;
    }
    else if (sync_num == 3){
        err = bt_le_per_adv_sync_delete(sync_handle3);
        sync_handle3 = NULL;
    }
	if (err) {
		printk("sync delete failed (err %d)\r\n", err);
		return err;
	}
#ifdef MYDIRECT_DEBUG
	printk("sync delete success\r\n");
#endif
	return 0;
}



/**
 * @brief 等待同步丢失
 * 
 * @return int 0等待成功；其他，错误码
 */
int wait_sync_lost(uint8_t sync_num){
    int err=0;
    printk("Waiting for periodic sync%d lost...\r\n", sync_num);

    if (sync_num == 1){
        err = k_sem_take(&sem_per_sync_lost1, K_FOREVER);
    }
    else if (sync_num == 2){
        err = k_sem_take(&sem_per_sync_lost2, K_FOREVER);
    }
    else if (sync_num == 3){
        err = k_sem_take(&sem_per_sync_lost3, K_FOREVER);
    }
    
    if (err) {
        printk("wait sync lost failed (err %d)\n", err);
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
int enable_cte_rx(uint8_t sync_num){
    int err=0;
	const struct bt_df_per_adv_sync_cte_rx_param cte_rx_params = {
		.max_cte_count = 5,
		.cte_types = BT_DF_CTE_TYPE_AOD_1US|BT_DF_CTE_TYPE_AOD_2US,
	};

	printk("Enable receiving of CTE of sync%d...\r\n", sync_num);
    
    if (sync_num == 1){
        err = bt_df_per_adv_sync_cte_rx_enable(sync_handle1, &cte_rx_params);
    }
    else if (sync_num == 2){
        err = bt_df_per_adv_sync_cte_rx_enable(sync_handle2, &cte_rx_params);
    }
    else if (sync_num == 3){
        err = bt_df_per_adv_sync_cte_rx_enable(sync_handle3, &cte_rx_params);
    }
	if (err) {
		printk("cte enable failed (err %d)\r\n", err);
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
 * @brief 没有匹配的回调函数，即不是连接对象，进行判断是否是周期广播信号（进行定位）
 * 
 * @param device_info 	：广播设备信息
 * @param connectable 	：是否可连接
 */
void scan_filter_not_match_cb(struct bt_scan_device_info *device_info,
				bool connectable)
{
	char le_addr[BT_ADDR_LE_STR_LEN];
    uint8_t base_num = 0;
    static char name_buf[10] = {0};
	bt_addr_le_to_str(device_info->recv_info->addr, le_addr, sizeof(le_addr));

    /* 找到周期广播信号 */
    if (device_info->recv_info->interval){
        /* 解析名称 */
        if (device_info->adv_data->len > 10){
            if ((uint8_t)device_info->adv_data->data[1] == 9){
                memcpy(name_buf, &(device_info->adv_data->data[2]), 9);
                if (!strcmp(name_buf, "dzy_base1")){
                    base_num = 1;
                }
                else if (!strcmp(name_buf, "dzy_base2")){
                    base_num = 2;
                }
                else if (!strcmp(name_buf, "dzy_base3")){
                    base_num = 3;
                }
            }
        }
        //printk("[scancb]: %d , c_s:%d\r\n", base_num, current_sem);
#ifdef MYDIRECT_DEBUG
        printk("[scancb]: %d , c_s:%d\r\n", base_num, current_sem);
#endif
        /* 创建同步句柄 */
        if ((!per_adv1_found) && (base_num==1) && (current_sem==1)){
            sync1_create_timeout_ms =
			adv_interval_to_ms(device_info->recv_info->interval) * SYNC_CREATE_TIMEOUT_INTERVAL_NUM;
            per_adv1_found = true;
            per_sid1 = device_info->recv_info->sid;
            bt_addr_le_copy(&per_addr1, device_info->recv_info->addr);
            /* 发送信号 */
            k_sem_give(&sem_per_adv1);
            current_sem = 0;
#ifdef MYDIRECT_DEBUG
            printk("[DEVICE]: %s, RSSI %i, sync_create_timeout_ms: %d \r\n", le_addr, device_info->recv_info->rssi, (int)sync1_create_timeout_ms);
#endif
        }
        else if ((!per_adv2_found) && (base_num==2) && (current_sem==2)){
            sync2_create_timeout_ms =
			adv_interval_to_ms(device_info->recv_info->interval) * SYNC_CREATE_TIMEOUT_INTERVAL_NUM;
            per_adv2_found = true;
            per_sid2 = device_info->recv_info->sid;
            bt_addr_le_copy(&per_addr2, device_info->recv_info->addr);
            /* 发送信号 */
            k_sem_give(&sem_per_adv2);
            current_sem = 0;
#ifdef MYDIRECT_DEBUG
            printk("[DEVICE]: %s, RSSI %i, sync_create_timeout_ms: %d \r\n", le_addr, device_info->recv_info->rssi, (int)sync1_create_timeout_ms);
#endif
        }
        else if ((!per_adv3_found) && (base_num==3) && (current_sem==3)){
            sync3_create_timeout_ms =
			adv_interval_to_ms(device_info->recv_info->interval) * SYNC_CREATE_TIMEOUT_INTERVAL_NUM;
            per_adv3_found = true;
            per_sid3 = device_info->recv_info->sid;
            bt_addr_le_copy(&per_addr3, device_info->recv_info->addr);
            /* 发送信号 */
            k_sem_give(&sem_per_adv3);
            current_sem = 0;
#ifdef MYDIRECT_DEBUG
            printk("[DEVICE]: %s, RSSI %i, sync_create_timeout_ms: %d \r\n", le_addr, device_info->recv_info->rssi, (int)sync1_create_timeout_ms);
#endif
        }
    }
    /* 打印信息 */
   
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
#ifdef MYDIRECT_DEBUG
    char le_addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

	printk("PER_ADV_SYNC[%u]: [DEVICE]: %s synced, "
	       "Interval 0x%04x (%u ms), PHY %s\r\n",
	       bt_le_per_adv_sync_get_index(sync), le_addr, info->interval,
	       adv_interval_to_ms(info->interval), phy2str(info->phy));
    
#endif
    printk("synced:%u, cur_need:%d, syn1h:%u, syn2h:%u, syn3h:%u\r\n", bt_le_per_adv_sync_get_index(sync), current_needed_sync, bt_le_per_adv_sync_get_index(sync_handle1), bt_le_per_adv_sync_get_index(sync_handle2), bt_le_per_adv_sync_get_index(sync_handle3));
    if (sync == sync_handle1){
        if (current_needed_sync == 1){
            k_sem_give(&sem_per_sync1);
        }
        else {
            // bt_le_per_adv_sync_delete(sync);
        }
    }
    else if (sync == sync_handle2){
        if (current_needed_sync == 2){
            k_sem_give(&sem_per_sync2);
        }
        else {
            // bt_le_per_adv_sync_delete(sync);
        }
    }
    else if (sync == sync_handle3){
        if (current_needed_sync == 3){
            k_sem_give(&sem_per_sync3);
        }
        else {
            // bt_le_per_adv_sync_delete(sync);
        };
    }
 
	//bt_scan_enable();
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

#ifdef MYDIRECT_DEBUG
	printk("PER_ADV_SYNC[%u]: [DEVICE]: %s sync terminated\r\n",
	       bt_le_per_adv_sync_get_index(sync), le_addr);
#endif
    if (sync == sync_handle1){
        k_sem_give(&sem_per_sync_lost1);
    }
    else if (sync == sync_handle2){
        k_sem_give(&sem_per_sync_lost2);
    }
    else if (sync == sync_handle3){
        k_sem_give(&sem_per_sync_lost3);
    }
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
#ifdef MYDIRECT_DEBUG
	printk("RSSI %i, CTE %s, data length %u, data: %s\r\n", info->rssi, cte_type2str(info->cte_type), buf->len, data_str);
#endif
    printk("rcev_per_cb[%u] RSSI %i\r\n", bt_le_per_adv_sync_get_index(sync), info->rssi);
    if (current_sync == 0){
        if (sync == sync_handle1){
            current_sync = 1;
            rssi_data = info->rssi;
        }
        else if (sync == sync_handle2){
            current_sync = 2;
            rssi_data = info->rssi;
        }
        else if (sync == sync_handle3){
            current_sync = 3;
            rssi_data = info->rssi;
        }
        k_sem_give(&sem_rssi_ready);
    }
    
}


float my_abs(float a){
    if (a<0){
        return -a;
    }
    return a;
}

// #define PID2 (0.1885)
#define PID2 (0.266573)
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
    static uint8_t i=0;  
#ifdef MYDIRECT_DEBUG
	printk("CTE[%u]: samples count %d, cte type %s\r\n", bt_le_per_adv_sync_get_index(sync), report->sample_count, cte_type2str(report->cte_type));
#endif
    printk("CTE[%u]\r\n", bt_le_per_adv_sync_get_index(sync));
    if (current_sync==1 && (sync==sync_handle1)){
        my_report = *report;
        k_sem_give(&sem_aod_ready);
    }
    else if (current_sync==2 && (sync==sync_handle2)){
        my_report = *report;
        k_sem_give(&sem_aod_ready);
    }
    else if (current_sync==3 && (sync==sync_handle3)){
        my_report = *report;
    }
    else {
        i++;
        if (i == 40){
            i = 0;
            current_sync = 0;
        }
        
        return ;
    }
    k_sem_give(&sem_aod_ready);
    
}

float relu_abs1(float a){
    if (a>1){
        return 1;
    }
    else if (a<-1){
        return -1;
    }
    return a;
}




const float base2_ant_bias[] = {-0.578, -0.70667, 0, -0.05, 0.416667, 0.6, 0.618333, 0.55, 0.46, 0.483333, 0.14, 0.3, -0.012, 0.506, 0.83, 0.695, 0.616667, 0.505, 0.395, 0.339, 0.66,1.035,0.664,0.51,0.825,0.806,0.306,0.82,0.78,0.265,1.095,1.16,0.868,1.114,0.8333,1.22333,1.305};
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
    static float ant1_angle[18] = {0};
    static float ant2_angle[18] = {0};
    static float delta_angle_avg1 = 0;
    static float temp;
    static char send_data[100] = {0};
    static uint8_t this_current_sync = 0;
    static struct bt_df_per_adv_sync_iq_samples_report this_report;

    static float this_rssi_data = 0;
    static float this_angle[3] = {0};
    static float this_distance[3] = {0};
    static float A[3] = {-48.4488, -56.4107, -51.4516};
    static float n[3] = {21.550, 21.494, 17.177};
    static uint8_t index2 = 0;
    uint8_t sent_i = 0;

    
	for (;;) {
        /* 等待两个信息的信号量 */
        k_sem_take(&sem_rssi_ready, K_FOREVER);
        k_sem_take(&sem_aod_ready, K_FOREVER);
        this_current_sync = current_sync;
        this_report = my_report;
        current_sync = 0;
        
        if (this_report.sample_count == 45){
            /* 获取参考信号相位 */
            for (i=0; i<8; i++){
                sig_refer_angle[i] = atan2(this_report.sample[i].q, this_report.sample[i].i);
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
            for (i=0; i<18; i++){
                index = 8+i*2;
                ant1_angle[i] = atan2(this_report.sample[index].q, this_report.sample[index].i);
                if (ant1_angle[i] < 0){
                    ant1_angle[i] = ant1_angle[i] + 2*pi;
                }
                ant1_angle[i] = fmod(ant1_angle[i]+delta_phi_4us, 2*pi);
            }

            /* 提取天线2信号相位 */
            for (i=0; i<18; i++){
                index = 9+i*2;
                ant2_angle[i] = atan2(this_report.sample[index].q, this_report.sample[index].i);
                if (ant2_angle[i] < 0){
                    ant2_angle[i] = ant2_angle[i] + 2*pi;
                }
                ant2_angle[i] = fmod(ant2_angle[i]+base2_ant_bias[this_report.chan_idx], 2*pi);
            }

            /* 计算1-2相位差 */
            delta_angle_avg1 = 0;
            for (i=0; i<18; i++){
                temp = (ant1_angle[i]) - (ant2_angle[i]);
                if (my_abs(temp) > pi){
                    if (ant1_angle[i] < pi){
                        temp = (ant1_angle[i]+2*pi) - (ant2_angle[i]);
                    }
                    else{
                        temp = (ant1_angle[i]) - (ant2_angle[i]+2*pi);
                    }
                }
                // sprintf(send_data, "%f\r\n", temp);
                // printk("%s", send_data);
                delta_angle_avg1 += temp;
            }
            delta_angle_avg1 = (delta_angle_avg1/18);


            /* 计算俯仰角和偏航角 */
            pitch = acos(relu_abs1((delta_angle_avg1*lambda[this_report.chan_idx])/(PID2)))*57.29578;
            yaw = 0;
            // pitch = atan(delta_angle_avg1/delta_angle_avg2);
            // yaw = atan(sqrt(delta_angle_avg1*delta_angle_avg1+delta_angle_avg2*delta_angle_avg2)/(PID2/(lambda[my_report.chan_idx])));
        }

		/* 发送接收的数据 */
        sprintf(send_data, "id:%d\nrssi:%d\nangle:%3.1f %3.1f\r\n", this_current_sync, rssi_data, delta_angle_avg1, pitch);
        printk("%s", send_data);
        /* 计算距离并发送到蓝牙网关 */
        if (this_current_sync > 0){
            index2 = this_current_sync-1;
            this_rssi_data = kalman_filter_rssi(rssi_data);
            this_angle[index2] = pitch;
            this_distance[index2] = pow(10, (A[index2]-this_rssi_data)/n[index2]);
            
            sent_i++;
            if (sent_i == 3){
                sent_i = 0;
                sprintf(send_data, "1,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n", this_distance[0], this_distance[1], this_distance[2], this_angle[0], this_angle[1], this_angle[2]);
                if (gatt_had_been_find && current_conn){
                    bt_uart_client_send(send_data, strlen(send_data));
                }
            }
            
        }
        
	}
}


void perodic_adv_thread1(void){
    int err;
    const struct bt_df_per_adv_sync_cte_rx_param cte_rx_params = {
		.max_cte_count = 5,
		.cte_types = BT_DF_CTE_TYPE_AOD_1US|BT_DF_CTE_TYPE_AOD_2US,
	};
    k_sem_take(&my_ble_ready, K_FOREVER);
    k_sem_give(&my_ble_ready);
    k_sem_give(&sem_per_adv_success);
    
    for (;;) {
        /* 找到对应的AOD广播设备 */
        k_sem_take(&sem_per_adv_success, K_FOREVER);
        if (sync_handle1 != NULL){
            delete_sync(1);
        }
        per_adv1_found = 0;
        current_sem = 1;
        printk("Wait base1 adv\r\n");
        err = k_sem_take(&sem_per_adv1, K_MSEC(5000));
        if (err) {
            printk("wait base1 adv failed (err %d)\r\n", err);
            k_sem_give(&sem_per_adv_success);
            k_msleep(10);
            continue;
        }
        printk("Found base1 adv\r\n");
        

		/* 进行同步 */
        current_needed_sync = 1;
		err = create_sync_handle(1);
        if (err) {
            printk("failed create sync1 (err %d)\r\n", err);
            err = delete_sync(1);
            if (err){
                return;
            }
            k_sem_give(&sem_per_adv_success);
            k_msleep(10);
            continue;
        }
		printk("Waite sync1\r\n");
        err = k_sem_take(&sem_per_sync1, K_MSEC(sync1_create_timeout_ms));
        if (err) {
            printk("wait sync1 failed (err %d)\r\n", err);
            err = delete_sync(1);
            if (err){
                return;
            }
            k_sem_give(&sem_per_adv_success);
            k_msleep(10);
            continue;
        }
        printk("success. Periodic sync1 established.\r\n");
        
		/* 接收CTE信息 */
		err = bt_df_per_adv_sync_cte_rx_enable(sync_handle1, &cte_rx_params);
        if (err) {
            printk("cte1 enable failed (err %d)\r\n", err);
            err = delete_sync(1);
            if (err){
                return;
            }
            k_sem_give(&sem_per_adv_success);
            k_msleep(10);
            continue;
        }
        printk("success. CTE receive1 enabled.\r\n");

		/* 等待失去同步 */
        printk("Waite sync1 lost\r\n");
        k_sem_give(&sem_per_adv_success);
        err = k_sem_take(&sem_per_sync_lost1, K_FOREVER);
        if (err) {
            printk("wait sync2 lost failed (err %d)\n", err);
            err = delete_sync(1);
            if (err){
                return;
            }
            continue;
        }
        delete_sync(1);
		printk("sync1 lose sync\r\n");
		k_msleep(2000);    
    }
}


void perodic_adv_thread2(void){
    int err;
    const struct bt_df_per_adv_sync_cte_rx_param cte_rx_params = {
		.max_cte_count = 5,
		.cte_types = BT_DF_CTE_TYPE_AOD_1US|BT_DF_CTE_TYPE_AOD_2US,
	};

    k_sem_take(&my_ble_ready, K_FOREVER);
    k_sem_give(&my_ble_ready);

    for (;;) {
        /* 找到对应的AOD广播设备 */
        k_sem_take(&sem_per_adv_success, K_FOREVER);
        if (sync_handle2 != NULL){
            delete_sync(2);
        }
        per_adv2_found = 0;
        current_sem = 2;
        printk("Wait base2 adv\r\n");
        err = k_sem_take(&sem_per_adv2, K_MSEC(5000));
        if (err) {
            printk("wait base2 adv failed (err %d)\r\n", err);
            k_sem_give(&sem_per_adv_success);
            k_msleep(10);
            continue;
        }
        printk("Found base2 adv\r\n");
        

		/* 创建周期性同步广播句柄 */
        current_needed_sync = 2;
		err = create_sync_handle(2);
        if (err) {
            printk("failed create sync2 (err %d)\r\n", err);
            err = delete_sync(2);
            if (err){
                return;
            }
            k_sem_give(&sem_per_adv_success);
            k_msleep(10);
            continue;
        }
        /* 等待周期性同步广播 */
		printk("Waite sync2\r\n");
        err = k_sem_take(&sem_per_sync2, K_MSEC(sync2_create_timeout_ms));
        if (err) {
            printk("wait sync2 failed (err %d)\r\n", err);
            err = delete_sync(2);
            if (err){
                return;
            }
            k_sem_give(&sem_per_adv_success);
            k_msleep(10);
            continue;
        }
        printk("success. Periodic sync2 established.\r\n");
        

		/* 接收CTE信息 */
        err = bt_df_per_adv_sync_cte_rx_enable(sync_handle2, &cte_rx_params);
        if (err) {
            printk("cte2 enable failed (err %d)\r\n", err);
            err = delete_sync(2);
            if (err){
                return;
            }
            k_sem_give(&sem_per_adv_success);
            k_msleep(10);
            continue;
        }
        printk("success. CTE receive2 enabled.\r\n");

		/* 等待失去同步 */
        printk("Waite sync2 lost\r\n");
        k_sem_give(&sem_per_adv_success);
        err = k_sem_take(&sem_per_sync_lost2, K_FOREVER);
        if (err) {
            printk("wait sync2 lost failed (err %d)\n", err);
            err = delete_sync(2);
            if (err){
                return;
            }
            continue;
        }
        delete_sync(2);
		printk("sync2 lose sync\r\n");
		k_msleep(2000);
    }
}


void perodic_adv_thread3(void){
     int err;
    const struct bt_df_per_adv_sync_cte_rx_param cte_rx_params = {
		.max_cte_count = 5,
		.cte_types = BT_DF_CTE_TYPE_AOD_1US|BT_DF_CTE_TYPE_AOD_2US,
	};

    k_sem_take(&my_ble_ready, K_FOREVER);
    k_sem_give(&my_ble_ready);

    for (;;) {
        /* 找到对应的AOD广播设备 */
        k_sem_take(&sem_per_adv_success, K_FOREVER);
        if (sync_handle3 != NULL){
            delete_sync(3);
        }
        
        per_adv3_found = 0;
        current_sem = 3;
        printk("Wait base3 adv\r\n");
        err = k_sem_take(&sem_per_adv3, K_MSEC(5000));
        if (err) {
            printk("wait base3 adv failed (err %d)\r\n", err);
            k_sem_give(&sem_per_adv_success);
            k_msleep(10);
            continue;
        }
        printk("Found base3 adv\r\n");
        

		/* 创建周期性同步广播句柄 */
        current_needed_sync = 3;
		err = create_sync_handle(3);
        if (err) {
            printk("failed create sync3 (err %d)\r\n", err);
            err = delete_sync(3);
            if (err){
                return;
            }
            k_sem_give(&sem_per_adv_success);
            k_msleep(10);
            continue;
        }
        /* 等待周期性同步广播 */
		printk("Waite sync3\r\n");
        err = k_sem_take(&sem_per_sync3, K_MSEC(sync3_create_timeout_ms));
        if (err) {
            printk("wait sync3 failed (err %d)\r\n", err);
            err = delete_sync(3);
            if (err){
                return;
            }
            k_sem_give(&sem_per_adv_success);
            k_msleep(10);
            continue;
        }
        printk("success. Periodic sync3 established.\r\n");
        

		/* 接收CTE信息 */
        err = bt_df_per_adv_sync_cte_rx_enable(sync_handle3, &cte_rx_params);
        if (err) {
            printk("cte3 enable failed (err %d)\r\n", err);
            err = delete_sync(3);
            if (err){
                return;
            }
            k_sem_give(&sem_per_adv_success);
            k_msleep(10);
            continue;
        }
        printk("success. CTE receive3 enabled.\r\n");

		/* 等待失去同步 */
        printk("Waite sync3 lost\r\n");
        k_sem_give(&sem_per_adv_success);
        err = k_sem_take(&sem_per_sync_lost3, K_FOREVER);
        if (err) {
            printk("wait sync3 lost failed (err %d)\n", err);
            err = delete_sync(3);
            if (err){
                return;
            }
            continue;
        }
		delete_sync(3);
		printk("sync3 lose sync\r\n");
		k_msleep(2000);
    }
}