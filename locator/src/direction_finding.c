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


/* 定义信号量 */
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
void create_sync_handle(uint8_t sync_num){
    static struct bt_le_per_adv_sync_param sync_create_param1;
    static struct bt_le_per_adv_sync_param sync_create_param2;
    static struct bt_le_per_adv_sync_param sync_create_param3;
	int err=0;
    printk("Creating Periodic Advertising Sync%d...\r\n", sync_num);
    if (sync_num == 1){
        bt_addr_le_copy(&sync_create_param1.addr, &per_addr1);
        sync_create_param1.options = BT_LE_PER_ADV_SYNC_OPT_SYNC_ONLY_CONST_TONE_EXT;
        sync_create_param1.sid = per_sid1;
        sync_create_param1.skip = 0;
        sync_create_param1.timeout = 0xa;
        err = bt_le_per_adv_sync_create(&sync_create_param1, &sync_handle1);  
    }
    else if (sync_num == 2){
        bt_addr_le_copy(&sync_create_param2.addr, &per_addr2);
        sync_create_param2.options = BT_LE_PER_ADV_SYNC_OPT_SYNC_ONLY_CONST_TONE_EXT;
        sync_create_param2.sid = per_sid2;
        sync_create_param2.skip = 0;
        sync_create_param2.timeout = 0xa;
        err = bt_le_per_adv_sync_create(&sync_create_param2, &sync_handle2);
    }
    else if (sync_num == 3){
        bt_addr_le_copy(&sync_create_param3.addr, &per_addr3);
        sync_create_param3.options = BT_LE_PER_ADV_SYNC_OPT_SYNC_ONLY_CONST_TONE_EXT;
        sync_create_param3.sid = per_sid3;
        sync_create_param3.skip = 0;
        sync_create_param3.timeout = 0xa;
        err = bt_le_per_adv_sync_create(&sync_create_param3, &sync_handle3);
    }
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
int delete_sync(uint8_t sync_num)
{
	int err=0;

	printk("Deleting Periodic Advertising Sync%d...\r\n", sync_num);
    if (sync_num == 1){
        err = bt_le_per_adv_sync_delete(sync_handle1);
    }
    else if (sync_num == 2){
        err = bt_le_per_adv_sync_delete(sync_handle2);
    }
    else if (sync_num == 3){
        err = bt_le_per_adv_sync_delete(sync_handle3);
    }
	if (err) {
		printk("sync delete failed (err %d)\r\n", err);
		return err;
	}
	printk("sync delete success\r\n");
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
    if (device_info->recv_info->interval){
        if (!per_adv1_found){
            sync1_create_timeout_ms =
			adv_interval_to_ms(device_info->recv_info->interval) * SYNC_CREATE_TIMEOUT_INTERVAL_NUM;
            per_adv1_found = true;
            per_sid1 = device_info->recv_info->sid;
            bt_addr_le_copy(&per_addr1, device_info->recv_info->addr);
            /* 发送信号 */
            k_sem_give(&sem_per_adv1);
            printk("[DEVICE]: %s, RSSI %i, sync_create_timeout_ms: %d \r\n", le_addr, device_info->recv_info->rssi, (int)adv_interval_to_ms);
        }
        // else if (!per_adv2_found){
        //     sync2_create_timeout_ms =
		// 	adv_interval_to_ms(device_info->recv_info->interval) * SYNC_CREATE_TIMEOUT_INTERVAL_NUM;
        //     per_adv2_found = true;
        //     per_sid2 = device_info->recv_info->sid;
        //     bt_addr_le_copy(&per_addr2, device_info->recv_info->addr);
        //     /* 发送信号 */
        //     k_sem_give(&sem_per_adv2);
        //     printk("[DEVICE]: %s, RSSI %i, sync_create_timeout_ms: %d \r\n", le_addr, device_info->recv_info->rssi, (int)adv_interval_to_ms);
        // }
        // else if (!per_adv3_found){
        //     sync3_create_timeout_ms =
		// 	adv_interval_to_ms(device_info->recv_info->interval) * SYNC_CREATE_TIMEOUT_INTERVAL_NUM;
        //     per_adv3_found = true;
        //     per_sid3 = device_info->recv_info->sid;
        //     bt_addr_le_copy(&per_addr3, device_info->recv_info->addr);
        //     /* 发送信号 */
        //     k_sem_give(&sem_per_adv3);
        //     printk("[DEVICE]: %s, RSSI %i, sync_create_timeout_ms: %d \r\n", le_addr, device_info->recv_info->rssi, (int)adv_interval_to_ms);
        // }
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
	char le_addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

	printk("PER_ADV_SYNC[%u]: [DEVICE]: %s synced, "
	       "Interval 0x%04x (%u ms), PHY %s\r\n",
	       bt_le_per_adv_sync_get_index(sync), le_addr, info->interval,
	       adv_interval_to_ms(info->interval), phy2str(info->phy));
    if (sync == sync_handle1){
        k_sem_give(&sem_per_sync1);
    }
    else if (sync == sync_handle2){
        k_sem_give(&sem_per_sync2);
    }
    else if (sync == sync_handle3){
        k_sem_give(&sem_per_sync3);
    }
 
	
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
	printk("RSSI %i, CTE %s, data length %u, data: %s\r\n", info->rssi, cte_type2str(info->cte_type), buf->len, data_str);
    
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
	printk("CTE[%u]: samples count %d, cte type %s\r\n", bt_le_per_adv_sync_get_index(sync), report->sample_count, cte_type2str(report->cte_type));
    
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
    static float ant3_angle[18] = {0};
    static float delta_angle_avg1 = 0;
    static float delta_angle_avg2 = 0;
    static float temp;
    static char send_data[100] = {0};
    static float ant2_bias1 = -1.1;

    
	for (;;) {
        /* 等待两个信息的信号量 */
        current_sync = 0;
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
            for (i=0; i<18; i++){
                index = 8+i*2;
                ant1_angle[i] = atan2(my_report.sample[index].q, my_report.sample[index].i);
                if (ant1_angle[i] < 0){
                    ant1_angle[i] = ant1_angle[i] + 2*pi;
                }
            }

            /* 提取天线2信号相位 */
            for (i=0; i<18; i++){
                index = 9+i*2;
                ant2_angle[i] = atan2(my_report.sample[index].q, my_report.sample[index].i);
                if (ant2_angle[i] < 0){
                    ant2_angle[i] = ant2_angle[i] + 2*pi;
                }
                ant2_angle[i] = fmod(ant2_angle[i]+ant2_bias1, 2*pi);
            }

            /* 计算1-2相位差 */
            delta_angle_avg1 = 0;
            for (i=0; i<18; i++){
                temp = (ant1_angle[i]+delta_phi_4us) - (ant2_angle[i]);
                if (my_abs(temp) > pi){
                    if (ant1_angle[i] < pi){
                        temp = (ant1_angle[i]+2*pi+delta_phi_4us) - (ant2_angle[i]);
                    }
                    else{
                        temp = (ant1_angle[i]+delta_phi_4us) - (ant2_angle[i]+2*pi+delta_phi_4us);
                    }
                }
                // sprintf(send_data, "%f\r\n", temp);
                // printk("%s", send_data);
                delta_angle_avg1 += temp;
            }
            delta_angle_avg1 = (delta_angle_avg1/18);


            /* 计算俯仰角和偏航角 */
            pitch = acos(relu_abs1((delta_angle_avg1*lambda[my_report.chan_idx])/(PID2)))*57.29578;
            yaw = 0;
            // pitch = atan(delta_angle_avg1/delta_angle_avg2);
            // yaw = atan(sqrt(delta_angle_avg1*delta_angle_avg1+delta_angle_avg2*delta_angle_avg2)/(PID2/(lambda[my_report.chan_idx])));
           
        }

		/* 发送接收的数据 */
        sprintf(send_data, "id:%d\nrssi:%d\nangle:%d %f\r\n", 1, rssi_data, pitch, delta_angle_avg1);
        printk("%s", send_data);
	}
}


void perodic_adv_thread1(void){
    int err;
    k_sem_take(&my_ble_ready, K_FOREVER);
    k_sem_give(&my_ble_ready);
    for (;;) {
        per_adv1_found = 0;
        bt_scan_enable();
        /* 找到对应的AOD广播设备 */
        printk("Waiting for base1 periodic advertising...\r\n");
        err = k_sem_take(&sem_per_adv1, K_FOREVER);
        if (err) {
            printk("wait periodic failed (err %d)\r\n", err);
            continue;
        }
        printk("success. Found periodic advertising.\r\n");

		/* 进行同步 */
		create_sync_handle(1);
		printk("Waiting for periodic sync...\r\n");
        err = k_sem_take(&sem_per_sync1, K_MSEC(sync1_create_timeout_ms));
        if (err) {
            printk("failed waitting (err %d)\r\n", err);
            err = delete_sync(1);
            if (err){
                return;
            }
            continue;
        }
        printk("success. Periodic sync1 established.\r\n");

		/* 接收CTE信息 */
		err = enable_cte_rx(1);
        if (err) {
            err = delete_sync(1);
            if (err){
                return;
            }
            continue;
        }

		/* 等待失去同步 */
        printk("Waiting for periodic sync1 lost...\r\n");
        err = k_sem_take(&sem_per_sync_lost1, K_FOREVER);
        if (err) {
            printk("wait sync lost failed (err %d)\n", err);
            err = delete_sync(1);
            if (err){
                return;
            }
            continue;
        }
		printk("sync1 lose sync\r\n");
		k_msleep(2000);
    }
}


void perodic_adv_thread2(void){
    int err;
    k_sem_take(&my_ble_ready, K_FOREVER);
    k_sem_give(&my_ble_ready);
    for (;;) {
        per_adv2_found = 0;
        bt_scan_enable();
        /* 找到对应的AOD广播设备 */
        printk("Waiting for base2 periodic advertising...\r\n");
        err = k_sem_take(&sem_per_adv2, K_FOREVER);
        if (err) {
            printk("wait periodic failed (err %d)\r\n", err);
            continue;
        }
        printk("success. Found periodic advertising.\r\n");

		/* 进行同步 */
		create_sync_handle(2);
		printk("Waiting for periodic sync2...\r\n");
        err = k_sem_take(&sem_per_sync2, K_MSEC(sync2_create_timeout_ms));
        if (err) {
            printk("failed waitting sync2 (err %d)\r\n", err);
            err = delete_sync(2);
            if (err){
                return;
            }
            continue;
        }
        printk("success. Periodic sync2 established.\r\n");

		/* 接收CTE信息 */
		err = enable_cte_rx(2);
        if (err) {
            err = delete_sync(2);
            if (err){
                return;
            }
            continue;
        }

		/* 等待失去同步 */
		wait_sync_lost(2);
		
		printk("sync2 lose sync\r\n");
		k_msleep(2000);
    }
}


void perodic_adv_thread3(void){
    
}