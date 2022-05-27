/**
 * @file direction_finding.h
 * @author hang chen (thomaszb.cn)
 * @brief 
 * @version 0.1
 * @date 2022-04-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __DIRECTION_FINDING_H
#define __DIRECTION_FINDING_H

#include "bluetooth/scan.h"
#include "bluetooth/bluetooth.h"



#define SYNC_CREATE_TIMEOUT_INTERVAL_NUM 7
#define MY_GAP_ADV_MAX_DATA_LEN 3301
#define BLE2STM32_STACKSIZE 1024
#define BLE2STM32_PRIORITY 7


extern struct k_sem sem_per_sync;
extern volatile bool per_adv_found;
extern uint32_t sync_create_timeout_ms;

int wait_sync(void);
int delete_sync(void);
int enable_cte_rx(void);
int wait_sync_lost(void);
int wait_central_adv(void);
void ble2stm32_thread(void);
void create_sync_handle(void);
void direction_finding_init(void);
void scan_filter_not_match_cb(struct bt_scan_device_info*, bool);
void sync_cb(struct bt_le_per_adv_sync*, struct bt_le_per_adv_sync_synced_info*);
void term_cb(struct bt_le_per_adv_sync*, const struct bt_le_per_adv_sync_term_info*);
void cte_recv_cb(struct bt_le_per_adv_sync*, struct bt_df_per_adv_sync_iq_samples_report const*);
void sync_recv_cb(struct bt_le_per_adv_sync*, const struct bt_le_per_adv_sync_recv_info*, struct net_buf_simple*);


#endif