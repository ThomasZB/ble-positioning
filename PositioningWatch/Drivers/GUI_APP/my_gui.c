/**
 * @file my_gui.c
 * @author hang chen (thomaszb@qq.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "my_gui.h"
#include "lvgl.h"


char num[3] = "-46";
lv_point_t line_points[IMU_MAX_LEN] = { {100, 100}, {120, 140}, {130, 160}, {124, 155}, {135, 130} };

/* 声明字体 */
LV_FONT_DECLARE(my_font);

/* 样式 */
lv_style_t my_style;
/* 标签 */
lv_obj_t* aoa_data_tab;
lv_obj_t* imu_data_tab;
lv_obj_t* rssi_data_tab;
lv_obj_t* data_monitoring_tv;
/* 距离 */
lv_obj_t* ant1_rssi_label;
lv_obj_t* ant1_distance_label;
lv_obj_t* ant2_rssi_label;
lv_obj_t* ant2_distance_label;
lv_obj_t* ant3_rssi_label;
lv_obj_t* ant3_distance_label;
/* 角度 */
lv_obj_t* ant1_yaw_angle_label;
lv_obj_t* ant1_pitch_angle_label;
lv_obj_t* ant2_yaw_angle_label;
lv_obj_t* ant2_pitch_angle_label;
lv_obj_t* ant3_yaw_angle_label;
lv_obj_t* ant3_pitch_angle_label;
/* 位移 */
lv_obj_t* step_count_label;

static lv_group_t * g;




/* 内部调用 */
float my_relu10(float a);
void imu_data_create(lv_obj_t* tab);
void aod_data_create(lv_obj_t* AOD);
void rssi_data_create(lv_obj_t* tab);


/**
 * @brief 启动gui
 * 
 */
void my_gui_start(void){
	/* 绑定按键 */
	g = lv_group_create();
    lv_group_set_default(g);

    lv_indev_t * cur_drv = NULL;
    for(;;) {
        cur_drv = lv_indev_get_next(cur_drv);
        if(!cur_drv) {
            break;
        }

        if(cur_drv->driver->type == LV_INDEV_TYPE_KEYPAD) {
            lv_indev_set_group(cur_drv, g);
        }

        if(cur_drv->driver->type == LV_INDEV_TYPE_ENCODER) {
            lv_indev_set_group(cur_drv, g);
        }
    }
	
	
    data_monitoring_tv = lv_tabview_create(lv_scr_act(), LV_DIR_TOP, LV_DPI_DEF / 3);

    /* 设置样式 */
    lv_style_init(&my_style);
    lv_style_set_text_font(&my_style, &my_font);

    /* 创建table */
    rssi_data_tab = lv_tabview_add_tab(data_monitoring_tv, "RSSI");
    aoa_data_tab = lv_tabview_add_tab(data_monitoring_tv, "AOD");
    imu_data_tab = lv_tabview_add_tab(data_monitoring_tv, "IMU");

    /* table中内容创建 */
    lv_obj_add_style(rssi_data_tab, &my_style, 0);
    rssi_data_create(rssi_data_tab);
    lv_obj_add_style(aoa_data_tab, &my_style, 0);
    aod_data_create(aoa_data_tab);
    lv_obj_add_style(imu_data_tab, &my_style, 0);
    imu_data_create(imu_data_tab);
}


/**
 * @brief 设置rssi的数据
 * 
 * @param ant_num   ：哪一根天线
 * @param rssi      ：rssi
 * @param distance  ：距离
 */
void set_rssi_data(uint8_t ant_num, int rssi, float distance){
    if (ant_num == 1){
        lv_label_set_text_fmt(ant1_rssi_label, "%d", rssi);
        lv_label_set_text_fmt(ant1_distance_label, "%.3f", distance);
    }
    else if (ant_num == 2){
        lv_label_set_text_fmt(ant2_rssi_label, "%d", rssi);
        lv_label_set_text_fmt(ant2_distance_label, "%.3f", distance);
    }
    else if (ant_num == 3){
        lv_label_set_text_fmt(ant3_rssi_label, "%d", rssi);
        lv_label_set_text_fmt(ant3_distance_label, "%.3f", distance);
    }
}


/**
 * @brief 设置AOD数据
 * 
 * @param ant_num   ：哪一跟天线
 * @param angle     ：角度
 */
void set_aod_data(uint8_t ant_num, float pitch_angle, float yaw_angle){
    if (ant_num == 1){
        lv_label_set_text_fmt(ant1_pitch_angle_label, "%03.1f°", pitch_angle);
        lv_label_set_text_fmt(ant1_yaw_angle_label, "%03.1f°", yaw_angle);;
    }
    else if (ant_num == 2){
        lv_label_set_text_fmt(ant2_pitch_angle_label, "%03.1f°", pitch_angle);
        lv_label_set_text_fmt(ant2_yaw_angle_label, "%03.1f°", yaw_angle);;
    }
    else if (ant_num == 3){
        lv_label_set_text_fmt(ant3_pitch_angle_label, "%03.1f°", pitch_angle);
        lv_label_set_text_fmt(ant3_yaw_angle_label, "%03.1f°", yaw_angle);;
    }
}


uint8_t imu_set_step(unsigned long step_count){
    lv_label_set_text_fmt(step_count_label, "已经走了%d步 ", step_count);
	return 0;
}


/**
 * @brief 创建一个图层用于画位移
 * 
 * @param tab ：在哪里创建
 */
void imu_data_create(lv_obj_t* tab){
    lv_obj_t* name;

    /* 创建标题 */
    name = lv_label_create(tab);
    lv_obj_set_pos(name, 0, 0);
    lv_label_set_text_static(name, "步数:");

    step_count_label = lv_label_create(tab);
    lv_obj_set_pos(step_count_label, 40, 32);
    lv_label_set_text_fmt(step_count_label, "已经走了%d步 ", 0);
}


/**
 * @brief 创建AOD的界面
 * 
 * @param tab ：在哪一个tab下创建
 */
void aod_data_create(lv_obj_t* tab){
    lv_obj_t* name;

    /* 创建标题 */
    name = lv_label_create(tab);
    lv_obj_set_pos(name, 0, 0);
    lv_label_set_text_static(name, "与天线1角度:");
    name = lv_label_create(tab);
    lv_obj_set_pos(name, 16, 32);
    lv_label_set_text_static(name, "俯仰:");
    name = lv_label_create(tab);
    lv_obj_set_pos(name, 120, 32);
    lv_label_set_text_static(name, "水平:");
    /* 创建数字 */
    ant1_pitch_angle_label = lv_label_create(tab);
    lv_obj_set_pos(ant1_pitch_angle_label, 160, 32);
    lv_label_set_text_fmt(ant1_pitch_angle_label, "%03.1f°", 23.1);
    ant1_yaw_angle_label = lv_label_create(tab);
    lv_obj_set_pos(ant1_yaw_angle_label, 56, 32);
    lv_label_set_text_fmt(ant1_yaw_angle_label, "%03.1f°", 23.1);
        
    /* 创建标题 */
    name = lv_label_create(tab);
    lv_obj_set_pos(name, 0, 64);
    lv_label_set_text_static(name, "与天线1角度:");
    name = lv_label_create(tab);
    lv_obj_set_pos(name, 16, 96);
    lv_label_set_text_static(name, "俯仰:");
    name = lv_label_create(tab);
    lv_obj_set_pos(name, 120, 96);
    lv_label_set_text_static(name, "水平:");
    /* 创建数字 */
    ant2_pitch_angle_label = lv_label_create(tab);
    lv_obj_set_pos(ant2_pitch_angle_label, 160, 96);
    lv_label_set_text_fmt(ant2_pitch_angle_label, "%03.1f°", 23.1);
    ant2_yaw_angle_label = lv_label_create(tab);
    lv_obj_set_pos(ant2_yaw_angle_label, 56, 96);
    lv_label_set_text_fmt(ant2_yaw_angle_label, "%03.1f°", 23.1);
    
    /* 创建标题 */
    name = lv_label_create(tab);
    lv_obj_set_pos(name, 0, 128);
    lv_label_set_text_static(name, "与天线1角度:");
    name = lv_label_create(tab);
    lv_obj_set_pos(name, 16, 160);
    lv_label_set_text_static(name, "俯仰:");
    name = lv_label_create(tab);
    lv_obj_set_pos(name, 120, 160);
    lv_label_set_text_static(name, "水平:");
    /* 创建数字 */
    ant3_pitch_angle_label = lv_label_create(tab);
    lv_obj_set_pos(ant3_pitch_angle_label, 160, 160);
    lv_label_set_text_fmt(ant3_pitch_angle_label, "%03.1f°", 23.1);
    ant3_yaw_angle_label = lv_label_create(tab);
    lv_obj_set_pos(ant3_yaw_angle_label, 56, 160);
    lv_label_set_text_fmt(ant3_yaw_angle_label, "%03.1f°", 23.1);
    
}


/**
 * @brief 创建RSSI的界面
 * 
 * @param tab ：在哪一个tab下创建
 */
void rssi_data_create(lv_obj_t* tab){
    lv_obj_t* name;

    /* 创建标题 */
    name = lv_label_create(tab);
    lv_obj_set_pos(name, 0, 0);
    lv_label_set_text_static(name, "天线1:");
    name = lv_label_create(tab);
    lv_obj_set_pos(name, 16, 32);
    lv_label_set_text_static(name, "RSSI:");
    name = lv_label_create(tab);
    lv_obj_set_pos(name, 96, 32);
    lv_label_set_text_static(name, "距离:");
    /* 创建数字 */
    ant1_rssi_label = lv_label_create(tab);
    lv_obj_set_pos(ant1_rssi_label, 56, 32);
    lv_label_set_text_fmt(ant1_rssi_label, "%d", 0);
    ant1_distance_label = lv_label_create(tab);
    lv_obj_set_pos(ant1_distance_label, 136, 32);
    lv_label_set_text_fmt(ant1_distance_label, "%.3f", 1.23);
    
     /* 创建标题 */
    name = lv_label_create(tab);
    lv_obj_set_pos(name, 0, 64);
    lv_label_set_text_static(name, "天线2:");
    name = lv_label_create(tab);
    lv_obj_set_pos(name, 16, 96);
    lv_label_set_text_static(name, "RSSI:");
    name = lv_label_create(tab);
    lv_obj_set_pos(name, 96, 96);
    lv_label_set_text_static(name, "距离:");
    /* 创建数字 */
    ant2_rssi_label = lv_label_create(tab);
    lv_obj_set_pos(ant2_rssi_label, 56, 96);
    lv_label_set_text_fmt(ant2_rssi_label, "%d", 0);
    ant2_distance_label = lv_label_create(tab);
    lv_obj_set_pos(ant2_distance_label, 136, 96);
    lv_label_set_text_fmt(ant2_distance_label, "%.3f", 1.23);

    /* 创建标题 */
    name = lv_label_create(tab);
    lv_obj_set_pos(name, 0, 128);
    lv_label_set_text_static(name, "天线3:");
    name = lv_label_create(tab);
    lv_obj_set_pos(name, 16, 160);
    lv_label_set_text_static(name, "RSSI:");
    name = lv_label_create(tab);
    lv_obj_set_pos(name, 96, 160);
    lv_label_set_text_static(name, "距离:");
    /* 创建数字 */
    ant3_rssi_label = lv_label_create(tab);
    lv_obj_set_pos(ant3_rssi_label, 56, 160);
    lv_label_set_text_fmt(ant3_rssi_label, "%d", 0);
    ant3_distance_label = lv_label_create(tab);
    lv_obj_set_pos(ant3_distance_label, 136, 160);
    lv_label_set_text_fmt(ant3_distance_label, "%.3f", 1.23);
}


float my_relu10(float a){
    if (a>10){
        return 10;
    }
    else if (a<-10){
        return -10;
    }
    return a;
}
