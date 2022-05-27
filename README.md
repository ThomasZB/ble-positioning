# ble-positioning
# 项目简介

使用蓝牙5.1实现室内定，项目基于nordic sdk 1.9.1，使用vscode编程（芯片选用52811，目前考虑换成52833）

![image-20220418151622193](https://pic-1302177449.cos.ap-chongqing.myqcloud.com/blog_picimage-20220418151622193.png)



# 库修改

## 添加修改自动连接函数

### 说明

项目需要在自动连接和不连接之间切换，库中没有对应函数。

### 修改方法

在`v1.9.1\nrf\subsys\bluetooth\scan.c`中挑一个地方加上如下函数，并在`scan.h`中声明

```c
void bt_scan_conn_modify(bool connect_if_match){
	bt_scan.connect_if_match = connect_if_match;
}
```

## 修改AOD的bug

### 说明

官方的库使用AOD有bug，看了官方的库文件发现其实AOD使用的指令和AOA一样，但官方的代码有一点小bug，没考虑到AOD时天线数量是0

### 修改方法

在`direciton.c`中找到第326行（理论上是326行），也就是函数`prepare_cl_cte_rx_enable_cmd_params`里面，修改方法如下：

```c
//dest_ant_ids = net_buf_add(*buf, params->num_ant_ids); 这一行替换成下面一行
dest_ant_ids = net_buf_add(*buf, switch_pattern_len);
```

# 吐槽

nordic官方代码bug是真的多

# 定位手表板子bug

1. 36和37引脚封装弄反（后面再修）
1. TX引脚接到了NC引脚
1. 按键硬件消抖
1. 电源部分集成到板子
1. 调光电路
