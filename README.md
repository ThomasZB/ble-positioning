# ble-positioning
# 项目简介

使用蓝牙5.1实现室内定位，项目基于nordic sdk 1.9.1，使用vscode编程

![image-20220418151622193](https://pic-1302177449.cos.ap-chongqing.myqcloud.com/blog_picimage-20220418151622193.png)



# 库修改

## 说明

项目需要在自动连接和不连接之间切换，库中没有对应函数。

## 修改方法

在`scan.c`中挑一个地方加上如下函数，并在`scan.h`中声明

```c
void bt_scan_conn_modify(bool connect_if_match){
	bt_scan.connect_if_match = connect_if_match;
}
```

