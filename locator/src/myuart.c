/**
 * @file myuart.c
 * @author hang chen (thomaszb.cn)
 * @brief 
 * @version 0.1
 * @date 2022-04-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "myuart.h"

/* 串口设备 */
const struct device *uart;


/**
 * @breif: 串口初始化
 */
int uart_init(void){
	int err;
	static struct uart_data_t rx;

	/* 初始化串口设备 */
	uart = device_get_binding(DT_LABEL(DT_NODELABEL(uart0)));
	if (!uart) {
		return -ENXIO;
	}

	/* 设置串口回调函数 */
	err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		return err;
	}

	/* 初始化串口接收 */
	return uart_rx_enable(uart, rx.data, sizeof(rx.data),
			      UART_RX_TIMEOUT);
}



/**
 * @brief 串口回调函数，目前没用
 * 
 * @param dev 
 * @param evt 
 * @param user_data 
 */
void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data){

}


/**
 * @breif: 重定向printk函数，让printk打印向串口
 */
int arch_printk_char_out(int c)
{

	uart_poll_out(uart, (unsigned char)c);
	/* do nothing */
	return c;
}