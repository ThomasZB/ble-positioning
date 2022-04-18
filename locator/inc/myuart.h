#ifndef __MYUART_H
#define __MYUART_H

#include "drivers/uart.h"

#define UART_BUF_SIZE 20
#define UART_RX_TIMEOUT 50

/* 串口数据格式 */
struct uart_data_t {
	void *fifo_reserved;
	uint8_t  data[UART_BUF_SIZE];
	uint16_t len;
};

extern const struct device *uart;

int uart_init(void);
void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data);


#endif