#ifndef __MYUART_H
#define __MYUART_H

#include <drivers/uart.h>


extern const struct device *uart;

int uart_init(void);


#endif