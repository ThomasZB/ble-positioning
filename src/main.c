/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include "myuart.h"


void main(void)
{
	unsigned char a[] = "hello world\r\n";

	NRFX_DELAY_US(1000000);
	uart_init();

	while (1){
		uart_tx(uart, a, sizeof(a), 10000);
		// printk("hello world!\r\n");
		NRFX_DELAY_US(2000000);
	}
	
}
