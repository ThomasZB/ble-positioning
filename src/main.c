/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include "mybluetooth.h"
#include "myuart.h"


void main(void)
{
	int err;
	
	NRFX_DELAY_US(1000000);
	uart_init();

	err = bt_enable(bt_ready_callback);
	while (1){
		printk("hello world\r\n");
		NRFX_DELAY_US(2000000);
	}
}
