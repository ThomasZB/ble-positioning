# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(my_test1)

include_directories(inc)
target_sources(app PRIVATE  
    src/main.c
    src/myuart.c
    src/mybluetooth.c
    src/uart_client.c
    src/direction_finding.c
    src/kalman_filter.c)

