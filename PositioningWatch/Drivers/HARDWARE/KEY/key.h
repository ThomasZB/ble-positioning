#ifndef __KEY_H
#define __KEY_H

#include "stdint.h"


typedef struct{
	volatile uint8_t right;
	volatile uint8_t left;
	volatile uint8_t key;
}e5e8_status_type;


extern volatile uint8_t key_status;
extern e5e8_status_type e5e8_status;
extern volatile uint16_t whitch_key_input;


#endif

