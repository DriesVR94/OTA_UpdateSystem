/*
 * application2.c
 *
 *  Created on: Mar 9, 2024
 *      Author: Jaime Ibáñez Rivera
 */

#include "applications.h"
#include <stdio.h>
#include "memory_operations.h"

void __attribute__((section(".custom_section3"))) application3 (void)
{
	printf("in app 21 \r\n");
}



