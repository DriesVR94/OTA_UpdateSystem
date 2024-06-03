/*
 * application2.c
 *
 *  Created on: Mar 1, 2024
 *      Author: Dries Van Ranst
 */

#include "applications.h"
#include <stdio.h>
#include "memory_operations.h"

void __attribute__((section(".custom_section2"))) application2 (void)
{
	printf("in app 2 \r\n");
}



