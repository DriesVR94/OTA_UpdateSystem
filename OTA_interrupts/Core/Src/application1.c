/*
 * application1.c
 *
 *  Created on: Mar 1, 2024
 *      Author: Dries Van Ranst
 */

#include "applications.h"
#include "memory_operations.h"
#include <stdio.h>


void __attribute__((section(".custom_section1"))) application1 (void)
{
	printf("in app 1 \r\n");
}



