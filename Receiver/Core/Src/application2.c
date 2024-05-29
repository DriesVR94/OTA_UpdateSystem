/*
 * application2.c
 *
 *  Created on: May 29, 2024
 *      Author: jaumas
 */

#include "applications.h"
#include <stdio.h>


void __attribute__((section(".custom_section2"))) application2 (void)
{
	printf("in app 2 \r\n");
}

