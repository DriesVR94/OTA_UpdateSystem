/*
 * app1.c
 *
 *  Created on: May 28, 2024
 *      Author: Dries Van Ranst
 */

#include "applications.h"
#include <stdio.h>

void __attribute__((section(".custom_section1"))) app1 (void)
{
	printf("in app 1.1 \r\n");
}

