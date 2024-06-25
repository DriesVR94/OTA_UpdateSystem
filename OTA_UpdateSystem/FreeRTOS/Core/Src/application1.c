/*
 * application1.c
 *
 *  Created on: May 29, 2024
 *      Author: jaumas
 */

#include "applications.h"
#include <stdio.h>


void __attribute__((section(".custom_section1"))) application1 (void)
{
	printf("in app 1 \r\n");
}

