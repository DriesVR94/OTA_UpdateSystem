/*
 * application1.c
 *
 *  Created on: May 28, 2024
 *      Author: jaime
 */
#include "applications.h"
#include <stdio.h>


void __attribute__((section(".custom_section0"))) application1 (void)
{
	printf("in app 1 \r\n");
}



