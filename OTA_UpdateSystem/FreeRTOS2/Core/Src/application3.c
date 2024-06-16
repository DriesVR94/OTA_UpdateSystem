/*
 * application3.c
 *
 *  Created on: May 29, 2024
 *      Author: jaumas
 */


#include "applications.h"
#include <stdio.h>


void __attribute__((section(".custom_section2"))) application3 (void)
{
	printf("in app 3 \r\n");
}



