/*
 * application1.c
 *
 *  Created on: Apr 26, 2024
 *      Author: jaime
 */

#include "applications.h"
#include "memory_operations.h"
#include <stdio.h>


void __attribute__((section(".custom_section2"))) application11 (void)
{
	printf("in app 1.1 \r\n");

	int a = 32;
	int b = 10;


	for (int e =0; e<15; e++)
	{
		e = e +a-b;
	}

	for (int e =0; e<15; e++)
	{
		e = e +a-b;
	}

	for (int e =0; e<15; e++)
	{
		e = e +a-b;

	}
}

