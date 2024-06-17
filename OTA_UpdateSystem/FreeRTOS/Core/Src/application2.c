/*
 * application2.c
 *
 *  Created on: May 29, 2024
 *      Author: jaumas
 */

#include "applications.h"
#include <stdio.h>


void __attribute__((section(".custom_section1"))) application2 (void)
{
	int a = 11;

	int b = 6;

	int c = a + b;


}

