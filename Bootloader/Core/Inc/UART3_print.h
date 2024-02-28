/*
 * UART_print.h
 *
 *  Created on: Feb 26, 2024
 *      Author: Dries Van Ranst
 */

#ifndef INC_UART3_PRINT_H_
#define INC_UART3_PRINT_H_

void MX_USART3_UART_Init(void);

int uart_putchar(int ch);

#endif /* INC_UART3_PRINT_H_ */
