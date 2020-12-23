#ifndef _STM8_UART_H
#define _STM8_UART_H
/*==================================================================
  File Name    : uart.h
  Author       : Emile
  ------------------------------------------------------------------
  Purpose : This is the header file for uart.c
  ------------------------------------------------------------------
  This is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  This file is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this file.  If not, see <http://www.gnu.org/licenses/>.
  ==================================================================
*/ 
#include <iostm8s103f3.h>
#include <intrinsics.h>
#include <stdint.h>
#include <stdbool.h>

#define UART_BUFLEN (15)
#define TX_BUF_SIZE (20)
#define RX_BUF_SIZE (20)

void    uart_init(void);
void    uart_printf(char *s);
bool    uart_kbhit(void);
uint8_t uart_getc(void);
void    uart_putc(uint8_t ch);

#endif