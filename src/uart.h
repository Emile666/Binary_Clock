#ifndef _UART_H
#define _UART_H

#include <iostm8s103f3.h>
#include <intrinsics.h>
#include <stdint.h>

#define NO_ERR      (0x00)
#define ERR_CMD	    (0x01)
#define ERR_NUM	    (0x02)
#define UART_BUFLEN (20)

void    uart_init(void);
void    uart_printf(char *message);
uint8_t uart_kbhit(void);
int8_t  uart_getc(void);
void    uart_putc(char ch);

#endif