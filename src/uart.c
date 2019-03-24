//
//  This program shows how you can output a message on the UART on
//	the STM8S microcontroller.
//
//  This software is provided under the CC BY-SA 3.0 licence.  A
//  copy of this licence can be found at:
//
//  http://creativecommons.org/licenses/by-sa/3.0/legalcode
//
#include <stdio.h>
#include "uart.h"

char    rs232_inbuf[UART_BUFLEN];     // buffer for RS232 commands
uint8_t rs232_ptr = 0;                // index in RS232 buffer

//
//  Setup the UART to run at 115200 baud, no parity, one stop bit, 8 data bits.
//  Important: This relies upon the system-clock being set to run at 16 MHz.
//
void uart_init(void)
{
    //
    //  Clear the Idle Line Detected bit in the status register by a read
    //  to the UART1_SR register followed by a Read to the UART1_DR register.
    //
    unsigned char tmp = UART1_SR;
    tmp = UART1_DR;
    //
    //  Reset the UART registers to the reset values.
    //
    UART1_CR1 = 0;
    UART1_CR2 = 0;
    UART1_CR4 = 0;
    UART1_CR3 = 0;
    UART1_CR5 = 0;
    UART1_GTR = 0;
    UART1_PSCR = 0;
    //
    //  Now setup the port to 115200,n,8,1.
    //
    UART1_CR1_M = 0;        //  8 Data bits.
    UART1_CR1_PCEN = 0;     //  Disable parity.
    UART1_CR3_STOP = 0;     //  1 stop bit.
    UART1_BRR2 = 0x0b;      //  Set the baud rate registers to 115200 baud
    UART1_BRR1 = 0x08;      //  based upon a 16 MHz system clock.
    //
    //  Disable the transmitter and receiver.
    //
    UART1_CR2_TEN = 0;      //  Disable transmit.
    UART1_CR2_REN = 0;      //  Disable receive.
    //
    //  Set the clock polarity, clock phase and last bit clock pulse.
    //
    UART1_CR3_CPOL = 0;
    UART1_CR3_CPHA = 0;
    UART1_CR3_LBCL = 0;
    //
    //  Turn on the UART transmit, receive and the UART clock.
    //
    UART1_CR2_TEN = 1;
    UART1_CR2_REN = 1;
    UART1_CR3_CKEN = 0; // set to 0 or receive will not work!!
} // uart_init()

void uart_putc(char ch)
{    
     UART1_DR = (unsigned char)ch;     //  Put the next character into the data transmission register.
     while (UART1_SR_TXE == 0);        //  Wait for transmission to complete.
} // uart_putc()

//
//  Send the message in the string to UART1.
//
void uart_printf(char *message)
{
    char *ch = message;
    while (*ch)
    {
        if (*ch == '\n')
        {
            uart_putc('\r'); // add CR
        } // if
        uart_putc(*ch);
        ch++;                //  Grab the next character.
    } // while
} // uart_printf()

uint8_t uart_kbhit(void)
{
    return UART1_SR_RXNE;   // 1 if char is available
} // uart_kbhit()

int8_t uart_getc(void)
{
  return (UART1_DR);        // Get it
} // uart_getch()

