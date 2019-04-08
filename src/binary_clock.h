#ifndef BINARY_CLOCK_H
#define BINARY_CLOCK_H

#include <iostm8s103f3.h>
#include <intrinsics.h> 
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

//-----------------------------------------------------------------------------------------------
//  Schematic of the connections to the MCU.
// 
//                                      STM8S103F3
//                                     ------------
//                                 PD4 | 1     20 | PD3/AIN4 LED2 colon lower left
//                      UART1-TX   PD5 | 2     19 | PD2/AIN3 LED1 colon upper left
//                      UART1-RX   PD6 | 3     18 | PD1      SWIM
//                                nRST | 4     17 | PC7      
//                                 PA1 | 5     16 | PC6      LED4 colon lower right
//                                 PA2 | 6     15 | PC5      LED3 colon upper right
//                                 GND | 7     14 | PC4      IR_RCV
//                                VCAP | 8     13 | PC3      DI_3V3
//                                 VCC | 9     12 | PB4      I2C-SCL
//                                 PA3 | 10    11 | PB5      I2C-SDA
//                                     ------------
//-----------------------------------------------------------------------------------------------
#define I2C_SCL (0x10) /* PB4 */
#define I2C_SDA (0x20) /* PB5 */
#define DI_3V3  (0x08) /* PC3 */
#define IR_RCV  (0x10) /* PC4 */
#define LED3    (0x20) /* PC5 */
#define LED4    (0x40) /* PC6 */
#define LED1    (0x04) /* PD2 */
#define LED2    (0x08) /* PD3 */
#define TX      (0x20) /* PD5 */
#define RX      (0x40) /* PD6 */

//-----------------------------------------------------------------------------------------------
// https://wp.josh.com/2014/05/13/ws2812-neopixels-are-not-so-finicky-once-you-get-to-know-them/
//
// At 16 MHz, 1 NOP is approx. 62.5 nsec.
//
// Symbol Parameter	                Min	Typical	Max	Units Measured
// T0H	  0 code ,high voltage time	200	350	500	ns    360..400
// T1H	  1 code ,high voltage time	550	700	5.500	ns    760
// TLD	  data, low voltage time	450	600	5.000	ns    1120
// TLL	  latch, low voltage time	6.000			ns    3120 (max)
//
//-----------------------------------------------------------------------------------------------
#define wait_T0H  __asm("nop\n nop\n nop\n nop\n nop")
#define wait_T1H  wait_T0H; wait_T0H; __asm("nop")

#define ws2812b_send_1   PC_ODR |=  DI_3V3; /* Turn PC3 on */  \
                         wait_T1H;                           \
                         PC_ODR &= ~DI_3V3; /* Turn PC3 off */ 
#define ws2812b_send_0   PC_ODR |=  DI_3V3; /* Turn PC3 on */  \
                         wait_T0H;                           \
                         PC_ODR &= ~DI_3V3; /* Turn PC3 off */ 
                             
//-------------------------------------------------
// The Number of WS2812B devices present
// For the binary clock, this is a total of 20
//-------------------------------------------------
#define NR_LEDS (20)                    

//-------------------------------------------------
// Constants for the independent watchdog (IWDG)
//-------------------------------------------------
#define IWDG_KR_KEY_ENABLE  (0xCC)
#define IWDG_KR_KEY_REFRESH (0xAA)
#define IWDG_KR_KEY_ACCESS  (0x55)

void print_date_and_time(void);
void print_dow(uint8_t dow);
void execute_single_command(char *s);
void rs232_command_handler(void);

#endif
