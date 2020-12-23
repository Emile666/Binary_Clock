#ifndef _I2C_BB_H
#define _I2C_BB_H
/*==================================================================
   File Name    : i2c_bb.h
   Author       : Emile
  ------------------------------------------------------------------
  Purpose : This is the header file for i2c_bb.c
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
  ================================================================== */ 
#include <iostm8s103f3.h>
#include <stdbool.h>
#include <intrinsics.h> 
#include <stdint.h>
#include "binary_clock.h" /* For I2C_SDA and I2C_SCL defines */
#include "delay.h"
      
//----------------------------
// I2C defines
//----------------------------
#define I2C_ACK     (0)
#define I2C_NACK    (1)
#define I2C_ERROR   (2)
#define I2C_WRITE   (0)
#define I2C_READ    (1)
#define I2C_RETRIES (3)

#define SDA_in    (PB_DDR &= ~I2C_SDA) 			    /* Set SDA to input */
#define SDA_out   {PB_DDR |=  I2C_SDA; PB_CR1 |=  I2C_SDA;} /* Set SDA to push-pull output */
#define SDA_read  (PB_IDR &   I2C_SDA) 			    /* Read from SDA */
#define SDA_1     (PB_ODR |=  I2C_SDA) 			    /* Set SDA to 1 */
#define SDA_0     (PB_ODR &= ~I2C_SDA) 			    /* Set SDA to 0 */
#define SCL_out   {PB_DDR |=  I2C_SCL; PB_CR1 |=  I2C_SCL;} /* Set SCL to push-pull output */
#define SCL_1     {PB_ODR |=  I2C_SCL; i2c_delay_5usec(1);} /* Set SCL to 1 */
#define SCL_0     {PB_ODR &= ~I2C_SCL; i2c_delay_5usec(1);} /* Set SCL to 0 */

//----------------------------
// I2C-peripheral routines
//----------------------------
void    i2c_delay_5usec(uint16_t x);    // Standard I2C bus delay
uint8_t i2c_reset_bus(void);
void    i2c_init_bb(void);              // Initializes the I2C Interface. Needs to be called only once
uint8_t i2c_start_bb(uint8_t addr);     // Issues a start condition and sends address and transfer direction
uint8_t i2c_rep_start_bb(uint8_t addr); // Issues a repeated start condition and sends address and transfer direction
void    i2c_stop_bb(void);              // Terminates the data transfer and releases the I2C bus
uint8_t i2c_write_bb(uint8_t data);     // Send one byte to I2C device
uint8_t i2c_read_bb(uint8_t ack);       // Read one byte from I2C device and calls i2c_stop()

#endif
