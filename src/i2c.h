/*==================================================================
   File Name    : i2c.h
   Author       : Emile
  ------------------------------------------------------------------
  Purpose : This files contains the generic I2C related functions 
            for the STM8 uC. It is needed for every I2C device
			connected to the STM8 uC.
  ------------------------------------------------------------------
  This is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  This software is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this software.  If not, see <http://www.gnu.org/licenses/>.
  ================================================================== */ 
#ifndef _I2C_H
#define _I2C_H

#include <iostm8s103f3.h>
#include <stdbool.h>
#include <intrinsics.h> 
#include <stdint.h>
      
//----------------------------
// I2C defines
//----------------------------
#define I2C_ACK     (0)
#define I2C_NACK    (1)
#define I2C_ERROR   (2)
#define I2C_WRITE   (0)
#define I2C_READ    (1)
#define I2C_RETRIES (3)

//----------------------------
// I2C-peripheral routines
//----------------------------
void    i2c_init(void);                       // Initializes the I2C Interface. Needs to be called only once
uint8_t recv_ack_bit(void);                   // Waits for ACK or NACK bit. Return I2C_ERROR in case of no response
uint8_t i2c_start(uint8_t addr);              // Issues a start condition and sends address and transfer direction
void    i2c_rep_start(uint8_t addr);          // Issues a repeated start condition and sends address and transfer direction
void    i2c_stop(void);                       // Terminates the data transfer and releases the I2C bus
void    i2c_write(uint8_t data);              // Send one byte to I2C device
uint8_t i2c_read1(void);                      // Read one byte from I2C device and calls i2c_stop()
void    i2c_readN(uint8_t *buf, uint8_t len); // Read byte(s) from the I2C device

#endif
