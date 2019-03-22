/*==================================================================
  File Name    : i2c.c
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
  ==================================================================
*/ 
#include "i2c.h"
#include "delay.h"

void i2c_delay(void)
{   
    uint8_t i;
    
    for (i = 0; i < 100; i++) ;
} // i2c_delay()

//  I2C interrupts all share the same handler.
#pragma vector=I2C_ADDR_vector
__interrupt void I2C_IRQHandler(void)
{
} // I2C_IRQHandler()

/*-----------------------------------------------------------------------------
  Purpose  : This function initializes the I2C bus controller
  Variables: bb = true: use bit-banging instead of I2C device
  Returns  : --
  ---------------------------------------------------------------------------*/
void i2c_init(void)
{
    I2C_CR1    = 0;     // Disable I2C before configuration starts
    I2C_FREQR  = 16;    // Set the internal clock frequency to 16 MHz
    I2C_CCRH   = 0x00;  // I2C running is standard mode.
    I2C_CCRL   = 0x50;  // SCL clock speed is 100 kHz.
    I2C_OARH   = 0x40;  // 7 bit address mode.
    I2C_OARL   = 0x00;  // Clear the address registers
    I2C_TRISER = 17;    // Set SCL rise-time to 1000 nsec.
    I2C_ITR    = 0x00;  // Disable I2C interrupts
	I2C_CR1    = 1;     // Configuration complete so turn the peripheral on
} // i2c_init()

/*-----------------------------------------------------------------------------
  Purpose  : This function checks the ACK bit after an address has been sent.
             If a device is not present, a NACK (1) is returned.
  Variables: *err: true= I2C bus locked up, no response
  Returns  : --
  ---------------------------------------------------------------------------*/
uint8_t recv_ack_bit(void)
{
	uint8_t reg, ack, nack;
	uint8_t try = 0;

  do {
	ack  = I2C_SR1_ADDR;
	nack = I2C_SR2_AF;
	i2c_delay();
  } while (!ack && !nack && (++try < I2C_RETRIES));
  if (try >= I2C_RETRIES)
	  return I2C_ERROR;
  else if (ack)
  {   // ACK bit received
      reg = I2C_SR1; // Clear ADDR bit by reading I2C_SR1 and I2C_SR3
      reg = I2C_SR3;
      return I2C_ACK;
  } // if
  else
  {   // Acknowledge Failure
      I2C_SR2_AF = 0; // clear error bit
      return I2C_NACK;
  } // else
} // recv_ack_bit()

/*-----------------------------------------------------------------------------
  Purpose  : This function issues a start condition and sends address and 
             transfer direction.
  Variables: -
  Returns  : I2C_ACK, I2C_NACK, I2C_ERROR
  ---------------------------------------------------------------------------*/
uint8_t i2c_start(uint8_t address)
{
  I2C_CR2_START = 1;  // Generate Start condition
  while (!I2C_SR1_SB) i2c_delay(); // Wait until Start is sent
  I2C_DR   = address; // Send the slave address and the R/W bit
  return recv_ack_bit();
} // i2c_start()

/*-----------------------------------------------------------------------------
  Purpose  : This function issues a repeated-start condition and sends 
             address and transfer direction.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void i2c_rep_start(uint8_t address)
{
    uint8_t reg;
    
    I2C_CR2_START = 1; // Generate Repeated Start condition
    while (!I2C_SR1_SB) i2c_delay(); // Wait until Start is sent
    I2C_DR = address;         // Send the slave address and the R/W bit
    while (!I2C_SR1_ADDR) i2c_delay();
    reg = I2C_SR3;           // clear ADDR bit
} // i2c_rep_start()

/*-----------------------------------------------------------------------------
  Purpose  : This function issues a stop condition
  Variables: --
  Returns  : --
  ---------------------------------------------------------------------------*/
void i2c_stop(void)
{
    __disable_interrupt();           // Errata workaround (Disable interrupt)
    I2C_CR2_STOP = 1;                // generate stop here (STOP=1)
    __enable_interrupt();	         // Errata workaround (Enable interrupt)
    while (I2C_SR3_MSL) i2c_delay(); // wait until stop is performed
} // i2c_stop()

/*-----------------------------------------------------------------------------
  Purpose  : This function sends one byte to I2C device
  Variables: data: byte to be transferred
  Returns  : I2C_ACK : write successful
             I2C_NACK: write failed
  ---------------------------------------------------------------------------*/
void i2c_write(uint8_t data)
{
	I2C_DR = data;                    // send byte over I2C bus
	while (!I2C_SR1_TXE) i2c_delay(); // wait until Data Register is empty
} // i2c_write()

/*-----------------------------------------------------------------------------
  Purpose  : This function reads one byte from the I2C device.
             This is the Lujji version of i2c_read().
  Variables: -
  Returns  : byte read
  ---------------------------------------------------------------------------*/
uint8_t i2c_read1(void)
{
    I2C_CR2_ACK = 0;
    i2c_stop();
    while (!I2C_SR1_RXNE) ;
    return I2C_DR;
} // i2c_read1()

/*-----------------------------------------------------------------------------
  Purpose  : This function reads two or more bytes from the I2C device
             This is the Lujji version of i2c_readN()
  Variables: *buf : pointer to array to store data in
	     len  : #bytes to read from the I2C device
  Returns  : byte read from the I2C device
  ---------------------------------------------------------------------------*/
void i2c_readN(uint8_t *buf, uint8_t len)
{
    while (len-- > 1) 
    {
        I2C_CR2_ACK = 1;
        while (!I2C_SR1_RXNE) ;
        *(buf++) = I2C_DR;
    } // while
    *buf = i2c_read1();
} // i2c_readN()
