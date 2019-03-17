/*==================================================================
   File Name    : i2c.h
   Author       : Emile
  ------------------------------------------------------------------
  Purpose : This files contains the I2C related functions 
            for the STM8 uC.
            It is meant for the STC1000 thermostat hardware WR-032.
  ------------------------------------------------------------------
  STC1000+ is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  STC1000+ is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with STC1000+.  If not, see <http://www.gnu.org/licenses/>.
  ================================================================== */ 
#ifndef _I2C_H
#define _I2C_H   1

#include <iostm8s103f3.h>
#include <stdbool.h>
#include <intrinsics.h> 
#include <stdint.h>
#include "delay.h"
      
// Defines when bit-banging is used instead of I2C-peripheral
#define I2C_SDA   (0x20) /* PB5 */
#define I2C_SCL   (0x10) /* PB4 */
      
#define SDA_in    (PB_DDR &= ~I2C_SDA)
#define SDA_out   (PB_DDR |=  I2C_SDA)
#define SDA1      (PB_ODR |=  I2C_SDA)
#define SDA0      (PB_ODR &= ~I2C_SDA)
#define SCL1      {PB_ODR |=  I2C_SCL; i2c_delay();}
#define SCL0      {PB_ODR &= ~I2C_SCL; i2c_delay();}

/* defines the data direction (reading from I2C device) in i2c_start(), i2c_rep_start() */
#define I2C_ACK     (0)
#define I2C_NACK    (1)
#define I2C_WRITE   (0)
#define I2C_READ    (1)
#define I2C_RETRIES (3)

//-----------------------------------------------------------------
// The LM92 sign bit is normally bit 12. The value read from the
// LM92 is SHL3. Therefore the sign bit is at bit 15
// Same for the Full Scale value, normally 2^12 SHL3 = 2^15.
//-----------------------------------------------------------------
#define LM92_ADDR      (0x90)
#define LM92_SIGNb     (0x8000)
#define LM92_FS        (32768)
#define LM92_ERR       (0x4000)

//-------------------------------------------------------------------------
// MCP23008 8-BIT  IO Expander
//-------------------------------------------------------------------------
// Bank addressing, seq. operation disabled, slew rate enabled
// HW addressing enabled
#define MCP23008_INIT (0x2A)
#define MCP23008_ADDR (0x40)

// Defines for the MCP23008
#define IODIR   (0x00)
#define IPOL    (0x01)
#define GPINTEN (0x02)
#define DEFVAL  (0x03)
#define INTCON  (0x04)
#define IOCON   (0x05)
#define GPPU    (0x06)
#define INTF    (0x07)
#define INTCAP  (0x08)
#define GPIO    (0x09)
#define OLAT    (0x0A)

//-------------------------------------------------------------------------
// DS2482 Real-Time Clock (RTC)
//-------------------------------------------------------------------------
// DS2482 Configuration Register
// Standard speed (1WS==0), Strong Pullup disabled (SPU==0), Active Pullup enabled (APU==1)
#define DS2482_ADDR          (0x30)
#define DS2482_CONFIG        (0xE1)
#define DS2482_OW_POLL_LIMIT  (200)

// DS2482 commands
#define CMD_DRST   (0xF0)
#define CMD_WCFG   (0xD2)
#define CMD_CHSL   (0xC3)
#define CMD_SRP    (0xE1)
#define CMD_1WRS   (0xB4)
#define CMD_1WWB   (0xA5)
#define CMD_1WRB   (0x96)
#define CMD_1WSB   (0x87)
#define CMD_1WT    (0x78)

// DS2482 status bits 
#define STATUS_1WB  (0x01)
#define STATUS_PPD  (0x02)
#define STATUS_SD   (0x04)
#define STATUS_LL   (0x08)
#define STATUS_RST  (0x10)
#define STATUS_SBR  (0x20)
#define STATUS_TSB  (0x40)
#define STATUS_DIR  (0x80)

// DS3231 DOW defines
#define MONDAY		(1)
#define TUESDAY		(2)
#define WEDNESDAY	(3)
#define THURSDAY	(4)
#define FRIDAY		(5)
#define SATURDAY	(6)
#define SUNDAY		(7)

#define SQW_RATE_1	(0)
#define SQW_RATE_1K	(1)
#define SQW_RATE_4K	(2)
#define SQW_RATE_8K	(3)

#define OUTPUT_SQW	(0)
#define OUTPUT_INT	(1)

typedef struct _Time
{
    uint8_t	hour;
    uint8_t	min;
    uint8_t	sec;
    uint8_t	date;
    uint8_t	mon;
    uint16_t	year;
    uint8_t	dow;
} Time;

#define DS3231_ADR      (0xD0)

#define REG_SEC		(0x00)
#define REG_MIN		(0x01)
#define REG_HOUR	(0x02)
#define REG_DOW		(0x03)
#define REG_DATE	(0x04)
#define REG_MON		(0x05)
#define REG_YEAR	(0x06)
#define REG_CON		(0x0e)
#define REG_STATUS      (0x0f)
#define REG_AGING	(0x10)
#define REG_TEMPM	(0x11)
#define REG_TEMPL	(0x12)

// Bit-banging routines for I2C
void    i2c_delay(void);
uint8_t i2c_write_bb(uint8_t data);
uint8_t i2c_read_bb(uint8_t ack);
uint8_t i2c_start_bb(uint8_t address);
uint8_t i2c_rep_start_bb(uint8_t address);
void    i2c_stop_bb(void);
// I2C-peripheral routines
void    i2c_init(bool bb);           // Initializes the I2C Interface. Needs to be called only once
uint8_t i2c_start(uint8_t addr);     // Issues a start condition and sends address and transfer direction
void    i2c_rep_start(uint8_t addr); // Issues a repeated start condition and sends address and transfer direction
void    i2c_stop(void);              // Terminates the data transfer and releases the I2C bus
void    i2c_write(uint8_t data);     // Send one byte to I2C device
uint8_t i2c_read(uint8_t ack);       // Read one byte from I2C device
void    i2c_readN(uint8_t num_bytes, uint8_t *buf); // read byte(s) from the I2C device

int16_t lm92_read(uint8_t addr, uint8_t *err);

// Function prototypes for MCP23008
uint8_t mcp23008_init(void);
uint8_t mcp230xx_read(uint8_t reg);
uint8_t mcp230xx_write(uint8_t reg, uint8_t data);

// Function prototypes for DS2482
int8_t  ds2482_reset(uint8_t addr);
int8_t  ds2482_reset_bb(uint8_t addr);
int8_t  ds2482_write_config(uint8_t addr);
int8_t  ds2482_detect(uint8_t addr);
int8_t  ds2482_detect_bb(uint8_t addr);
uint8_t ds2482_search_triplet(uint8_t search_direction, uint8_t addr);

// Function prototypes for DS3231
bool    ds3231_gettime(Time *p);
void    ds3231_settime(uint8_t hour, uint8_t min, uint8_t sec);
uint8_t ds3231_calc_dow(uint8_t date, uint8_t mon, uint16_t year);
void    ds3231_setdate(uint8_t date, uint8_t mon, uint16_t year);
void    ds3231_setdow(uint8_t dow);
int16_t ds3231_gettemp(void);

#endif
