/*==================================================================
  File Name    : i2c_ds3231.c
  Author       : Emile
  ------------------------------------------------------------------
  Purpose : This files contains the DS3231 related functions.
            The DS3231 is a Real-Time Clock (RTC).
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
  along with this software. If not, see <http://www.gnu.org/licenses/>.
  ==================================================================
*/ 
#include "i2c.h"
#include "i2c_ds3231.h"

bool ds3231_read_register(uint8_t reg, uint8_t *value)
{
	bool err;

	err = (i2c_start(DS3231_ADR | I2C_WRITE) != I2C_ACK); // generate I2C start + output address to I2C bus
	if (!err) 
        { 
            i2c_write(reg);       // write register address to read from
            i2c_rep_start(DS3231_ADR | I2C_READ);
            *value = i2c_read1(); // Read register, generate I2C stop condition
        } // if
	return err;
} // ds3231_read_register()
 
bool ds3231_write_register(uint8_t reg, uint8_t value)
{
	bool err;

	err = (i2c_start(DS3231_ADR | I2C_WRITE) != I2C_ACK); // generate I2C start + output address to I2C bus
	if (!err) 
        {
            i2c_write(reg);   // write register address to write to
            i2c_write(value); // write value into register
            i2c_stop();       // close I2C bus
        } // if
	return err;
} // ds3231_write_register()

uint8_t	ds3231_decode(uint8_t value)
{
	uint8_t decoded = value & 0x7F;
	decoded = (decoded & 0x0F) + 10 * ((decoded & (0xF0)) >> 4);
	return decoded;
} // ds3231_decode()

uint8_t ds3231_decodeH(uint8_t value)
{
  if (value & 0x40) // 12 hour format
    value = (value & 0x0F) + ((value & 0x20) ? 10 : 0);
  else // 24 hour format
    value = (value & 0x0F) + (5 * ((value & 0x30) >> 3));
  return value;
} // ds3231_decodeH()

uint8_t	ds3231_decodeY(uint8_t value)
{
	uint8_t decoded = (value & 0x0F) + 10 * ((value & 0xF0) >> 4);
	return decoded;
} // ds3231_decodeY()

uint8_t ds3231_encode(uint8_t value)
{
	uint8_t encoded = ((value / 10) << 4) + (value % 10);
	return encoded;
} // ds3231_encode()

bool ds3231_gettime(Time *p)
{
	bool err;
	uint8_t buf[8];

	err = (i2c_start(DS3231_ADR | I2C_WRITE) != I2C_ACK); // generate I2C start + output address to I2C bus
	if (!err) 
        {
            i2c_write(REG_SEC); // seconds register is first register to read
            i2c_rep_start(DS3231_ADR | I2C_READ);
            
            i2c_readN(buf,7);                 // Read bytes and call i2c_stop()
            p->sec  = ds3231_decode(buf[0]);  // Read SECONDS register
            p->min  = ds3231_decode(buf[1]);  // Read MINUTES register
            p->hour = ds3231_decodeH(buf[2]); // Read HOURS register
            p->dow  = buf[3];                 // Read DOW register
            p->date = ds3231_decode(buf[4]);  // Read DAY register
            p->mon  = ds3231_decode(buf[5]);  // Read MONTH register
            p->year = 2000 + ds3231_decodeY(buf[6]); // Read YEAR register
	} // if
	else 
	{   // in case of error
		p->sec = p->min  = p->hour = p->year = 0; 
		p->dow = p->date = p->mon  = 1;
	} // else	
	return err;
} // ds3231_gettime()

void ds3231_settime(uint8_t hour, uint8_t min, uint8_t sec)
{
	if ((hour < 24) && (min < 60) && (sec < 60))
	{
		ds3231_write_register(REG_HOUR, ds3231_encode(hour));
		ds3231_write_register(REG_MIN , ds3231_encode(min));
		ds3231_write_register(REG_SEC , ds3231_encode(sec));
	} // if	
} // ds3231_settime()

// 1=Monday, 2=Tuesday, 3=Wednesday, 4=Thursday, 5=Friday, 6=Saturday, 7=Sunday
uint8_t ds3231_calc_dow(uint8_t date, uint8_t mon, uint16_t year)
{
	uint32_t JND = date + ((153L * (mon + 12 * ((14 - mon) / 12) - 3) + 2) / 5)
	+ (365L * (year + 4800L - ((14 - mon) / 12)))
	+ ((year + 4800L - ((14 - mon) / 12)) / 4)
	- ((year + 4800L - ((14 - mon) / 12)) / 100)
	+ ((year + 4800L - ((14 - mon) / 12)) / 400)
	- 32044L;
        JND = JND % 7;
        if (JND == 0) JND = 7;
	return JND;
} // ds3231_calc_dow()

void ds3231_setdate(uint8_t date, uint8_t mon, uint16_t year)
{
	uint8_t dow;
	
	if (((date > 0) && (date <= 31)) && ((mon > 0) && (mon <= 12)) && ((year >= 2000) && (year < 3000)))
	{
		dow = ds3231_calc_dow(date, mon, year);
		ds3231_write_register(REG_DOW,dow);
		year -= 2000;
		ds3231_write_register(REG_YEAR, ds3231_encode(year));
		ds3231_write_register(REG_MON , ds3231_encode(mon));
		ds3231_write_register(REG_DATE, ds3231_encode(date));
	} // if
} // ds3231_setdate()

void ds3231_setdow(uint8_t dow)
{
	if ((dow > 0) && (dow < 8))
		ds3231_write_register(REG_DOW, dow);
} // ds3231_setdow() 

// Returns the Temperature in a Q8.2 format
int16_t ds3231_gettemp(void)
{
	bool err;
	uint8_t msb,lsb;
	int16_t retv;
	
	err    = ds3231_read_register(REG_TEMPM, &msb);
	retv   = msb;
	retv <<= 2; // SHL 2
	if (!err) err = ds3231_read_register(REG_TEMPL, &lsb);
	retv  |= (lsb >> 6);
	if (retv & 0x0200)
	{   // sign-bit is set
		retv &= ~0x0200; // clear sign bit
		retv = -retv;    // 2-complement
	} // if
	if (!err) return retv;
	else      return 0;
} // ds3231_gettemp()
