/******************************************************************************
* File Name          : binstuff.c
* Date First Issued  : 07/05/2019
* Description        : uart binary lines with byte stuffing
*******************************************************************************/

#include <stdint.h>

/* 
The idea was triggered from--
https://en.m.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing

There are three readings, uint16_t readings to be sent on each line. Six binary bytes.

The uart routines convert incoming uart to lines based on <cr>.  Therefore, 
any reading byte that is >cr> has to be stuffed.

Rather some variable length scheme, I came up with the following--

The first byte has bits that correspond to the following six bytes 
that were originally 0x0D and substituted with 0x5A.  If the bit is 
off a 0x5A stands as received, and it the bit is on, the 0x5A 
(or whatever, except 0x0D) is replaced with 0x0D.

Since the first byte with the bits could also be 0x0D, the high order 
bit is always on so that it will never be 0x0D.

That still leaves a single bit that can be used as a one bit checksum.

If the message is sent using ascii-hex, 13 chars are needed (and no checksum, 
but ascii-hex checking would one type of check).  If it is sent with the above 
scheme, eight.

Msg: A <cr> terminated line with three readings in uint16_t form.
 byte[0] bit map of reading byte stuffed
 byte[1] Low  order byte of 1st reading
 byte[2] High order byte of 1st reading
 byte[3] Low  order byte of 2nd reading
 byte[4] High order byte of 2nd reading
 byte[5] Low  order byte of 3rd reading
 byte[6] High order byte of 3rd reading
 byte[7] line termination <cr> 0x0D

Example:
Stuff byte = 0x5A
Reading 1: 0x120D
Reading 2: 0x0D5A
Reading 3: 0x0D0D

Send reading, low order byte first

Example:
byte[0] 0bxx100111 (bit order: bit5 corresponds to byte[1])
byte[1] 0x5A 0 => 0x0D |
byte[2] 0x12 1 => 0x12 | 0x120D
byte[3] 0x5A 0 => 0x5A | 
byte[4] 0x5A 1 => 0x0D | 0x0D5A
byte[5] 0x5A 1 => 0x0D |
byte[6] 0x5A 1 => 0x0D | 0x0D0D
byte[7] 0x0D line terminator

xx = two bit checksum on readings
*/
#define BYTESTUFF 0x5A
#define CR 0x0D

/* *************************************************************************
 * int binstuff_decode(uint16_t *p, uint8_t* pin);
 *	@brief	: Convert incoming binary line to uint16_t array
 * @param	: p = Pointer to output 
 * @param	: pin = Pointer to input binary chars
 * @return	:  0 = success, -1 checksum failed
 * *************************************************************************/
int binstuff_decode(uint16_t *p, uint8_t* pin)
{
	uint32_t tmp;
	uint32_t chksum = 0;		
	uint16_t *pend = (p + 3);
	uint8_t chkbit = (*pin >> 7);  // Save checksum
	 int8_t bits   = (*pin << 2);  // Position 1st byte bit in bit 8

	while(p < pend)
	{
		/* High order byte. */
		if (bits < 0) // Stuffed byte?
 			*p = 0x0D; // Yes, substitute
		else
			*p = *(pin+1); // No. No change

		chksum += (*p & 0xff);
		*p  = (*p << 8);    // Position high byte
   	bits = (bits << 1);	

		/* Low order byte. */	
		if (bits < 0)
 			*p |= 0x0D;
		else
			*p |= *pin;

		chksum += (*p & 0xff);
   	bits = (bits << 1);	
	
		pin += 2; // Next input byte
		p   += 1; // Step to next output array position
	}

	if ((chksum ^ !0x1) != chkbit)
	{
		return -1;	
	}
	return 0;
}
