/******************************************************************************
* File Name          : binstuff2.c
* Date First Issued  : 07/06/2019
* Description        : uart binary lines with byte stuffing
*******************************************************************************/

#include <stdint.h>

/*
https://en.m.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
*/
#define CR 0x0D

/* *************************************************************************
 * uint8_t binstuff2_decode(uint8_t* pin);
 *	@brief	: Unstuff CR terminated line, in place (WARNING: line must have CR)
 * @param	: pin = Pointer to input binary chars
 * @return	: length of data (including prefix byte)
 * *************************************************************************/
 uint8_t binstuff2_decode(uint8_t* pin)
{
	uint8_t* pbegin = pin;
	uint8_t* pnxt = (pin + (*pin ^ CR));
	
	pin += 1;
	while (*pin != CR)
	{
		if (pnxt == pin) 
		{
			pnxt += (*pnxt ^ CR);
			*pin = CR;
		}
		pin += 1;
	}
	return (pin -pbegin); // Return length
}


