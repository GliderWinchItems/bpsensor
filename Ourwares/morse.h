/******************************************************************************
* File Name          : morse.h
* Date First Issued  : 02/13/2019
* Description        : Morse code
*******************************************************************************/

#ifndef __MORSE
#define __MORSE

/* ************************************************************************* */
void morse_string(char* p);
/*	@brief	: Send a character string as Morse code
 * @param	: p = pointer to string
 * *************************************************************************/
void morse_number(uint32_t n);
/*	@brief	: Send a character string as Morse code
 * @param	: nx = number to send
 * *************************************************************************/
void morse_trap(uint16_t x);
/*	@brief	: Disable interrupts, Send 'x' and endless loop
 * @param	: x = trap number to flash
 * *************************************************************************/
void morse_hex(uint32_t n);
/*	@brief	: Send a  hex number, skip leading zeroes
 * @param	: nx = number to send
 * *************************************************************************/

#endif

