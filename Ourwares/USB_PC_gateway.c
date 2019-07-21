/******************************************************************************
* File Name          : USB_PC_gateway.c
* Date First Issued  : 10/16/2013
* Board              : Discovery F4
* Description        : PC<->gateway 
*******************************************************************************/
/*
10-04-2013 - revised so that 'PC_gateway_comm.[ch]' has routines common to PC and stm32,
   and this routine is USART1 specific.
10-16-2013 - copy and revised for USB instead of USART1
*/

#include <fcntl.h>
#include <unistd.h>


#include "USB_PC_gateway.h"
#include "PC_gateway_comm.h"
#include <stdio.h>
#include <string.h>
//#include "xprintf.h"

/* Local buffer for extracting msgs/lines from incoming data.  This allows 'read' to
   attempt to retrieve more than just one byte. */
#define GETLOCALBUFSZ	256		// Size of local buffer for incoming bytes
static char localbuf[GETLOCALBUFSZ];	// 
static char* plocalbuf = &localbuf[0];	// Pointer into local buffer
static int localct = 0;			// Byte count of data remaining in buffer

u32 debug_inct;

/* **************************************************************************************
 * int USB_PC_get_msg_mode(int fd, struct PCTOGATEWAY* ptr, struct CANRCVBUF* pcan);
 * @brief	: Build message from PC, various modes modes
 * @param	: fd = file descriptor
 * @param	: ptr = Pointer to msg buffer (see common_can.h)
 *          : ptr->mode_link = selection of PC<->gateway mode and format (binary, ascii,...)
 *          : return: ptr->c[] = binary msg, possibly compressed
 *          : return: ptr->asc[] = asc line
 *          : return: ptr->ct; ptr->ctasc; counts for above, repsectively.
 *          : return: ptr->seq; sequence number extracted., (if applicable to mode)
 * @param	: pcan = pointer to struct with CAN msg (stm32 register format)
 * @return	:  1 = completed; ptr->ct hold byte count
 *          :  0 = msg not ready; 
 *    : If negative = Some error--
 *          : -1 = completed, but bad checksum
 *          : -2 = completed, but too few bytes to be a valid CAN msg
 *          : -3 = bad mode selection code
 *          : -4 = 'read' returned an error
 *    : If message completion has no errors, but following compression does-- 
 *          : -5 = Too few bytes to a valid 29b compressed msg
 *          : -6 = dlc: payload ct too large (> 8) in a 29 bit id msg
 *          : -7 = dlc doesn't match byte count in a 29 bit id msg
 *          : -8 = Too few bytes to a valid 11b compressed msg
 *          : -9 = dlc: payload ct too large (> 8) in a 11 bit id msg
 *          : -10 = dlc doesn't match byte count in a 11 bit id msg
 * ************************************************************************************** */
/* @param	: ptr->mode_link: mode selection
 *              : 0 = binary-- seq byte, data, chksum byte, '\n' framing byte
 *              : 1 = ascii-- mode 0 converted to ascii/hex, with '\n' new line framing
 *              : ... other modes in the future?
*/

int USB_PC_get_msg_mode(int fd, struct PCTOGATEWAY* ptr, struct CANRCVBUF* pcan )
{
	int retstatus;
	int temp = 0;
	char c;

	if (localct <= 0) // Is the buffer empty?
	{ // Here yes.
		/* Try to replenish buffer if it is empty. */
		localct = read(fd, localbuf, GETLOCALBUFSZ); // Seek a fist-full, or more, bytes
		if (localct == 0) return  0;	// But, if alas, there were none, return empty handed :(
		if (localct  < 0) return -4;	// Ooops!

		plocalbuf = &localbuf[0];	// Reset pointer for removing buffer bytes.
debug_inct += localct; // Debug: running ct of bytes
	}

	switch (ptr->mode_link)	// To use 'switch' for just two cases is lame, but allows for easy addition of more modes.
	{
	case 0:	// BINARY Mode (byte stuffing/frame byte format) 
		while (localct > 0)	// Onward through those bytes!
		{
			localct -= 1;		// Like a pacman, take a byte
			c = *plocalbuf++;	//   and move one step.
	
			if ( (retstatus = PC_msg_get(ptr,c)) != 0) // Did this byte complete a msg?
			{ // Here, either a good msg, or an error such as chksum or too many/few bytes
				if (retstatus >= 1)
					temp = CANuncompress(pcan, &ptr->cmprs); 
				if (temp < 0) return temp -= 4;
				return retstatus; // Note: there maybe be unused bytes still in the buffer.
			}		
		}
		break;

	case 1: // ASCII Mode (ascii/hex only with '\n' newline)
		while (localct > 0)	// Rattle through those chars!
		{
			localct -= 1;		// Count chars
			c = *plocalbuf++;	

			if ( (retstatus = PC_msg_getASCII(ptr, c)) != 0) // Did this char complete a msg?
			{ // Here, either a good line, or an error, such as too many or few chars, checksum err, or odd number char pairs
				if (retstatus >= 1)
					temp = CANuncompress(pcan, &ptr->cmprs); 
				if (temp < 0) return temp -= 4;
				return retstatus; // Note: there maybe be unused bytes still in the buffer.
			}		
		}
		break;
		
	case 2: // ASCII Mode (Gonzaga project: minimal compression)
		while (localct > 0)	// Rattle through those chars!
		{
			localct -= 1;		// Count chars
			c = *plocalbuf++;
//xprintf(6,"%c",c);

			if ( (retstatus = PC_msg_getASCII(ptr, c)) != 0) // Did this char complete a msg?
			{ // Here, either a good line, or an error, such as too many or few chars, checksum err, or odd number char pairs
				if (retstatus >= 1)
					temp = CANuncompress_G(pcan, &ptr->cmprs); 
				if (temp < 0) return temp -= 4;
				return retstatus; // Note: there maybe be unused bytes still in the buffer.
			}		

		}
		break;

	default:	return -3; // The bozo calling this routine failed miserably setting up the struct.
		break;	
	}

	return 0; // No more bytes to work with, AND msg is not complete...sorry, maybe next time.
}
/* **************************************************************************************
 * int USB_toPC_msgASCII(struct SERIALSENDTASKBCB** ppbcb, struct PCTOGATECOMPRESSED* p);
 * @brief	: Send msg to PC after converting binary msg to ASCII/HEX 
 * @param	: pbufy = Pointer buffer control block w buffer and uart handle
 * @param	: p = Pointer to struct with bytes to send to PC
 * @return	: count of bytes written
 * ************************************************************************************** */
int USB_toPC_msgASCII(struct SERIALSENDTASKBCB** ppbcb, struct PCTOGATECOMPRESSED* p)
{
	int sz;
	/* Convert binary to ascii, and prepare with sequence number at beginning, plus checksum and newline termination */
//	pbufy->pbcb->size = PC_msg_prepASCII(&b[0], BUFFSIZE, p);
	sz = PC_msg_prepASCII(ppbcb, p);	
	return	sz;
}
/* **************************************************************************************
 * int USB_toPC_msgBIN(struct SERIALSENDTASKBCB** ppbcb, struct PCTOGATECOMPRESSED* p);
 * @brief	: Send msg to PC in the binary format
 * @brief	: pbufy = Pointer buffer control block w buffer and uart handle
 * @param	: p = Pointer to struct with bytes to send to PC
 * @return	: count of bytes written
 * ************************************************************************************** */
int USB_toPC_msgBIN(struct SERIALSENDTASKBCB** ppbcb, struct PCTOGATECOMPRESSED* p)
{
	struct SERIALSENDTASKBCB* pbcb = *ppbcb;
	#define BUFFSIZE (PCTOGATEWAYSIZE*2+4)
	u8 b[BUFFSIZE];		// Sufficiently large output buffer

	/* Make an array with seq number, CAN msg. */
	if (p->ct >= PCTOGATEWAYSIZE) return -1; 	// Prevent overruns
	/* Prepare msg for sending.  Add byte stuffing, framing, and checksum to input bytes. */
	pbcb->size = PC_msg_prep(&b[0], BUFFSIZE, &p->cm[0], p->ct);

	/* Send to USB, or other output per file descriptor. */
	return yprintf(ppbcb,"%s",b);
}
/* **************************************************************************************
 * int USB_toPC_msg_mode(struct SERIALSENDTASKBCB** ppbcb, struct PCTOGATEWAY* ptr, struct CANRCVBUF* pcan);
 * @brief	: Send msg to PC in selected mode
 * @brief	: ppbcb = pointer to pointer to buffer control block
 * @param	: pcan = pointer to struct with CAN msg (stm32 register format)
 * @param	: ptr = Pointer to msg buffer
 *              : ptr->mode_link = selection of PC<->gateway mode and format (binary, ascii,...)
 *              : ptr->mode_send = CAN msg (from calling routine) is binary or ascii
 *              : ptr->c[] = binary msg, possibly compressed
 *              : ptr->asc[] = asc line
 *              : ptr->ct; ptr->ctasc; counts for above, repsectively.
 *              : ptr->seq; sequence number to be sent., (if applicable to mode)
 * @return	: postive = number of bytes written
 *              : negative = error
 *              : -1 = The bozo that called this routine gave us booogus ptr->mode_link|send!
 * ************************************************************************************** */
int USB_toPC_msg_mode(struct SERIALSENDTASKBCB** ppbcb, struct PCTOGATEWAY* ptr, struct CANRCVBUF* pcan)
{
//	struct SERIALSENDTASKBCB* pbcb = *ppbcb;
	switch (ptr->mode_link)	// 'switch' for just two cases is lame, but allow for many more modes.
	{
	case 0:	// BINARY Mode: PC<->gateway
		CANcompress(&ptr->cmprs, pcan);		// Compress
		// TODO
		break; // JIC

	case 1: // ASCII/HEX mode: PC<->gateway (same "strong" compression as case 0 above)
		CANcompress(&ptr->cmprs, pcan);		// Heavy compression
		return USB_toPC_msgASCII(ppbcb, &ptr->cmprs);
		break; // JIC

	case 2: // ASCII/HEX Gonzaga format (minimal compression)
		CANcompress_G(&ptr->cmprs, pcan);	// dlc = one byte the only compression
		return USB_toPC_msgASCII(ppbcb, &ptr->cmprs);
		break;
	}
	return -1;
}
/* **************************************************************************************
 * int USB_toPC_msg_asciican(struct SERIALSENDTASKBCB** ppbcb, char* pin, struct PCTOGATEWAY* ptr);
 * @brief	: I have a CAN msg in ASCII/HEX (no seq, no chksum).  Send in selected mode.
 * *param	: pbufy = pointer to buffer control block w buffer for uart
 * *param	: ptr = pointer to stuff used in conversions
 * @return	: postive = number of bytes written
 *              : negative = error
 *              : -1 = The bozo that called this routine gave us booogus ptr->mode_link!
 * @NOTE	: Be sure ptr->mode_link is set!
 * ************************************************************************************** */
int USB_toPC_msg_asciican(struct SERIALSENDTASKBCB** ppbcb, char* pin, struct PCTOGATEWAY* ptr)
{
	int ret;
	struct CANRCVBUF can;

	/* Convert msg to binary */
	ret = PC_msg_asctobin(ptr, pin);
	if (ret < 0) return ret;


	/* Handle binary msg no differently than others */
	ptr->cmprs.seq = ptr->seq;		// Add sequence number (for PC checking for missing msgs)
	
	/* Send as we would any msg that is in binary form. */
	return USB_toPC_msg_mode(ppbcb, ptr, &can);
}


