/******************************************************************************
* File Name          : PC_gateway_comm.c
* Date First Issued  : 10/04/2013
* Board              : Not specific to PC or stm32
* Description        : PC<->gateway 
*******************************************************************************/
#include "PC_gateway_comm.h"
#include <stdio.h>

static void strwrd(u8* pout, u32 x);

/* Mostly for debugging, and hardware testing. */
static u32 PC_chksum_ct_err;	// Count checksum errors
static u32 PC_toofew_ct_err;	// Count msg frames with too few bytes
static u32 PC_oddeven_ct_err;	// Count odd/even errors on ascii/hex input
static u32 PC_toomany_ct_err;	// Count msg frames with too many bytes


/* **************************************************************************************
 * u8 CANgenchksum(u8* p, int ct);
 * @brief	: Generate a one byte checksum
 * @param	: p = pointer to array to be checksummed
 * @param	: ct = number of bytes to be checksummed
 * @return	: Checksum
 * ************************************************************************************** */
u8 CANgenchksum(u8* p, int ct)
{
	int i = 0;
	u32 x = CHECKSUM_INITIAL;
	for (i = 0; i < ct; i++)
		x += *p++;
	x += (x >> 16);	// Add carries into high half word
	x += (x >> 16);	// Add carry if previous add generated a carry
	x += (x >> 8);  // Add high byte of low half word
	x += (x >> 8);  // Add carry if previous add generated a carry
	return (u8)x;
}
/* **************************************************************************************
 * static char* hex(char *p, u8 c)	// Convert 'c' to hex, placing in output *p.
 * ************************************************************************************** */
/* bin to ascii lookup table */
static const char h[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
static char* hex(char *p, u8 c)	// Convert 'c' to hex, placing in output *p.
{
		*p++ = h[((c >> 4) & 0x0f)];	// Hi order nibble
		*p++ = h[(c & 0x0f)];		// Lo order nibble
		return p;			// Return new output pointer position
}
/* **************************************************************************************
 * int PC_msg_get(struct PCTOGATEWAY* ptr, u8 c);
 * @brief	: Build message from incoming BINARY bytes.  Binary data including seq number goes
 *              : int ptr.cmprs struct.  ASCII data goes into ptr->asc[]
 * @param	: ptr = Pointer to msg buffer (see common_can.h)
 *		:  ptr->cmprs.cm[] = binary bytes (including seq number, but not checksum)
 *		:  ptr->cmprs.ct = count of binary bytes (including seq number, but not checksum)
 *		:  ptr->asc[] = ascii/hex bytes as received, including '\n' plus a '\0'
 *		:  ptr->ctasc = count of ascii chars = (2* binary ct + 1);
 * @param	: c = byte to add msg being built
 * @return	:  1 = completed; ptr->cmprs.ct holds byte count of binary data
 *              :  0 = msg not ready; 
 *              : -1 = completed, but bad checksum
 *  		: -2 = completed, but too few bytes to be a valid CAN msg
 *  		: -3 = completed, but too many bytes to be a valid CAN msg
 * ************************************************************************************** */
/* Note: It is up to the caller to have the struct initialized, initially and after the 
message has been "consumed."  When there are errors this routine re-initializes.  */

static void strstuff(struct PCTOGATEWAY* ptr, u8 c)
{
	*ptr->cmprs.p++ = c;			// Save binary byte in binary byte array
	if (ptr->cmprs.ct > 0)			// Skip storing ascii for sequence number
		ptr->pasc = hex(ptr->pasc, c);	// Convert 'c' to hex and save in ascii array	
	ptr->cmprs.ct += 1;			// Binary byte count
	return;
}

int PC_msg_get(struct PCTOGATEWAY* ptr, u8 c)
{			
	int i;

	switch (c)
	{
	case CAN_PC_FRAMEBOUNDARY:	// Possible end of message
		if (ptr->prev == CAN_PC_ESCAPE)
		{ // Here, previous byte was an escape byte
			strstuff(ptr, c); // Store binary, ascii, and count binary bytes
		}
		else
		{ // Here, frame without preceding escape means End of Message
			i = ptr->cmprs.p - &ptr->cmprs.cm[0] - 1; // Number of bytes received in this frame less chksum
			ptr->cmprs.ct = i;	// Save for others
			if (i < 3)		// Too few bytes to comprise a valid msg?
			{ // Here yes. (min binary msg plus checksum = 3 bytes)
				PC_msg_initg(ptr);	// Initialize struct for the next message
				PC_toofew_ct_err += 1;	// Running count of this type of error.
				return -2;	// Return error code.
			}

			if (i >= (PCTOGATEWAYSIZE/2) )	// Too many bytes to comprise a valid msg?
			{ // Here yes.
				PC_msg_initg(ptr);	// Initialize struct for the next message
				PC_toomany_ct_err += 1;	// Running count of this type of error.
				return -3;	// Return error code.
			}

			/* Check checksum. */	
			if ( (CANgenchksum(&ptr->cmprs.cm[0], i)) == ptr->cmprs.cm[i])
			{ // Here checksum good
/* If incoming, complete & valid, messages are to be buffered, this is where the index for the
  buffer array would be advanced. */
				ptr->ctasc = (ptr->cmprs.ct * 2) + 1; // Compute byte ct for ascii array
				*ptr->pasc = '\0';	// Zero string terminator JIC a bozo forgot msg ends with '\n'
				ptr->seq = ptr->cmprs.cm[0];
				return 1;		// $$$$ COMPLETE & SUCCESS $$$$
			}
			else
			{ // Here, failed
				PC_msg_initg(ptr);	// Initialize struct for the next message
				PC_chksum_ct_err += 1;	// Running count of checksum errors
				return -1;		// Return error code.
			}
		}			
		break;

	case CAN_PC_ESCAPE: // Possible escape data byte, or escape for next byte.
		if (ptr->prev == CAN_PC_ESCAPE)
		{
			strstuff(ptr, c); // Store binary, ascii, and count binary bytes
			c = ~CAN_PC_ESCAPE;		// Set "previous" to not an escape
		}
		break;

	default: // All other bytes come here.
			strstuff(ptr, c); // Store binary, ascii, and count binary bytes
		break;
	}
	/* Prevent buffer overflow. */
	if ( ptr->pasc >= (&ptr->asc[0] + PCTOGATEWAYSIZE - 3 ) ) ptr->pasc -= 2; // Hold at end with space to allow '\n' '\0'

	ptr->prev = c;	// Save previous char for byte stuffing check.

	return 0;	// Return: binary fram not complete
}
/* **************************************************************************************
 * int PC_msg_getASCII(struct PCTOGATEWAY* ptr, u8 c);
 * @brief	: Build message from incoming ASCII/HEX bytes.  Binary data including seq number goes
 *              : int ptr.cmprs struct.  ASCII data goes into ptr->asc[]
 * @param	: ptr = Pointer to msg buffer (see common_can.h)
 *		:  ptr->cmprs.cm[] = binary bytes (including seq number, but not checksum)
 *		:  ptr->cmprs.ct = count of binary bytes (including seq number, but not checksum)
 *		:  ptr->cmprs.seq = 1 byte sequence number
 *		:  ptr->asc[] = ascii/hex bytes as received, including '\n' plus a '\0'
 *		:  ptr->ctasc = count of ascii chars = (2* binary ct + 1);
 *		:  ptr->seq = sequence number from 1st two chars
 * @param	: c = byte to add msg being built
 * @return	:  1 = completed; ptr->ct holds byte count
 *              :  0 = line not complete; 
 *              : -1 = completed, but bad checksum
 *  		: -2 = completed, but too few bytes to be a valid CAN msg
 *		: -3 = char count exceeds the max size for a max size CAN msg
 *		: -4 = not even number of hex incoming bytes (less newline)
 * ************************************************************************************** */
/*  Format of line returned in ptr->c[]:
incoming ascii expects is--
	seq,asciihex...,checksum, newline
(seq = one byte sequence number converted to ascii/hex)
(note: checksum is on the *binary* not ascii data):~/GliderWinch/sensor/gateway_ftdi/trunk
and converts to binary data returned in--
	ptr->c[]
where--
	ptr->ct = count of binary bytes of data.
*/

/* Lookup table to convert one hex char to binary (4 bits), no checking for illegal incoming hex */
const u8 hxbn[256] = {
/*          0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15   */
/*  0  */   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/*  1  */   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/*  2  */   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/*  3  */   0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  0,  0,  0,  0,  0,  0,
/*  4  */   0, 10, 11, 12, 13, 14, 15,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/*  5  */   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/*  6  */   0, 10, 11, 12, 13, 14, 15,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/*  7  */   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/*  8  */   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/*  9  */   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/* 10  */   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/* 11  */   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/* 12  */   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/* 13  */   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/* 14  */   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/* 15  */   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
};
 
//u8 unhex(char* p)
//{
//	u8 x = hxbn[(u8)(*p++)] << 4;// Get hi nibble of byte
//	x |= hxbn[(u8)(*p)];		// Or in low nibble
//	return x;
//}
int debugR;

/* Note: It is up to the caller to have the struct initialized, initially and after the 
message has been "consumed."  When there are errors this routine re-initializes.  */
int PC_msg_getASCII(struct PCTOGATEWAY* ptr, u8 c)
{			
	u8 x;
u8 zz;

//printf("%c",c);
//debugR += 1;
	

	/* End of msg check */
	if (c == '\n')
	{ // Newline ends the message

//int w;
//int i = ((ptr->ctasc - 0) >> 1);
//xprintf (6,"in ct: %i debugR: %i ctasc: %i \n",i, debugR, ptr->ctasc);
//for (w = 0; w < i; w++)
//xprintf(6,"%02x ",ptr->cmprs.cm[w]);
//xprintf(6,"\n\r");
//debugR = 0;
		*ptr->pasc++ = c;	// Store '\n'
		ptr->ctasc -= 2; // Adjust for not storing seq number bytes, but added '\n'

		ptr->seq = ptr->cmprs.cm[0];		// First binary byte is the sequence number
		ptr->cmprs.seq = ptr->cmprs.cm[0];	// Save for binary folk

		/* Check count for even pairing of hex char pairs */
		if ((ptr->ctasc & 0x1) != 0) // Should be odd at this point
		{ // Here, not an even pairing for the binary bytes
			PC_msg_initg(ptr);	// Initialize struct for the next message
			PC_oddeven_ct_err += 1;	// Running count of this type of error.
			return -4;	// Return error code.		{	
		}

		if (ptr->ctasc < 5)		// Too few incoming bytes to comprise a valid anything?
		{ // Here yes.
			PC_msg_initg(ptr);	// Initialize struct for the next message
			PC_toofew_ct_err += 1;	// Running count of this type of error.
			return -2;	// Return error code.
		}

		/* Binary byte count is the ascii ct divided by 2, less checksum */
		ptr->cmprs.ct = (ptr->ctasc >> 1);	// Number of binary bytes of msg data
					
		/* Check checksum. */	
		x = ptr->cmprs.cm[ptr->cmprs.ct];	// Checksum received
		zz=CANgenchksum(&ptr->cmprs.cm[0], (ptr->cmprs.ct));	// Checksum computed
//xprintf(6," CHKRCV %02x CMP %02x\n\r",x,zz);

		/* Compute checksum and compare. */
		if (zz  == x)
		{ // Here checksum good
				return 1;	// $$$$ COMPLETE & SUCCESS $$$$
		}
		else
		{ // Here, failed
//static u8 xx;:~/GliderWinch/sensor/gateway_ftdi/trunk
//xx = CANgenchksum(&ptr->cmprs.cm[0],ptr->cmprs.ct);
//printf("CHKSM: %02x %02x %02x  ptr->cmprs.ct: %i ptr->ctasc: %i\n",x,xx,zz,ptr->cmprs.ct, ptr->ctasc);
//int j;
//for (j = 0; j < ptr->cmprs.ct+2; j++)
//  printf("%02x",ptr->cmprs.cm[j]);
//printf("\n");
			PC_msg_initg(ptr);	// Initialize struct for the next message
			PC_chksum_ct_err += 1;	// Running count of checksum errors
			return -1;	// Return error code.
		}
	}
	if ( (ptr->ctasc & 0x1) == 0) // Even?
	{ // Here yes.  Even chars -> hi ord nibble of byte
		*ptr->cmprs.p = (hxbn[c] << 4); // Convert hex char to bin 4 bit
	}
	else
	{ // Here, Odd chars -> low ord nibble of byte
		*ptr->cmprs.p++ |= hxbn[c];	// Add nibble.  Byte complete.  Advance pointer.
	}

	/* Copy incoming chars into asc buffer. */
	*ptr->pasc++ = c; 	// Store incoming chars char array (for those who want raw ascii lines)
	ptr->ctasc += 1;	// Count incoming chars

	/* Max length check */
	if (ptr->ctasc > 37) 
	{ // Here incoming chars exceed the max number for a max size CAN msg
		PC_msg_initg(ptr);	// Initialize struct for the next message
		return -3;		// Return an error code
	}

	return 0;
}

/* **************************************************************************************
 * void PC_msg_initg(struct PCTOGATEWAY* p);
 * @brief	: Initialize struct for building a gateway message from the PC
 * @param	: Pointer to gateway message wrapper (see common_can.h)
 * ************************************************************************************** */
void PC_msg_initg(struct PCTOGATEWAY* p)
{
	p->pasc = &p->asc[0];		// Pointer that will store incoming ascii chars
	p->cmprs.p = &p->cmprs.cm[0];	// Pointer that will store incoming binary bytes
	p->ctasc = 0;			// Byte counter
	p->ct = 0;			// Byte counter
	p->cmprs.ct = 0;		// Byte counter
	p->chk = CHECKSUM_INITIAL;	// Checksum initial value
	p->prev = ~CAN_PC_ESCAPE;	// Begin with received byte not an escape.
	return;
}
/* **************************************************************************************
 * int CAN_id_valid(u32 id);
 * @brief	: Check if CAN id (32b) holds a valid CAN bus id
 * @param	: id = CAN id
 * @return	:  1 = 29 bit is OK
 *		:  0 = 11 bit is OK
 *		: -1 = faux CAN msg id bit 0 on
 *              : -2 = faux CAN msg id: 11 bit id, but stray bits in remaining 18 bits
 * ************************************************************************************** */
int CAN_id_valid(u32 id)
{
	if ( (id & 0x1) != 0 ) return -1; // faux ID
	if ( (id & 0x4) == 0 ) // Check IDE (extended id bit)
	{ // Here, an 11 bit address is specified
		if ((id & 0x001ffff8) != 0) return -2; // 11 bit id with stray bits in extended field
		return 0; // Here, 11b is OK.
	}
	return 1;	// Here 29 bit is OK.
}

/* **************************************************************************************
 * int PC_msg_asctobin(struct struct CANRCVBUF* pbin, struct PCTOGATEWAY* ptr, char* pin);
 * @brief	: Convert an ascii/hex msg (no seq or chksum) to binary CAN struct format
 * @param	: pbin = pointer to output binary CAN msg
 * @param	: ptr = pointer to intermediate binary array
 * @param	: pin = pointer to ascii/hex input line 
 * @param	: Pointer to gateway message wrapper (see common_can.h)
 * ************************************************************************************** */
/* Format expected from input--
iiiiiiiicc1122334455667788\n
Where (binary):
iiiiiiiii -> u32 with id
cc -> payload byte ct (0 - 8)
11... -> payload bytes (0 - 8)
\n is the line terminator of the ascii input
*/
int PC_msg_asctobin(struct PCTOGATEWAY* ptr, char* pin)
{
	int ct = 0;
	u8 x;
	u32 id;

	u8* pcmprs = &ptr->cmprs.cm[0];
	PC_msg_initg(ptr);	// Reset pointers

	
	while  ( (!((*pin == 0) || (*pin == '\n'))) && (ct < PCTOGATEWAYSIZE)  )
	{
		if ( (ct & 0x1) == 0) // Even?
		{ // Here yes.  Even chars -> hi ord nibble of byte
			*pcmprs = (hxbn[(u8)(*pin)] << 4);
		}
		else
		{ // Here, Odd chars -> low ord nibble of byte
			*pcmprs++ |= hxbn[(u8)(*pin)];	// Add nibble.  Byte complete.  Advance pointer.
		}
		pin ++; ct +=1;
	}
	if (ct >= PCTOGATEWAYSIZE)	return -5; // Error: run-away (no terminator)

	/* Compute size of resulting binary msg. */
	ptr->cmprs.ct = (pcmprs - &ptr->cmprs.cm[0]);

	/* Validity check */
	x = (u8)(ptr->cmprs.cm[4]);		// Should be the dlc
	if ( x > 8 ) return -1;		// Return: too many bytes for payload count

	if (ptr->cmprs.ct != (x + 5)) return -2; // Size inconsistent with dlc count
	id = (u32)(ptr->cmprs.cm[0]);		// Get ID in binary word form
	if (CAN_id_valid(id) != 0) return -3;	// Bits in extended field, but 11 bit id specified
	
	return 0;
}
/* **************************************************************************************
 * int PC_msg_prep(u8* pout, int outsize, u8* pin, int ct);
 * @brief	: Convert input bytes into output with byte stuffing, checksum and framing
 * @param	: pout = pointer to bytes with stuffing and chksum added 
 * @param	: outsize = size of output buffer (to prevent overflow if too small)
 * @param	: pin = Pointer to bytes to send
 * @param	: ct = byte count to send (does not include frame bytes, chksum, or stuffing bytes)
 * @return	: number of bytes in prep'd message
 * ************************************************************************************** */
/*
Note, in the very worst-case the output may be over (2*ct+3) the size of the input.
*/
int PC_msg_prep(u8* pout, int outsize, u8* pin, int ct)
{
	u8 *p1 = pin;	// Redundant?
	u8 *p2 = pout;	// Working pointer
	u8 *p2e = pout + outsize; // End of output buffer pointer
	u8 chk;		// Checksum computed on input bytes
	int i;

	/* Compute chksum on input message. */
	chk = CANgenchksum(pin, ct);
	
	/* Set up CAN msg with byte stuffing in output buffer. */
	for (i = 0; i < ct; i++)
	{
		if ((*p1 == CAN_PC_FRAMEBOUNDARY) || (*p1 == CAN_PC_ESCAPE) )
		{
			*p2++ = (CAN_PC_ESCAPE); // Precede the following char with an escape byte
			if (p2 >= p2e) p2--;  // Prevent some bozo from overrunning the buffer.
		}
		*p2++ = *p1++;		// Place the real byte with precision.
		if (p2 >= p2e) p2 -= 1; // Prevent buffer overflow by a thoughtless scoundrel.
	}

	/* Set up chksum byte stuffing in output buffer. */
	if ((chk == CAN_PC_FRAMEBOUNDARY) || (chk == CAN_PC_ESCAPE) )
	{
		*p2++ = (CAN_PC_ESCAPE); // Precede following char with escape
		if (p2 >= p2e) p2--;  	// Prevent some jerk from jamming too many bytes.
	}
	*p2++ = chk;			// Quietly place the checksum.
	if (p2 >= p2e) p2--; 		// Prevent buffer overflow by some nefarious nerd.

	*p2++ = (CAN_PC_FRAMEBOUNDARY);	// Set up End of Frame byte

	return (p2 - pout);		// Return number of bytes in output
}
/* **************************************************************************************
 * int PC_msg_prepASCII(struct SERIALSENDTASKBCB** ppbcb, struct PCTOGATECOMPRESSED* p);
 * @brief	: Convert input binary bytes to ascii/hex output lines
 * @param	: ppbcb = pointer to pointer to control block w buffer
 * @param	: p = Pointer to struct with "stuff" used in conversions
 * @return	: number of bytes in message queued
 * ************************************************************************************** */
/*  Outgoing line format
SS1122....CC'\n' (all ascii except newline)
Where: SS = one byte sequence number
11 = 1st byte, 22 = 2nd byte, (ascii/hex)
CC = checksum byte (ascii/hex) (checksum is made on binary data, including sequence number byte)
*/

//int PC_msg_prepASCII(char* pout, int outsize, struct PCTOGATECOMPRESSED* p)
int PC_msg_prepASCII(struct SERIALSENDTASKBCB** ppbcb, struct PCTOGATECOMPRESSED* p)
{
	struct SERIALSENDTASKBCB* pbcb = *ppbcb;

	/* Block if this buffer is not available. */
	xSemaphoreTake(pbcb->semaphore, 0);

	char* pout = (char*)pbcb->pbuf;	// Output buffer pointer

	u8* pin = &p->cm[0];	// Pointer to CAN binary msg
	char *p2 = pout;	// Working pointer
	char *p2e = pout + pbcb->maxsize - 4; // End of output buffer pointer (less checksum and newline)
	u8 chk;			// Checksum computed on input bytes
	int i = 0;		// Counter for copying

	/* Compute checksum on binary message, including sequence number */
	chk = CANgenchksum(&p->cm[0], p->ct);
	
	/* Convert binary input message to ASCII/HEX output message */
	while ((p2 < p2e) && (i++ < p->ct)) 
		p2 = hex(p2, *pin++);

	/* Add checksum to line */
	p2 = hex(p2, chk);

	/* Add terminator we have chosen ('\n') */
	*p2++ = ASCIIMSGTERMINATOR;
//if (p->ct >= 16) while(1==1);
	pbcb->size = (p2 - pout);

	/* Place Buffer Control Block on queue to SerialTaskSend */
	vSerialTaskSendQueueBuf(ppbcb); // Place on queue

	return (p2 - pout);	// Return number of bytes in output	
}


/* **************************************************************************************
 * int CANcompress(struct PCTOGATECOMPRESSED* pout, struct CANRCVBUF* pin);
 * @brief	: Convert a "standard" format CAN msg into a compressed format byte array with byte ct.
 * @param	: pout = pointer to output w compressed msg in a byte array
 * @param	: pin = pointer to input with a "standard" format CAN msg
 * @return	:  0 = OK; 
 *		: -1 = bogus dlc count with 29b id msg
 *		: -2 = bogus dlc count with 11b id msg
 *		: pout->ct = total number of bytes in compressed msg
 *		: pout->c[] = msg bytes
 * Note: see '../svn_discoveryf4/docs/trunk/Userdocs/gateway_format.txt'
 * Note: 'cm[0] holds the sequence number
 * ************************************************************************************** */
static void strwrd(u8* pout, u32 x)
{ // Store u32 in byte array
	*pout++ = (u8)x; *pout++ = (x >> 8); *pout++ = (x >> 16); *pout = (x >> 24); return;
}
static void strhalfwrd(u8* pout, u32 x)
{ // Store u32 in byte array
	*pout++ = (u8)x; *pout++ = (x >> 8); return;
}
int CANcompress(struct PCTOGATECOMPRESSED* pout, struct CANRCVBUF* pin)
{

	u32 i;
	int ret;
	u32 tmp;
	/* Identify id types--11b, 29b, faux CAN id. */
	ret = CAN_id_valid(pin->id);
	if (ret < 0) return ret;	// Return: not a valid CAN bus msg id

	pout->cm[0] = pout->seq;	// Place sequence number ahead of CAN msg bytes

	tmp = (pin->dlc & 0xf);
	
	if (ret > 0)
	{ // Here, 29 bit id msg
		strwrd( &pout->cm[1], ( pin->id | 0x1 ) ); // Low ord bit ON

		if (tmp > 8) return -1;	// JIC a bogus count
		pout->cm[5] = (u8)tmp;

		for (i = 0; i < tmp; i++)	// Copy payload
			pout->cm[i+6] = pin->cd.u8[i];

		pout->ct = (tmp + 6);	// Save number of bytes in compressed msg
		return 0;
	}
	/* Here, 11 bit id msg */
	if (tmp > 8) return -2;		// JIC a bogus count

	pout->ct = (tmp + 3);	// Save number of bytes in compressed msg (w seq number)

	if ((pin->id & 0x2) != 0) // Check RTR bit
	{ // Here, give RTR a special dlc count
		tmp = 9; // RTR code 9 means dlc = 0
	}
	else
	{
		for (i = 0; i < tmp; i++) // Copy payload
			pout->cm[i+3] = pin->cd.u8[i];
	}
	/* 11b id plus dlc|rtr  */
	strhalfwrd( &pout->cm[1], (pin->id >> 16) );
	pout->cm[1] &= ~0x1f;
	pout->cm[1] |= (u8)(tmp << 1);	

	return 0;
}
/* **************************************************************************************
 * int CANcompress_G(struct PCTOGATECOMPRESSED* pout, struct CANRCVBUF* pcan);
 * @brief	: Convert a "standard" format CAN msg into Gonzaga format byte array with byte ct.
 * @param	: pout = pointer to output w compressed msg in a byte array
 * @param	: pcan = pointer to input with a "standard" format CAN msg
 * @return	:  0 = OK; 
 *		: -1 = bogus dlc count with 29b id msg
 *		: -2 = bogus dlc count with 11b id msg
 *		: pout->ct = total number of bytes in compressed msg
 *		: pout->c[] = msg bytes
 * Note: see '../svn_discoveryf4/docs/trunk/Userdocs/gateway_format.txt'
 * Note: 'cm[0] holds the sequence number
 * ************************************************************************************** */
int CANcompress_G(struct PCTOGATECOMPRESSED* pout, struct CANRCVBUF* pcan)
{
	u32 i;
	u32 tmp;

	pout->cm[0] = pout->seq;		// Place sequence number ahead of CAN msg bytes
	strwrd( &pout->cm[1], pcan->id );	// 
	tmp = (pcan->dlc & 0xf);

	if (tmp > 8) return -1;
	pout->cm[5] = tmp;

	for (i = 0; i < tmp; i++)	// Copy payload
		pout->cm[i+6] = pcan->cd.u8[i];

	pout->ct = (tmp + 6);	// Save number of bytes in compressed msg

	return 0;
}
/* **************************************************************************************
 * int CANuncompress(struct CANRCVBUF* pout,  struct PCTOGATECOMPRESSED* pin);
 * @brief	: Given a binary msg (w/o byte stuffing/framing), convert to "standard" CAN register format
 * @param	: pout = pointer to output w uncompressed msg in register format
 * @param	: pin = pointer to input with compressed (binary) msg
 * @return	:  0 = OK, and--
 *        	:    pin->seq = 1st byte of received (binary) frame
 *        	:    pout = struct filled with id, dlc, payload
 *		: negative = Some error--
 *		: -1 = Too few bytes to a valid 29b compressed msg
 *		: -2 = dlc: payload ct too large (> 8) in a 29 bit id msg
 *		: -3 = dlc doesn't match byte count in a 29 bit id msg
 *		: -4 = Too few bytes to a valid 11b compressed msg
 *		: -5 = dlc: payload ct too large (> 8) in a 11 bit id msg
 *		: -6 = dlc doesn't match byte count in a 11 bit id msg
 * Note: see '../svn_discoveryf4/docs/trunk/Userdocs/gateway_format.txt'
 * Note: 'cm[0] holds the sequence number
 * ************************************************************************************** */
static u32 getwd(u8* p) // Move non-aligned bytes into a 4 byte word
{
	return ((*(p+3) << 24) | (*(p+2) << 16) | (*(p+1) << 8) | (*p + 0));
}
static u32 gethalfwd(u8* p) // Move non-aligned bytes into a 2 byte half-word
{
	return ((*(p+1) << 8) | (*p + 0));
}

int CANuncompress(struct CANRCVBUF* pout,  struct PCTOGATECOMPRESSED* pin)
{
	u32 i;
	u32 tmp = 0;
	if ((pin->cm[1] & 0x01) != 0)	// Is this an 11 bit or 29 bit id?
	{ // Here, a 29 bit 
		if (pin->ct < 6) return -1;	// Too few bytes to a valid 29b compressed msg
		pout->id = getwd (&pin->cm[1]);	// Get 4 bytes into word format
		pout->id &= ~0x1;		// Low bit OFF (reserved for hardware transmit trigger)

		tmp = pin->cm[5];		// dlc byte
		if (tmp > 8)	return -2;	// dlc: payload ct too large (> 8)
		pout->dlc = tmp;
		
		if (pin->ct != (s16)(6 + tmp)) return -3; // dlc doesn't match byte count

		for (i = 0; i < tmp; i++)	// Copy payload
			pout->cd.u8[i] = pin->cm[i + 5];
		return 0;	// Success with 29 bit id msg.
	}
	/* Here, an 11 bit id. */
	if (pin->ct < 3) return -4;		// Too few bytes to a valid 11b compressed msg

	/* Extract id and dlc|rtr from half word */
	tmp = gethalfwd (&pin->cm[1]);		// Get 2 bytes into half word form
	pout->id = ((tmp << 16) & 0xffe00000);	// Positon 11b id with dlc stripped.
	tmp = ((tmp >> 1) & 0xf);		// Extract dlc embedded in id's low ord bits
	pout->dlc = tmp;			// Set payload byte ct

	if (tmp > 9)	return -5;		// dlc: payload ct too large (> 8)
	if (tmp == 9)				// Check for RTR
	{ // Here, dlc count code means RTR
		pout->id |= 0x2;		// Set RTR bit in id
		pout->dlc = 0;			// RTR only has zero payload ct
	}

	if (pin->ct != (s16)(3 + pout->dlc)) return -6; // dlc doesn't match byte count

	for (i = 0; i < pout->dlc; i++)		// Copy payload
		pout->cd.u8[i] = pin->cm[i + 3];

	return 0;	// Success with 11 bit id msg.
}
/* **************************************************************************************
 * int CANuncompress_G(struct CANRCVBUF* pout,  struct PCTOGATECOMPRESSED* pin);
 * @brief	: Given a binary msg (w/o byte stuffing/framing), convert to "standard" CAN register format
 * @param	: pout = pointer to output w uncompressed msg in register format
 * @param	: pin = pointer to input with compressed (binary) msg
 * @return	:  0 = OK, and--
 *        	:    pin->seq = 1st byte of received (binary) frame
 *        	:    pout = struct filled with id, dlc, payload
 *		: negative = Some error--
 *		: -1 = Too few bytes to a valid 29b compressed msg
 *		: -2 = dlc: payload ct too large (> 8) in a 29 bit id msg
 *		: -3 = dlc doesn't match byte count in a 29 bit id msg
 * Note: see '../svn_discoveryf4/docs/trunk/Userdocs/gateway_format.txt'
 * Note: 'cm[0] holds the sequence number
 * ************************************************************************************** */
int CANuncompress_G(struct CANRCVBUF* pout,  struct PCTOGATECOMPRESSED* pin)
{
	u32 i;
	u32 tmp = 0;

	if (pin->ct < 6) return -1;		// Too few bytes to a valid msg	
	pout->id = getwd (&pin->cm[1]);		// Get 4 bytes into word format
	tmp = pin->cm[5];			// dlc byte
	if (tmp > 8)	return -2;		// dlc: payload ct too large (> 8)
	if (pin->ct != (s16)(6 + tmp)) return -3; // dlc doesn't match byte count
	pout->dlc = tmp;
	for (i = 0; i < tmp; i++)		// Copy payload
		pout->cd.u8[i] = pin->cm[i + 6];
	return 0;	// Success with 29 bit id msg.
}

