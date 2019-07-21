/******************************************************************************
* File Name          : gateway_CANtoPC.c
* Date First Issued  : 01/19/2019
* Board              : FreeRTOS/STM32CubeMX
* Description        : Simplified gateway (binary) to PC (ascii/hex)
*******************************************************************************/
/*
Implements LINK_MODE 2 (see PC_gateway_comm and USB_PC_gateway, in svn_common/trunk)
and see gateway_format.txt in svn_discovery/docs/trunk/Userdocs)
*/
#include "gateway_CANtoPC.h"

static uint8_t seq = 0; // Running sequence number for checking for missing CAN msgs

/* **************************************************************************************
 * static char* hex(char *p, u8 c)	// Convert 'c' to hex, placing in output *p.
 * ************************************************************************************** */
/* bin to ascii lookup table */
static const char h[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
static uint8_t* hex(uint8_t *p, uint8_t c)	// Convert 'c' to hex, placing in output *p.
{
		*p++ = h[((c >> 4) & 0x0f)];	// Hi order nibble
		*p++ = h[(c & 0x0f)];		// Lo order nibble
		return p;			// Return new output pointer position
}
/* **************************************************************************************
 * void gateway_CANtoPC(struct SERIALSENDTASKBCB** ppbcb, struct CANRCVBUF* pcan);
 * @brief	: Convert CAN msg into ascii/hex in a buffer for SerialTaskSend
 * @param	: pycb = pointer to poiner to buffer control block w buffer and uart handle
 * @param	: pcan = CAN msg
 * @return	: 
 * ************************************************************************************** */
void gateway_CANtoPC(struct SERIALSENDTASKBCB** ppbcb, struct CANRCVBUF* pcan)
{
	struct SERIALSENDTASKBCB* pbcb = *ppbcb;
	int i;
	uint32_t x = CHECKSUM_INITIAL;
	uint8_t* pout = pbcb->pbuf; // Pointer into output buffer

	/* Convert sequence number */
	x += seq;	// Checksum;
	pout = hex(pout,seq);
	seq += 1;

	/* Convert CAN ID */
	x += (pcan->id >>  0) & 0xff; pout = hex(pout, (pcan->id >>  0));
	x += (pcan->id >>  8) & 0xff; pout = hex(pout, (pcan->id >>  8));
	x += (pcan->id >> 16) & 0xff; pout = hex(pout, (pcan->id >> 16));
	x += (pcan->id >> 24) & 0xff; pout = hex(pout, (pcan->id >> 24));

	/* Convert DLC */
	x += pcan->dlc & 0xf; pout = hex(pout, pcan->dlc);

	/* Convert payload */
	if ((pcan->dlc & 0xf) > 8) pcan->dlc = 8; // Prevent bogus runaway
	for (i = 0; i < pcan->dlc; i++)
	{
		x += pcan->cd.uc[i];
		pout = hex(pout, pcan->cd.uc[i]);
	}

	/* Complete checksum */
	x += (x >> 16);	// Add carries into high half word
	x += (x >> 16);	// Add carry if previous add generated a carry
	x += (x >> 8);  // Add high byte of low half word
	x += (x >> 8);  // Add carry if previous add generated a carry

	/* Convert checksum */
	pout = hex(pout, x);

	/* Frame terminator */
	*pout++ = ASCIIMSGTERMINATOR;
	
	/* String termination, jic */
	*pout = 0; // Note: no pointer advance

	/* Set byte count in output buffer */
	pbcb->size = pout - pbcb->pbuf;

	return;
}
#ifdef CHECKSUMCODEFORREFERENCE
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
#endif
