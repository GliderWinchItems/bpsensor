/******************************************************************************
* File Name          : gateway_PCtoCAN.c
* Date First Issued  : 02/10/2019
* Description        : Convert incoming PC ascii/hex CAN msgs to binary CAN format
*******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "gateway_PCtoCAN.h"
#include "malloc.h"

/*
struct GATEWAYPCTOCAN
{
	struct CANRCVBUF*	pcan; // Ptr into buffer for received CAN msg
	uint32_t chksumx;       // Checksum in progress
	uint8_t binseq;         // Received sequence number (binary)
	uint8_t ctrseq;			// Software maintained sequence number
	uint8_t state;          // State of decoding
	uint8_t error;          // Error code: 0 = no errors
	uint8_t bin;            // Bin byte in progress
	uint8_t odd;            // Nibble: Odd = 1, even = 0;
	uint8_t ctr;            // Data storing counter
};
*/

static void new_init(struct GATEWAYPCTOCAN* p)
{
	/* Initialize for new CAN msg construction */
	p->state   = 0;
	p->error   = 0;
	p->ctr     = 0;
	p->odd     = 1;
	p->chksumx = CHECKSUM_INITIAL;	// Checksum initial value
	return;
}
/* **************************************************************************************
 * struct GATEWAYPCTOCAN* gateway_PCtoCAN_init(struct SERIALRCVBCB* prbcb);
 * @brief	: Get decode block calloc'd and initialized
 * @param	: 
 * @return	: pointer: NULL = failed.
 * ************************************************************************************** */
struct GATEWAYPCTOCAN* gateway_PCtoCAN_init(struct SERIALRCVBCB* prbcb)
{
	struct GATEWAYPCTOCAN* p;

	/* Get control block for conversion to CAN */
	p = (struct GATEWAYPCTOCAN*)calloc(1,sizeof(struct GATEWAYPCTOCAN));
	if (p == NULL) return p;
	prbcb->pgptc = p; // Save ptr in BCB for unloading dma

	new_init(p);	// Initialize for new (first) CAN msg construction
	p->pcanp = (struct CANRCVBUFPLUS*)prbcb->pbegin;
	return p;
}
/* **************************************************************************************
 * void gateway_PCtoCAN_unloaddma(struct SERIALRCVBCB* prbcb);
 * @brief	: build CAN msgs and add to line buffers for dma data available
 * @return	: prbcb->pgptc->error:
 *          :      0  = no errors
 *				: (1<<0) |=  1 not assigned
 *          : (1<<1) |=  2 completed, but bad checksum
 *  		   : (1<<2) |=  4 line terminator and state sequence not complete
 *		      : (1<<3) |=  8 sequence number did not mismatch
 *		      : (1<<4) |= 10 too many chars
 *          : (1<<5) |= 20 DLC greater than 8 (too large)
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
const uint8_t hexbin[256] = {
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

void gateway_PCtoCAN_unloaddma(struct SERIALRCVBCB* prbcb)
{	// Here, a DMA interrupt means there is new data in the DMA buffer

	struct GATEWAYPCTOCAN* p = prbcb->pgptc;	// Easy to use ptr
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	uint16_t dmandtr;	// Number of data items remaining in DMA NDTR register
	int32_t diff;
	char c;
		
	/* Get number of data items remaining in DMA buffer "now" from DMA NDTR register. */
	dmandtr = __HAL_DMA_GET_COUNTER(prbcb->phuart->hdmarx); 

	/* Difference between where we are taking out chars, and where DMA is, or was, storing. */
	diff = prbcb->penddma - dmandtr - prbcb->ptakedma; 
	if (diff < 0)
	{ // Wrap around
		diff += prbcb->dmasize;
	}

	/* Remove DMA chars and construct CAN msg(s), using data available. */
	while (diff > 0) // Run until chars available are used up.
	{
		diff -= 1;
		c = *prbcb->ptakedma++; // XGet char from dma buffer and advance dma ptr
		if (prbcb->ptakedma == prbcb->penddma) prbcb->ptakedma = prbcb->pbegindma;
			
		/* CAN msg ascii/hex separated with LINETERMINATOR. */
		// 0x0D takes care of someone typing in stuff with minicom
		if ((c == 0XD) || (c == LINETERMINATOR))
		{ // Here End of Line
			
			/* End of line signals end of CAN msg; beginning of new.  */
			if (p->state != 7) // Did it end correctly?
			{ // Here, no.
				p->error |= (1<<2);	// Line terminator came at wrong place.
			}

			/* Give the user of the CAN msg some info. */
			p->pcanp->seq   = p->binseq;
			p->pcanp->error = p->error;

			/* Advance to beginning of next CAN msg (line) buffer */
			prbcb->padd += prbcb->linesize;	// Step ahead one line buffer length
			if (prbcb->padd == prbcb->pend) prbcb->padd = prbcb->pbegin;

			/* Notify originating task that a CAN msg is ready. */
			xTaskNotifyFromISR(prbcb->tskhandle, 
				prbcb->notebit,	/* 'or' bit assigned to buffer to notification value. */
				eSetBits,         /* Use the 'or' option */
				&xHigherPriorityTaskWoken );

			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

			/* Initialize for next CAN msg */
			new_init(p);

			/* CAN msg starts at beginning of next "line" buffer. */
			p->pcanp = (struct CANRCVBUFPLUS*)prbcb->padd;
		}
		else
		{ // Not end-of-line.  Convert ascii/hex to binary bytes
			if (p->odd == 1)
			{ // High order nibble
				p->odd = 0;
				p->bin = hexbin[(uint8_t)c] << 4; // Lookup binary, given ascii
			}
			else
			{ // Low order nibble completes byte
				p->odd = 1;
				p->bin |= hexbin[(uint8_t)c];
				
				/* Store binary bytes directly into CAN buffer location */
				/* Build checksum as we go. */
				switch(p->state)
				{
				case 0: // Sequence number
					p->binseq   = p->bin;
					p->chksumx += p->bin;
					p->state   += 1;
					break;
				case 1: // Low order byte of CAN id word
					p->pcanp->can.id   = (p->bin << 0);
					p->state   += 1;
					p->chksumx += p->bin;
					break;
				case 2:
					p->pcanp->can.id  |= (p->bin << 8);
					p->chksumx += p->bin;
					p->state   += 1;
					break;
				case 3:
					p->pcanp->can.id  |= (p->bin << 16);
					p->chksumx += p->bin;
					p->state   += 1;
					break;
				case 4: // High order byte of CAN id word
					p->pcanp->can.id  |= (p->bin << 24);
					p->chksumx += p->bin;
					p->state   += 1;
					break;
				case 5: // DLC byte -> CAN msg dlc word
					if (p->bin > 8)
					{ // DLC too large.
						p->bin = 8; //Do not overrun array!
						p->error |= (1<<5);
					}
					p->pcanp->can.dlc = p->bin;
					p->chksumx   += p->bin;
					p->state     += 1;
					break;
				case 6: // Fill data bytes per dlc
					if (p->ctr < p->pcanp->can.dlc)
					{
						p->pcanp->can.cd.uc[p->ctr] = p->bin;
						p->chksumx += p->bin;
						p->ctr     += 1;
						break;
					}
					/* Here, checksum check.  Complete checksum calculation. */
					p->chksumx += (p->chksumx >> 16); // Add carries into high half word
					p->chksumx += (p->chksumx >> 16); // Add carry if previous add generated a carry
					p->chksumx += (p->chksumx >> 8);  // Add high byte of low half word
					p->chksumx += (p->chksumx >> 8);  // Add carry if previous add generated a carry
					if ((p->chksumx & 0xff) != p->bin)
					{ // Here, checksums mismatch
						p->error |=  (1<<1);
					}

					/* Check for missing msgs. */
					p->ctrseq += 1;	// Advance software maintained sequence number
					if (p->binseq != p->ctrseq)
					{
						p->error  |= (1<<3);	// Sequence number mismatch
						p->ctrseq = p->binseq; // Reset
					}
					p->state = 7;
					break;

				case 7:
					p->error |= (1<<4); // Too many chars
					break;
				}		
			}
		}
	}
	return;
}
/* *************************************************************************
 * struct CANRCVBUFPLUS* gateway_PCtoCAN_getCAN(struct SERIALRCVBCB* pbcb);
 *	@brief	: Get pointer to next available CAN msg
 * @param	: pbcb = Pointer to Buffer Control Block
 * @return	: Pointer to CAN Plus msg buffer; NULL = no new msgs
 * *************************************************************************/
struct CANRCVBUFPLUS* gateway_PCtoCAN_getCAN(struct SERIALRCVBCB* pbcb)
{
	struct CANRCVBUFPLUS* p = NULL;

	/* Check no new lines. */
	if (pbcb->ptake == pbcb->padd) return p;
	p = (struct CANRCVBUFPLUS*)pbcb->ptake;

	/* Advance 'take' pointer w wraparound check. */
	pbcb->ptake += pbcb->linesize;
	if (pbcb->ptake >= pbcb->pend) pbcb->ptake = pbcb->pbegin;

	return p;
}


