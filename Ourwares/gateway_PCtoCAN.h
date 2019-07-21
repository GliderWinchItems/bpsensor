/******************************************************************************
* File Name          : gateway_PCtoCAN.h
* Date First Issued  : 02/10/2019
* Description        : Convert incoming PC ascii/hex CAN msgs to binary CAN format
*******************************************************************************/

#ifndef __GATEWAYPCTOCAN
#define __GATEWAYPCTOCAN

#include "SerialTaskReceive.h"




/* ************************************************************************************** */
void gateway_PCtoCAN_unloaddma(struct SERIALRCVBCB* prbcb);
/* @brief	: build CAN msgs and add to line buffers for dma data available
 * @return	: prbcb->pgptc->error:
 *          :      0  = no errors
 *				: (1<<0) |=  1 not assigned
 *          : (1<<1) |=  2 completed, but bad checksum
 *  		   : (1<<2) |=  4 line terminator and state sequence not complete
 *		      : (1<<3) |=  8 sequence number did not mismatch
 *		      : (1<<4) |= 10 too many chars
 *          : (1<<5) |= 20 DLC greater than 8 (too large)
 * ************************************************************************************** */
struct GATEWAYPCTOCAN* gateway_PCtoCAN_init(struct SERIALRCVBCB* prbcb);
/* @brief	: Get decode block calloc'd and initialized
 * @param	: 
 * @return	: pointer: NULL = failed.
 * ************************************************************************************** */
struct CANRCVBUFPLUS* gateway_PCtoCAN_getCAN(struct SERIALRCVBCB* pbcb);
/*	@brief	: Get pointer to next available CAN msg
 * @param	: pbcb = Pointer to Buffer Control Block
 * @return	: Pointer to CAN Plus  msg buffer; NULL = no new msgs
 * *************************************************************************/

#endif

