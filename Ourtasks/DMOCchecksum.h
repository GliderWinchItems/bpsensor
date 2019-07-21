/******************************************************************************
* File Name          : DMOCchecksum.h
* Date First Issued  : 03/05/2019
* Description        : Compute DMOC checksum
*******************************************************************************/

#ifndef __DMOCCHECKSUM
#define __DMOCCHECKSUM

#include <stdint.h>
#include "common_can.h"

/* *************************************************************************/
uint8_t DMOCchecksum(struct CANRCVBUF* pcan);
/*	@brief	: Compute CAN msg payload checksum for DMOC
 * @param	: pcan = pointer to CAN msg to be sent 
 * @return	: checksum
 * *************************************************************************/

#endif

