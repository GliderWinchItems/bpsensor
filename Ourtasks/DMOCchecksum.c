/******************************************************************************
* File Name          : DMOCchecksum.c
* Date First Issued  : 03/05/2019
* Description        : Compute DMOC checksum
*******************************************************************************/
#include "DMOCchecksum.h"

/* *************************************************************************
 * uint8_t DMOCchecksum(struct CANRCVBUF* pcan);
 *	@brief	: Compute CAN msg payload checksum for DMOC
 * @param	: pcan = pointer to CAN msg to be sent 
 * @return	: checksum
 * *************************************************************************/
uint8_t DMOCchecksum(struct CANRCVBUF* pcan)
{
	uint8_t i;
	uint8_t sum;
	
	/* Checksum is based on a right justified 11b address. */
	sum = (pcan->id >> 21);

	for (i = 0; i < pcan->dlc; i++)
		sum += pcan->cd.uc[i];

	return ((int)256 - (sum + 3));
}

/* 
byte DmocMotorController::calcChecksum(CAN_FRAME thisFrame) {
    byte cs;
    byte i;
    cs = thisFrame.id;
    for (i = 0; i < 7; i++)
        cs += thisFrame.data.bytes[i];
    i = cs + 3;
    cs = ((int) 256 - i);
    return cs;
}
*/
